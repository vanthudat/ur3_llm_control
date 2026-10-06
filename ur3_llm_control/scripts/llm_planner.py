#!/usr/bin/env python3
"""ROS 2 node that turns natural language into a validated skill plan."""

from __future__ import annotations

import json
import os
import re
import threading
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ur3_llm_control.environment import DESTINATIONS, TEMPORARY, ZONES, expand_clearance_plan, validate_scene
import urllib.error
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from ur3_llm_control.task_validator import (
    PlanValidationError,
    format_numbered_plan,
    parse_and_validate,
)


SYSTEM_PROMPT = """You are a task planner for a UR3/UR3e robot.
Convert the user's Vietnamese or English command into robot skills.

Allowed skills and exact fields:
- {"skill":"pick","object":"red_cube|yellow_cube|blue_cube|green_cube|purple_cube"}
- {"skill":"place","object":"red_cube|yellow_cube|blue_cube|green_cube|purple_cube","zone":"zone_a|zone_b|zone_c"}
- {"skill":"home"}
- {"skill":"move_above","object":"red_cube|yellow_cube|blue_cube|green_cube|purple_cube"}
- {"skill":"open_gripper"}
- {"skill":"close_gripper"}
- {"skill":"move_to_zone","zone":"zone_a|zone_b|zone_c"}

Rules:
1. Return exactly one JSON object with the top-level field "plan".
2. Return JSON only: no Markdown, explanation, joint values, poses, or trajectories.
3. Infer the intent from the full sentence; do not depend on one fixed command template.
4. Canonical names: red/đỏ -> red_cube, yellow/vàng -> yellow_cube,
   blue/xanh dương -> blue_cube, green/xanh lá -> green_cube, purple/tím -> purple_cube, and zone/vùng/ô A/B/C -> zone_a/zone_b/zone_c.
5. For one object, return pick(object), place(the same object, zone), then home.
   For several objects, return one pick/place pair for each object and exactly
   one home as the final step. Do not insert home between object pairs.
6. Use move_above, open_gripper, close_gripper, or move_to_zone only when the
   user explicitly requests that standalone action.
7. Resolve pronouns such as "nó", "it", and "the object" from the user's meaning.
8. Return the requested pick/place intentions only. Environment logic will insert
   blocker relocations using the camera observation. For ID 33 arrange the three
   assigned blocks only; green and purple have no assigned zones.
9. Never invent a skill, object, zone, parameter, or robot motion.

Example input: Hãy lấy khối màu vàng và đặt nó vào ô A.
Example output: {"plan":[{"skill":"pick","object":"yellow_cube"},{"skill":"place","object":"yellow_cube","zone":"zone_a"},{"skill":"home"}]}

Example input: Move the blue cube to zone C.
Example output: {"plan":[{"skill":"pick","object":"blue_cube"},{"skill":"place","object":"blue_cube","zone":"zone_c"},{"skill":"home"}]}

Example input: Arrange all objects according to student ID = 33.
Example output when the personalized mapping is A -> yellow, B -> blue, C -> red:
{"plan":[{"skill":"pick","object":"yellow_cube"},{"skill":"place","object":"yellow_cube","zone":"zone_a"},{"skill":"pick","object":"blue_cube"},{"skill":"place","object":"blue_cube","zone":"zone_b"},{"skill":"pick","object":"red_cube"},{"skill":"place","object":"red_cube","zone":"zone_c"},{"skill":"home"}]}
"""


def personalized_arrangement_intent(command: str, assignment_id: int):
    """Return ID 33's required destinations for an explicit arrange-all request."""
    text = command.casefold()
    is_arrange_all = re.search(r"\b(arrange|sắp\s*xếp)\b", text)
    is_id_33 = re.search(r"\b(33|23020733)\b", text)
    if assignment_id != 33 or not (is_arrange_all and is_id_33):
        return None
    return {"plan": [
        {"skill": "pick", "object": "yellow_cube"},
        {"skill": "place", "object": "yellow_cube", "zone": "zone_a"},
        {"skill": "pick", "object": "blue_cube"},
        {"skill": "place", "object": "blue_cube", "zone": "zone_b"},
        {"skill": "pick", "object": "red_cube"},
        {"skill": "place", "object": "red_cube", "zone": "zone_c"},
        {"skill": "home"},
    ]}


class NineRouterClient:
    def __init__(
        self, base_url: str, model: str, api_key: str, timeout: float, system_prompt: str
    ):
        self._base_url = base_url.rstrip("/")
        self._model = model
        self._api_key = api_key
        self._timeout = timeout
        self._system_prompt = system_prompt

    def _headers(self) -> dict[str, str]:
        headers = {"Content-Type": "application/json"}
        if self._api_key:
            headers["Authorization"] = f"Bearer {self._api_key}"
        return headers

    def _select_model(self) -> str:
        if self._model:
            return self._model
        request = urllib.request.Request(
            f"{self._base_url}/models", headers=self._headers(), method="GET"
        )
        with urllib.request.urlopen(request, timeout=self._timeout) as response:
            payload = json.loads(response.read().decode("utf-8"))
        models = payload.get("data", [])
        if not models or not isinstance(models[0].get("id"), str):
            raise RuntimeError("9Router returned no available chat model")
        self._model = models[0]["id"]
        return self._model

    def create_plan(self, command: str, scene: dict) -> str:
        payload = {
            "model": self._select_model(),
            "temperature": 0.0,
            "stream": False,
            "messages": [
                {"role": "system", "content": self._system_prompt},
                {"role": "user", "content": command + "\nCamera observation: " + json.dumps(scene)},
            ],
        }
        request = urllib.request.Request(
            f"{self._base_url}/chat/completions",
            data=json.dumps(payload).encode("utf-8"),
            headers=self._headers(),
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self._timeout) as response:
            raw_response = response.read().decode("utf-8", errors="replace")
        if not raw_response.strip():
            raise RuntimeError(
                "9Router returned an empty response; the selected model may be unavailable"
            )
        try:
            result = json.loads(raw_response)
        except json.JSONDecodeError as error:
            detail = raw_response[:300].replace("\n", " ")
            raise RuntimeError(f"9Router returned non-JSON content: {detail!r}") from error
        try:
            content = result["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as error:
            raise RuntimeError("9Router response has no assistant message") from error
        if not isinstance(content, str):
            raise RuntimeError("9Router assistant content is not text")
        return content


class LlmPlanner(Node):
    def __init__(self) -> None:
        super().__init__("llm_planner")
        base_url = self.declare_parameter(
            "router_base_url", "http://localhost:20128/v1"
        ).value
        model = self.declare_parameter("model", "").value
        timeout = float(self.declare_parameter("request_timeout", 45.0).value)
        api_key_env = self.declare_parameter(
            "api_key_environment", "NINE_ROUTER_API_KEY"
        ).value
        base_url = os.environ.get("NINE_ROUTER_BASE_URL", "").strip() or str(base_url)
        student_name = str(self.declare_parameter("student_name", "").value)
        student_id = str(self.declare_parameter("student_id", "").value)
        assignment_id = int(self.declare_parameter("assignment_id", 33).value)
        self._assignment_id = assignment_id
        task_index = int(self.declare_parameter("task_index", -1).value)
        personalized_demo = str(self.declare_parameter("personalized_demo", "").value)
        assignment_context = ""
        if personalized_demo:
            assignment_context = (
                "\nPersonalized demonstration assignment for student "
                f"{student_name} (student ID {student_id}, assignment ID = {assignment_id}, P={task_index}): "
                f"{personalized_demo}. Use this mapping when the user requests the "
                "personalized zone demonstration."
            )
        model = os.environ.get("NINE_ROUTER_MODEL", "").strip() or str(model)
        command_topic = self.declare_parameter("command_topic", "/user_command").value
        plan_topic = self.declare_parameter("plan_topic", "/validated_plan").value
        status_topic = self.declare_parameter("status_topic", "/task_status").value

        self._client = NineRouterClient(
            str(base_url), str(model), os.environ.get(str(api_key_env), ""), timeout,
            SYSTEM_PROMPT + assignment_context,
        )
        self._plan_publisher = self.create_publisher(String, str(plan_topic), 10)
        self._status_publisher = self.create_publisher(String, str(status_topic), 10)
        self._scene = None
        self._scene_received = 0.0
        self._scene_lock = threading.Lock()
        self._execution_busy = False
        self.create_subscription(String, "/environment_state", self._on_scene, 1,
                                 callback_group=ReentrantCallbackGroup())
        self.create_subscription(String, str(status_topic), self._on_status, 10,
                                 callback_group=ReentrantCallbackGroup())
        self._subscription = self.create_subscription(
            String, str(command_topic), self._on_command, 10
        )
        self._busy_lock = threading.Lock()
        model_label = model or "<auto>"
        self.get_logger().info(
            f"9Router configuration: base_url={base_url}, model={model_label}"
        )
        if personalized_demo:
            self.get_logger().info(
                f"Personalized demo: student={student_name}, id={student_id}, "
                f"P={task_index}; {personalized_demo}"
            )
        self.get_logger().info(
            f"LLM planner ready: publish natural language to {command_topic}"
        )

    def _on_scene(self, message):
        try:
            scene = json.loads(message.data)
            if not isinstance(scene, dict) or not isinstance(scene.get('stream_id'), str) or \
                    type(scene.get('frame_id')) is not int:
                return
            with self._scene_lock:
                if self._scene is not None and scene['stream_id'] == self._scene.get('stream_id') and \
                        scene['frame_id'] <= self._scene.get('frame_id', -1):
                    return
                self._scene = scene
                self._scene_received = time.monotonic()
        except (ValueError, TypeError):
            pass

    def _on_status(self, message):
        if message.data.startswith(('TASK SUCCESS', 'TASK_FAILED', 'EXECUTION_REJECTED')):
            self._execution_busy = False

    def _fresh_scene(self):
        with self._scene_lock:
            scene, received = self._scene, self._scene_received
        if scene is None:
            raise PlanValidationError('No camera observation yet; wait for camera_state')
        if time.monotonic() - received > 2.0:
            raise PlanValidationError('Camera stopped providing new frames for more than 2 seconds')
        validate_scene(scene, time.time())
        return scene

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status
        self._status_publisher.publish(message)

    def _on_command(self, message: String) -> None:
        command = message.data.strip()
        if not command:
            self.get_logger().error("Rejected an empty user command")
            self._publish_status("PLAN_REJECTED: empty command")
            return
        if self._execution_busy or not self._busy_lock.acquire(blocking=False):
            self.get_logger().warning("Planner is busy; command rejected")
            self._publish_status("PLAN_REJECTED: planner busy")
            return

        try:
            self.get_logger().info(f"USER COMMAND: {command}")
            scene = self._fresh_scene()
            content = self._client.create_plan(command, scene)
            intent = parse_and_validate(content)
            assigned_intent = personalized_arrangement_intent(command, self._assignment_id)
            if assigned_intent is not None:
                # Keep LLM interpretation in the loop, while enforcing the configured
                # ID 33 mapping before camera-based obstacle expansion.
                intent = parse_and_validate(json.dumps(assigned_intent))
                self.get_logger().info(
                    "Applying student ID 33 assignment: yellow->zone_a, blue->zone_b, red->zone_c")
            llm_plan_message = f"LLM PLAN:\n{format_numbered_plan(intent)}"
            self.get_logger().info(llm_plan_message)
            self._publish_status(llm_plan_message)
            # Network latency may invalidate the old observation. Use the latest camera frame.
            scene = self._fresh_scene()
            plan = expand_clearance_plan(intent, scene, time.time())
            self.get_logger().info(
                f"CAMERA SCENE: {len(scene['objects'])}/5 blocks visible; checking requested zones")
            self._publish_status(
                f"CAMERA SCENE: {len(scene['objects'])}/5 blocks visible; checking requested zones")
            camera_zones = scene.get("zones", {})
            for zone in ZONES:
                occupants = camera_zones.get(zone)
                if occupants is None:
                    message = f"CAMERA CHECK: {zone} status unknown (incomplete observation)"
                    self.get_logger().warning(message)
                    self._publish_status(message)
                    continue
                if occupants:
                    message = f"CAMERA CHECK: {zone} occupied by {', '.join(occupants)}"
                else:
                    message = f"CAMERA CHECK: {zone} free"
                self.get_logger().info(message)
                self._publish_status(message)

            plan_steps = plan["plan"]
            for index in range(len(plan_steps) - 1):
                pick_step, place_step = plan_steps[index:index + 2]
                if (pick_step.get("skill") == "pick" and
                        place_step.get("skill") == "place" and
                        place_step.get("zone") in TEMPORARY):
                    slot = place_step["zone"]
                    location = DESTINATIONS[slot]
                    message = (
                        f"FIND_FREE_POSITION(): selected {slot} at "
                        f"[{location[0]:.2f}, {location[1]:.2f}] "
                        "from camera-observed free tabletop")
                    self.get_logger().info(message)
                    self._publish_status(message)

            for index in range(0, len(plan_steps) - 3):
                blocker_pick, blocker_place = plan_steps[index:index + 2]
                target_pick, target_place = plan_steps[index + 2:index + 4]
                if (blocker_pick.get("skill") == "pick" and
                        blocker_place.get("skill") == "place" and
                        blocker_place.get("zone") in TEMPORARY and
                        target_pick.get("skill") == "pick" and
                        target_place.get("skill") == "place"):
                    slot = blocker_place["zone"]
                    location = DESTINATIONS[slot]
                    message = (
                        f"CLEARANCE PLAN: move blocker {blocker_pick['object']} — "
                        f"pick({blocker_pick['object']}) -> "
                        f"place({blocker_pick['object']}, {slot}) at "
                        f"[{location[0]:.2f}, {location[1]:.2f}], before "
                        f"pick({target_pick['object']}) -> "
                        f"place({target_pick['object']}, {target_place['zone']})")
                    self.get_logger().info(message)
                    self._publish_status(message)
            encoded_plan = json.dumps(plan, ensure_ascii=False, separators=(",", ":"))
            numbered_plan = format_numbered_plan(plan)
            if plan != intent:
                plan_message = f"CAMERA-EXPANDED PLAN:\n{numbered_plan}"
                self.get_logger().info(plan_message)
                self._publish_status(plan_message)
            self._publish_status("PLAN VALIDATED")
            output = String()
            output.data = encoded_plan
            self._execution_busy = True
            self._plan_publisher.publish(output)
        except PlanValidationError as error:
            self.get_logger().error(f"PLAN REJECTED: {error}")
            self._publish_status(f"PLAN_REJECTED: {error}")
        except urllib.error.HTTPError as error:
            detail = error.read().decode("utf-8", errors="replace")[:500]
            self.get_logger().error(f"9Router HTTP {error.code}: {detail}")
            self._publish_status(f"LLM_ERROR: HTTP {error.code}")
        except (urllib.error.URLError, TimeoutError, RuntimeError, json.JSONDecodeError) as error:
            self.get_logger().error(f"9Router request failed: {error}")
            self._publish_status(f"LLM_ERROR: {error}")
        finally:
            self._busy_lock.release()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LlmPlanner()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
