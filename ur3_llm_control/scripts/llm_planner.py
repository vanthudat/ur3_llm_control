#!/usr/bin/env python3
"""ROS 2 node that turns natural language into a validated skill plan."""

from __future__ import annotations

import json
import os
import threading
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
- {"skill":"pick","object":"red_cube|yellow_cube|blue_cube"}
- {"skill":"place","object":"red_cube|yellow_cube|blue_cube","zone":"zone_a|zone_b|zone_c"}
- {"skill":"home"}
- {"skill":"move_above","object":"red_cube|yellow_cube|blue_cube"}
- {"skill":"open_gripper"}
- {"skill":"close_gripper"}
- {"skill":"move_to_zone","zone":"zone_a|zone_b|zone_c"}

Rules:
1. Return exactly one JSON object with the top-level field "plan".
2. Return JSON only: no Markdown, explanation, joint values, poses, or trajectories.
3. Infer the intent from the full sentence; do not depend on one fixed command template.
4. Canonical names: red/đỏ -> red_cube, yellow/vàng -> yellow_cube,
   blue/xanh dương -> blue_cube, and zone/vùng/ô A/B/C -> zone_a/zone_b/zone_c.
5. To move an object to a zone, return exactly pick(object),
   place(the same object, zone), then home.
6. Use move_above, open_gripper, close_gripper, or move_to_zone only when the
   user explicitly requests that standalone action.
7. Resolve pronouns such as "nó", "it", and "the object" from the user's meaning.
8. Never invent a skill, object, zone, parameter, or robot motion.

Example input: Hãy lấy khối màu vàng và đặt nó vào ô A.
Example output: {"plan":[{"skill":"pick","object":"yellow_cube"},{"skill":"place","object":"yellow_cube","zone":"zone_a"},{"skill":"home"}]}

Example input: Move the blue cube to zone C.
Example output: {"plan":[{"skill":"pick","object":"blue_cube"},{"skill":"place","object":"blue_cube","zone":"zone_c"},{"skill":"home"}]}
"""


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

    def create_plan(self, command: str) -> str:
        payload = {
            "model": self._select_model(),
            "temperature": 0.0,
            "stream": False,
            "messages": [
                {"role": "system", "content": self._system_prompt},
                {"role": "user", "content": command},
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
        task_index = int(self.declare_parameter("task_index", -1).value)
        personalized_demo = str(self.declare_parameter("personalized_demo", "").value)
        assignment_context = ""
        if personalized_demo:
            assignment_context = (
                "\nPersonalized demonstration assignment for student "
                f"{student_name} (ID {student_id}, P={task_index}): "
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
        if not self._busy_lock.acquire(blocking=False):
            self.get_logger().warning("Planner is busy; command rejected")
            self._publish_status("PLAN_REJECTED: planner busy")
            return

        try:
            self.get_logger().info(f"USER COMMAND: {command}")
            content = self._client.create_plan(command)
            plan = parse_and_validate(content)
            encoded_plan = json.dumps(plan, ensure_ascii=False, separators=(",", ":"))
            numbered_plan = format_numbered_plan(plan)
            self.get_logger().info(f"LLM PLAN:\n{numbered_plan}")
            self._publish_status(f"LLM PLAN:\n{numbered_plan}")
            self._publish_status("PLAN VALIDATED")
            output = String()
            output.data = encoded_plan
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
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
