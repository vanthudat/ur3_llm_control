#!/usr/bin/env python3
"""Send one natural-language command and wait for the robot task result."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String


FAILURE_PREFIXES = (
    "PLAN_REJECTED:",
    "LLM_ERROR:",
    "EXECUTION_REJECTED:",
    "TASK_FAILED:",
)


class CommandClient(Node):
    def __init__(self, command_topic: str, status_topic: str) -> None:
        super().__init__("command_client")
        self.result: bool | None = None
        self.publisher = self.create_publisher(String, command_topic, 10)
        self.subscription = self.create_subscription(
            String, status_topic, self._on_status, 10
        )

    def _on_status(self, message: String) -> None:
        status = message.data
        print(status, flush=True)
        if status == "TASK SUCCESS":
            self.result = True
        elif status.startswith(FAILURE_PREFIXES):
            self.result = False


def parse_arguments(args: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a natural-language command to ur3_llm_control."
    )
    parser.add_argument("command", nargs="+", help="Vietnamese or English robot command")
    parser.add_argument(
        "--timeout",
        type=float,
        default=300.0,
        help="Maximum seconds to wait for the complete robot task",
    )
    parser.add_argument("--command-topic", default="/user_command")
    parser.add_argument("--status-topic", default="/task_status")
    return parser.parse_args(args)


def main(args=None) -> int:
    ros_args = sys.argv if args is None else [sys.argv[0], *args]
    options = parse_arguments(remove_ros_args(args=ros_args)[1:])
    command = " ".join(options.command).strip()
    if not command:
        print("RESULT: FAILED - empty command", file=sys.stderr)
        return 2
    if options.timeout <= 0.0:
        print("RESULT: FAILED - timeout must be positive", file=sys.stderr)
        return 2

    rclpy.init(args=ros_args)
    node = CommandClient(options.command_topic, options.status_topic)
    try:
        subscriber_deadline = time.monotonic() + 10.0
        while node.publisher.get_subscription_count() == 0:
            if time.monotonic() >= subscriber_deadline:
                print("RESULT: FAILED - llm_planner is not running", file=sys.stderr)
                return 2
            rclpy.spin_once(node, timeout_sec=0.1)

        message = String()
        message.data = command
        print(f"USER COMMAND: {command}", flush=True)
        node.publisher.publish(message)

        task_deadline = time.monotonic() + options.timeout
        while node.result is None and time.monotonic() < task_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.result is True:
            print("RESULT: SUCCESS")
            return 0
        if node.result is False:
            print("RESULT: FAILED", file=sys.stderr)
            return 1
        print("RESULT: FAILED - task timeout", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("RESULT: CANCELLED", file=sys.stderr)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
