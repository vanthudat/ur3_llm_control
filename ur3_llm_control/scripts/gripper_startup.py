#!/usr/bin/env python3
"""Open the simulated gripper as soon as its controller becomes available."""

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


def main(args=None):
    rclpy.init(args=args)
    node = Node("gripper_startup")
    client = ActionClient(
        node,
        FollowJointTrajectory,
        "/gripper_controller/follow_joint_trajectory",
    )
    try:
        if not client.wait_for_server(timeout_sec=5.0):
            node.get_logger().warning(
                "Gripper controller did not become available for startup open; "
                "the skill executor will retry later."
            )
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            "gripper_finger_joint", "gripper_fixed_finger_joint"
        ]
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 800_000_000
        goal.trajectory.points = [point]

        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, goal_future, timeout_sec=5.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            node.get_logger().warning(
                "Startup open command was not accepted; the skill executor will retry later."
            )
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=5.0)
        result = result_future.result()
        if result is None or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            node.get_logger().warning(
                "Startup gripper open did not finish; the skill executor will retry later."
            )
        else:
            node.get_logger().info("Gripper opened during controller startup.")
    finally:
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
