"""Launch a lightweight robot view, or optionally the MoveIt RViz panel."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _make_rviz_node(context):
    package_share = FindPackageShare("ur3_llm_control").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    if rviz_config not in ("view_robot.rviz", "view_moveit.rviz"):
        raise RuntimeError(f"Unsupported RViz config: {rviz_config}")

    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            os.path.join(package_share, "urdf", "ur_on_table.urdf.xacro"),
            " name:=ur ur_type:=", ur_type,
        ]),
        value_type=str,
    )
    robot_description_semantic = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            os.path.join(package_share, "srdf", "ur_with_gripper.srdf.xacro"),
            " name:=ur",
        ]),
        value_type=str,
    )
    parameters = []
    if rviz_config == "view_moveit.rviz":
        parameters.append(os.path.join(package_share, "config", "kinematics.yaml"))
    parameters.append({
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
    })
    return [Node(
        package="rviz2", executable="rviz2", name="rviz2_ur3", output="screen",
        # RViz Humble's Ogre GLX renderer requires an X11/XWayland native
        # window handle; Qt's Wayland surface is not a valid GLX parent.
        additional_env={"QT_QPA_PLATFORM": "xcb"},
        arguments=["-d", os.path.join(package_share, "rviz", rviz_config)],
        parameters=parameters,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e", choices=["ur3", "ur3e"]),
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument(
            "rviz_config", default_value="view_robot.rviz",
            choices=["view_robot.rviz", "view_moveit.rviz"],
            description="Use the lightweight robot view or the MoveIt planning panel.",
        ),
        OpaqueFunction(function=_make_rviz_node),
    ])
