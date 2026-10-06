import os
import random
import re

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _start_randomized_simulation(context, ur_type, gazebo_gui, randomize_obstacles):
    package_share = FindPackageShare("ur3_llm_control").perform(context)
    source_world = os.path.join(package_share, "worlds", "llm_pick_place.sdf")
    world_file = source_world
    if randomize_obstacles.perform(context).lower() == "true":
        with open(source_world, "r", encoding="utf-8") as source:
            world = source.read()

        rng = random.SystemRandom()
        zone_positions = {
            "zone_a": (0.40, 0.14),
            "zone_b": (0.40, 0.00),
            "zone_c": (0.40, -0.14),
        }
        tabletop_positions = [
            (0.16, 0.24), (0.34, 0.24), (0.16, -0.24), (0.34, -0.24),
        ]
        assigned_starts = [(0.25, 0.14), (0.25, 0.00), (0.25, -0.14)]

        # At least one extra cube occupies a randomly selected zone every run.
        first_zone = rng.choice(list(zone_positions))
        first_xy = zone_positions[first_zone]
        second_candidates = [
            ("zone", name, xy) for name, xy in zone_positions.items()
            if name != first_zone and
            (xy[0]-first_xy[0])**2 + (xy[1]-first_xy[1])**2 >= 0.12**2
        ] + [
            ("table", "table", xy) for xy in tabletop_positions
            if all((xy[0]-x)**2 + (xy[1]-y)**2 >= 0.12**2
                   for x, y in [first_xy, *assigned_starts])
        ]
        _, second_zone, second_xy = rng.choice(second_candidates)
        placements = [(first_zone, first_xy), (second_zone, second_xy)]
        rng.shuffle(placements)

        for model_name, (label, (x, y)) in zip(("green_cube", "purple_cube"), placements):
            pattern = re.compile(
                rf'(<model\s+name="{model_name}">.*?<pose>)[^<]+(</pose>)', re.DOTALL)
            world, replacements = pattern.subn(
                rf'\g<1>{x:.3f} {y:.3f} 0.742 0 0 0\g<2>', world, count=1)
            if replacements != 1:
                raise RuntimeError(f"Could not randomize initial pose for {model_name}")
            print(f"[random_world] {model_name} -> {label} ({x:.2f}, {y:.2f})", flush=True)

        world_file = os.path.join("/tmp", f"ur3_llm_control_world_{os.getpid()}.sdf")
        with open(world_file, "w", encoding="utf-8") as output:
            output.write(world)

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gz"), "/launch/ur_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "description_package": "ur3_llm_control",
            "description_file": "ur_on_table.urdf.xacro",
            "runtime_config_package": "ur3_llm_control",
            "controllers_file": "ur_controllers.yaml",
            "launch_rviz": "false",
            "gazebo_gui": gazebo_gui,
            "world_file": world_file,
        }.items(),
    )
    return [simulation]


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    executor_delay = LaunchConfiguration("executor_delay")
    execute_motion = LaunchConfiguration("execute_motion")
    model = LaunchConfiguration("model")
    randomize_obstacles = LaunchConfiguration("randomize_obstacles")

    package_share = FindPackageShare("ur3_llm_control")
    scene_config = PathJoinSubstitution([package_share, "config", "scene.yaml"])
    llm_config = PathJoinSubstitution([package_share, "config", "llm.yaml"])
    student_config = PathJoinSubstitution([package_share, "config", "student.yaml"])
    kinematics_config = PathJoinSubstitution(
        [package_share, "config", "kinematics.yaml"]
    )
    simulation = OpaqueFunction(
        function=_start_randomized_simulation,
        kwargs={"ur_type": ur_type, "gazebo_gui": gazebo_gui,
                "randomize_obstacles": randomize_obstacles},
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "description_package": "ur3_llm_control",
            "description_file": "ur_on_table.urdf.xacro",
            "moveit_config_package": "ur3_llm_control",
            "moveit_config_file": "ur_with_gripper.srdf.xacro",
            "use_sim_time": "true",
            "launch_rviz": "false",
            "launch_servo": "false",
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "rviz.launch.py"])
        ),
        condition=IfCondition(launch_rviz),
        launch_arguments={"ur_type": ur_type, "use_sim_time": "true"}.items(),
    )

    camera_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="camera_bridge",
        output="screen", arguments=[
            "/table_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/table_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/table_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/wrist_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/wrist_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
    )
    camera_state = Node(
        package="ur3_llm_control", executable="camera_state", output="screen",
        parameters=[{"use_sim_time": True}],
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )
    gripper_controller = TimerAction(period=8.0, actions=[gripper_spawner])
    open_gripper_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_spawner,
            on_exit=[
                Node(
                    package="ur3_llm_control",
                    executable="gripper_startup",
                    name="gripper_startup",
                    output="screen",
                    condition=IfCondition(execute_motion),
                )
            ],
        )
    )

    planner = Node(
        package="ur3_llm_control",
        executable="llm_planner",
        name="llm_planner",
        output="screen",
        parameters=[llm_config, student_config, {"model": model, "use_sim_time": True}],
    )

    executor = TimerAction(
        period=executor_delay,
        actions=[
            Node(
                package="ur3_llm_control",
                executable="skill_executor",
                name="skill_executor",
                output="screen",
                parameters=[
                    scene_config,
                    kinematics_config,
                    {"execute_motion": execute_motion},
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3e",
                choices=["ur3", "ur3e"],
                description="UR robot model used by Gazebo and MoveIt.",
            ),
            DeclareLaunchArgument(
                "gazebo_gui", default_value="true", description="Show Gazebo GUI."
            ),
            DeclareLaunchArgument(
                "randomize_obstacles", default_value="true",
                choices=["true", "false"],
                description="Randomize green and purple cube starting positions on each Gazebo start.",
            ),
            DeclareLaunchArgument(
                "launch_rviz", default_value="false",
                description="Also start RViz in this launch; default false for a separate RViz process."
            ),
            DeclareLaunchArgument(
                "executor_delay",
                default_value="20.0",
                description="Seconds allowed for Gazebo and MoveIt startup.",
            ),
            DeclareLaunchArgument(
                "execute_motion",
                default_value="true",
                choices=["true", "false"],
                description="Plan only when false; execute trajectories when true.",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="ag/gemini-3.8-flash-high",
                description="9Router model ID.",
            ),
            simulation,
            moveit,
            rviz,
            camera_bridge,
            camera_state,
            open_gripper_after_spawn,
            gripper_controller,
            planner,
            executor,
        ]
    )
