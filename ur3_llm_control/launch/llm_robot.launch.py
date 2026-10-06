from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    executor_delay = LaunchConfiguration("executor_delay")
    execute_motion = LaunchConfiguration("execute_motion")
    model = LaunchConfiguration("model")

    package_share = FindPackageShare("ur3_llm_control")
    world_file = PathJoinSubstitution([package_share, "worlds", "llm_pick_place.sdf"])
    scene_config = PathJoinSubstitution([package_share, "config", "scene.yaml"])
    llm_config = PathJoinSubstitution([package_share, "config", "llm.yaml"])
    student_config = PathJoinSubstitution([package_share, "config", "student.yaml"])
    kinematics_config = PathJoinSubstitution(
        [package_share, "config", "kinematics.yaml"]
    )

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
            "launch_rviz": launch_rviz,
            "launch_servo": "false",
        }.items(),
    )

    gazebo_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_pose_bridge",
        output="screen",
        arguments=[
            "/world/llm_robot/set_pose@ros_gz_interfaces/srv/SetEntityPose"
        ],
    )

    gripper_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=["gripper_controller", "-c", "/controller_manager"],
            )
        ],
    )

    planner = Node(
        package="ur3_llm_control",
        executable="llm_planner",
        name="llm_planner",
        output="screen",
        parameters=[llm_config, student_config, {"model": model}],
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
                "launch_rviz", default_value="true", description="Show MoveIt RViz."
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
            gazebo_pose_bridge,
            gripper_controller,
            planner,
            executor,
        ]
    )
