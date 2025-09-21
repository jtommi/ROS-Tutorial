import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "my_bot"
    package_path = get_package_share_directory(package_name)

    logger = LaunchConfiguration("log_level")

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(package_path, "launch", "rsp.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true", "use_ros2_control": "true"}.items(),
    )

    # joystick = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             os.path.join(
    #                 package_path,
    #                 "launch",
    #                 "joystick.launch.py",
    #             )
    #         ]
    #     ),
    #     launch_arguments={"use_sim_time": "true"}.items(),
    # )

    default_world = os.path.join(package_path, "worlds", "obstacles.world")

    world = LaunchConfiguration("world")

    world_arg = DeclareLaunchArgument(
        "world", default_value=default_world, description="World to load"
    )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "my_bot", "-z", "0.1"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_cont",
            "--controller-ros-args",
            "-r /diff_cont/cmd_vel:=/cmd_vel",
        ],
        # ros_arguments=[
        #     "--log-level",
        #     logger,
        # ],
    )

    joint_broadcast_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster"],
    )

    bridge_params = os.path.join(package_path, "config", "gz_bridge.yaml")
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )

    # ros_gz_image_bridge = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=["/camera/image_raw"],
    # )

    # Launch them all!
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value=["debug"],
                description="Logging level",
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_broadcast_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_broadcast_spawner,
                    on_exit=[diff_drive_spawner],
                )
            ),
            rsp,
            world_arg,
            gazebo,
            spawn_entity,
            ros_gz_bridge,
            # joystick,
            # ros_gz_image_bridge,
        ]
    )
