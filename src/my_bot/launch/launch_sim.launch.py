import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory("ros_gz_sim")
    pkg_path = get_package_share_directory("my_bot")
    gz_launch_path = os.path.join(ros_gz_sim_pkg_path, "launch", "gz_sim.launch.py")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    pkg_path,
                    "launch",
                    "robot_state_publisher_launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    default_world = os.path.join(pkg_path, "worlds", "empty.world")
    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument(
        "world", default_value=default_world, description="World to load"
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = LaunchDescription(
        [
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.path.join(pkg_path, "models"),
            ),
            SetEnvironmentVariable(
                "GZ_SIM_PLUGIN_PATH",
                os.path.join(pkg_path, "plugins"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch_path),
                launch_arguments={
                    "gz_args": [
                        "-r ",  # -v4
                        world,
                    ],
                    "on_exit_shutdown": "True",
                }.items(),
            ),
        ]
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "my_bot",
            "-z",
            "0.1",
        ],
    )

    bridge_params = os.path.join(pkg_path, "config", "gz_bridge.yaml")
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
    #     package="ros_gz_bridge",
    #     executable="image_bridge",
    #     arguments=["/camera/image_raw"],
    # )

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            world_arg,
            gazebo,
            spawn_entity,
            ros_gz_bridge,
        ]
    )
