import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
                        "empty.sdf"
                        # PathJoinSubstitution([pkg_path, "worlds/example_world.sdf"])
                    ],  # Replace with your own world file
                    "on_exit_shutdown": "True",
                }.items(),
            ),
        ]
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_spawn_model.launch.py",
                )
            ]
        ),
        launch_arguments={
            "world": "empty",
            "topic": "robot_description",
            "entity_name": "my_bot",
            "x": "5.0",
            "y": "5.0",
            "z": "0.5",
            "on_exit_shutdown": "True",
        }.items(),
    )

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
        ]
    )
