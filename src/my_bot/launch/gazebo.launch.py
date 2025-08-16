from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory("ros_gz_sim")
    pkg_path = FindPackageShare("my_bot")  # Replace with your own package name
    gz_launch_path = PathJoinSubstitution(
        [ros_gz_sim_pkg_path, "launch", "gz_sim.launch.py"]
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                PathJoinSubstitution([pkg_path, "models"]),
            ),
            SetEnvironmentVariable(
                "GZ_SIM_PLUGIN_PATH",
                PathJoinSubstitution([pkg_path, "plugins"]),
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
            # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    # "/example_imu_topic@sensor_msgs/msg/Imu@gz.msgs.IMU",
                ],
                remappings=[
                    # ("/example_imu_topic", "/remapped_imu_topic"),
                ],
                output="screen",
            ),
        ]
    )
