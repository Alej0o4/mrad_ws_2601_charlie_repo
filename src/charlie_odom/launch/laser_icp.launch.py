import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "charlie_odom"
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "laser_icp_params.yaml",
    )

    laser_icp_node = Node(
        package=package_name,
        executable="laser_icp_node",
        name="laser_icp_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation/Gazebo clock if true",
        ),
        laser_icp_node,
    ])
