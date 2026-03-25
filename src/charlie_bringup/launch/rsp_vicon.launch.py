import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    bringup_pkg_name = "charlie_bringup"
    description_pkg_name = "charlie_description"

    robot_name = "vicon_urdf"
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ... Robot description (xacro -> URDF XML string) ...
    xacro_file = os.path.join(
        get_package_share_directory(description_pkg_name),
        robot_name,
        "robot.urdf.xacro",
    )
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
    )

    vicon_odom_node = Node(
        package='charlie_bringup', # Cambia esto al paquete donde guardes tu script python
        executable='vicon_to_odom_node', # Nombre configurado en setup.py
        name='vicon_to_odom',
        output='screen'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulation (Gazebo) clock if true",
            ),
            rsp,
        ]
    )
