import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    aebs_pkg_name = "charlie_aebs"

    # Cargamos parámetros desde YAML
    parameters_file = os.path.join(get_package_share_directory(aebs_pkg_name), 'config', 'parameters_aebs.yaml')

    aebs_node = Node(
        package='charlie_aebs',
        executable='aebs',
        name='aebs',
        output='screen',
        parameters=[parameters_file]
    )

    return LaunchDescription([
        aebs_node,
    ])