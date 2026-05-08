import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Obtener la ruta de la carpeta config del paquete
    config_dir = os.path.join(
        get_package_share_directory('charlie_path_tracking'),
        'config',
        'pure_pursuit_ackermann.yaml'
    )

    # Nodo de Pure Pursuit Ackermann
    pure_pursuit_node = Node(
        package='charlie_path_tracking',
        executable='pure_pursuit_ackermann',
        name='pure_pursuit_ackermann_node',
        output='screen',
        parameters=[config_dir]
    )

    return LaunchDescription([
        pure_pursuit_node
    ])