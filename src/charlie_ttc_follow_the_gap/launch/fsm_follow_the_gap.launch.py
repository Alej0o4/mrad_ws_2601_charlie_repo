import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Definir nombre del paquete y rutas
    package_name = 'charlie_ttc_follow_the_gap'
    
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'fsm_params.yaml'
    )

    # 2. Nodo de Percepción: Encuentra el hueco seguro
    fsm_gap_finder_node = Node(
        package=package_name,
        executable='fsm_gap_finder',
        name='fsm_gap_finder',
        output='screen',
        parameters=[config_file_path],
    )

    # 3. Nodo de Control: Mueve el robot hacia el hueco
    fsm_control_node = Node(
        package=package_name,
        executable='fsm_control',
        name='fsm_control',
        output='screen',
        parameters=[config_file_path],
    )

    return LaunchDescription([
        fsm_gap_finder_node,
        fsm_control_node,
    ])