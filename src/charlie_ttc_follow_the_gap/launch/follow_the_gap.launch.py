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
        'ttc_params.yaml'
    )

    # 2. Nodo de Percepción: Encuentra el hueco seguro
    ttc_gap_finder_node = Node(
        package=package_name,
        executable='ttc_gap_finder',
        name='ttc_gap_finder',
        output='screen',
        parameters=[config_file_path],
        # Remap si tu odometría viene de otro lado (ej: rf2o)
        remappings=[
            ('/odom', '/odom'), 
            ('/scan', '/scan')
        ]
    )

    # 3. Nodo de Control: Mueve el robot hacia el hueco
    ttc_control_node = Node(
        package=package_name,
        executable='ttc_control',
        name='ttc_control',
        output='screen',
        parameters=[config_file_path],
        # Remap: La salida va a la entrada "wall_following/ctrl" del Mux
        remappings=[
            ('/cmd_vel_ctrl', '/cmd_vel_ctrl') 
        ]
    )

    return LaunchDescription([
        ttc_gap_finder_node,
        ttc_control_node
    ])