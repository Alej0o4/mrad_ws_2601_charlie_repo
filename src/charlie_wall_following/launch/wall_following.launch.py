#librerias python para la ejecución de multiples nodos
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Buscar la ruta del archivo de configuración YAML
    package_name = 'charlie_wall_following'
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'wall_follower_params.yaml'
    )

    # 2. Definir el nodo dist_finder
    dist_finder_node = Node(
        package=package_name,
        executable='dist_finder', # El nombre del entry_points del setup.py
        name='dist_finder',       # Importante: Debe coincidir con el del YAML
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': True}  
        ]
    )

    # 3. Definir el nodo control
    control_node = Node(
        package=package_name,
        executable='control',     # El nombre del entry_points del setup.py
        name='control',           # Importante: Debe coincidir con el del YAML
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': True}  
        ]
    )

    # 4. Retornar la descripción
    return LaunchDescription([
        dist_finder_node,
        control_node
    ])