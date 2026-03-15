import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Definir el nombre de tu paquete
    pkg_name = 'charlie_path_planner'
    
    # 2. Construir la ruta absoluta hacia tu archivo YAML
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'waypoints.yaml'
    )

    # 3. Configurar el nodo
    waypoint_manager_node = Node(
        package=pkg_name,
        executable='waypoint_manager',
        name='waypoint_manager', # Nombre del nodo en el grafo de ROS
        output='screen',         # Para ver los logs en la terminal
        parameters=[config_file] # ¡Aquí le pasamos el YAML!
    )

    # 4. Retornar la descripción del launch
    return LaunchDescription([
        waypoint_manager_node
    ])