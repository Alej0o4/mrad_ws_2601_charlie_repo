import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Definir el paquete donde están tus cosas
    pkg_name = 'charlie_path_planner'  # <-- Cambia esto por el nombre de tu paquete real, si es diferente

    # 2. Construir la ruta absoluta hacia el archivo ara.yaml
    config_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ara.yaml'
    )

    # 3. Definir el nodo y pasarle el archivo de parámetros
    ara_planner_node = Node(
        package=pkg_name,
        executable='ara_star_node', # <-- ¡Ojo! Este debe ser el nombre definido en tu setup.py
        name='ara_star_node',
        output='screen',
        parameters=[config_file_path]
    )

    return LaunchDescription([
        ara_planner_node
    ])