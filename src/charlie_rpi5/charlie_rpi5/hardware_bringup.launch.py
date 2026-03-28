import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Definir el paquete donde están tus archivos (ajusta el nombre si es necesario)
    package_name = 'ros2_ws_2601' # <-- Cambia esto al paquete donde esté tu YAML y tu script de Python

    # 2. Construir la ruta absoluta al archivo YAML
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'hardware_params.yaml'
    )

    # 3. Configurar el nodo con el archivo de parámetros
    twist_cmd_node = Node(
        package=package_name, # <-- Asegúrate que coincida con el paquete en tu setup.py
        executable='twist_cmd_node', # <-- El nombre del ejecutable definido en setup.py (entry_points)
        name='twist_cmd_node_charlie',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        twist_cmd_node
    ])