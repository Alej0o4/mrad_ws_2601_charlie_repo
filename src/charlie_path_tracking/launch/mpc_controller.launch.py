import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Define el nombre de tu paquete (CÁMBIALO AL NOMBRE REAL DE TU PAQUETE)
    package_name = 'charlie_path_tracking'  # Reemplaza con el nombre real de tu paquete

    # 2. Construye la ruta absoluta hacia el archivo YAML
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mpc_params.yaml'
    )

    # 3. Define la configuración del nodo
    mpc_node = Node(
        package=package_name,
        executable='mpc_controller_node', # El nombre del ejecutable definido en tu setup.py
        name='mpc_controller_node',       # Debe coincidir con la raíz del YAML
        output='screen',                  # Imprime los get_logger() en la terminal
        parameters=[
            config_file_path,
            {'use_sim_time': True}]     # Inyecta el archivo de parámetros
    )

    # 4. Retorna la descripción del lanzamiento
    return LaunchDescription([
        mpc_node
    ])