import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Definimos el nombre del paquete
    pkg_name = 'charlie_sysid'

    # 2. Ruta por defecto al archivo YAML (ejemplo: session_a)
    default_yaml_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'session_general.yaml'
    )

    # 3. Declaramos el argumento de consola
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_yaml_path,
        description='Ruta absoluta al archivo YAML de la sesión VICON'
    )

    # 4. Creamos el nodo y le pasamos el argumento como parámetro
    sysid_node = Node(
        package=pkg_name,
        executable='sysid_excitation_node', # Nombre que le diste en tu setup.py
        name='sysid_excitation_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_arg,
        sysid_node
    ])