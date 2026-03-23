import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'charlie_path_planner'
    
    # 1. Configurar la variable de tiempo de simulación
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 2. Buscar la ruta del archivo YAML
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'dijkstra.yaml'
    )

    # 3. Definir el nodo de Dijkstra
    dijkstra_node = Node(
        package=package_name,
        executable='dijkstra_node',          # <-- Asegúrate de que este sea el nombre en tu setup.py
        name='charlie_path_planner_dijkstra', # <-- IMPORTANTE: Debe coincidir con el nombre raíz en el YAML
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 4. Retornar la descripción del launch
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usa el reloj de simulación de Gazebo'
        ),
        dijkstra_node
    ])