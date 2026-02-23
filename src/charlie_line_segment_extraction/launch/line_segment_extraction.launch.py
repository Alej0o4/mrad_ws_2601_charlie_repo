import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Buscar la ruta del archivo de configuración YAML
    package_name = 'charlie_line_segment_extraction'
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'line_extractor_params.yaml'
    )

    # 2. Definir el nodo de extracción de líneas
    line_extractor_node = Node(
        package=package_name,
        executable='line_segment_extraction_node', # Nombre definido en setup.py entry_points
        name='line_segment_extraction_node', # Debe coincidir exactamente con el nombre raíz en el YAML
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': True}   # Forzar uso del tiempo de Gazebo
        ]
    )

    # 3. Retornar la descripción
    return LaunchDescription([
        line_extractor_node
    ])