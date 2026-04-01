import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. DECLARACIÓN DE ARGUMENTOS ---
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar', default_value='true',
        description='Activar el escáner LiDAR'
    )
    
    enable_aebs_arg = DeclareLaunchArgument(
        'enable_aebs', default_value='true',
        description='Activar el sistema AEBS (Requiere enable_lidar:=true)'
    )

    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_aebs = LaunchConfiguration('enable_aebs')

    # --- 2. LÓGICA CONDICIONAL ---
    # El AEBS solo arranca si explícitamente se pide y el LiDAR está encendido
    aebs_is_active_expr = PythonExpression([
        "'", enable_aebs, "' == 'true' and '", enable_lidar, "' == 'true'"
    ])

    # El ESC de los motores escucha al AEBS si este está activo, si no, al raw
    twist_cmd_topic_expr = PythonExpression([
        "'/cmd_vel_aebs' if (", aebs_is_active_expr, ") else '/cmd_vel_raw'"
    ])

    # --- 3. RUTAS DE ARCHIVOS (YAMLs y Launches externos) ---
    hardware_params_file = os.path.join(
        get_package_share_directory('yb_eb_pkg'), 'config', 'hardware_params.yaml'
    )
    
    aebs_params_file = os.path.join(
        get_package_share_directory('charlie_aebs'), 'config', 'parameters_aebs.yaml'
    )

    # Ruta al archivo launch original del fabricante del LiDAR
    # (Ajusta 'sllidar_launch.py' si el archivo del proveedor tiene otro nombre)
    sllidar_launch_file = os.path.join(
        get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_launch.py'
    )


    # --- 4. DEFINICIÓN DE NODOS Y LAUNCHES INCORPORADOS ---

    # A. Motores (Con remapeo dinámico)
    twist_cmd_node = Node(
        package='yb_eb_pkg',
        executable='twist_cmd_node',
        name='twist_cmd_node_charlie',
        output='screen',
        parameters=[
            hardware_params_file,
            {'cmd_vel_topic': twist_cmd_topic_expr} 
        ]
    )

    # B. LiDAR (Llamando al launch del proveedor)
    lidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch_file),
        condition=IfCondition(enable_lidar) # La condición se aplica a todo el launch externo
        # Si el launch del proveedor necesita argumentos (ej. puerto serial), puedes pasarlos aquí:
        # launch_arguments={'serial_port': '/dev/ttyUSB0'}.items()
    )

    # C. AEBS (Con su propio archivo YAML y remapeo)
    aebs_node = Node(
        package='charlie_aebs', 
        executable='aebs',      
        name='aebs_node',
        output='screen',
        condition=IfCondition(aebs_is_active_expr), 
        remappings=[
            ('/diffdrive_controller/cmd_vel', '/cmd_vel_aebs')
        ],
        parameters=[aebs_params_file] # <--- Añadido el YAML del AEBS
    )

    # --- 5. RETORNO DEL LAUNCH ---
    return LaunchDescription([
        enable_lidar_arg,
        enable_aebs_arg,
        twist_cmd_node,
        lidar_include,
        aebs_node
    ])