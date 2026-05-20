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
        description='Activar el sistema AEBS FSM (Requiere enable_lidar:=true)'
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
    external_esc_params_file = os.path.join(
        get_package_share_directory('yb_eb_pkg'), 'config', 'external_esc_params.yaml'
    )
    
    # [CAMBIO] Apuntamos al NUEVO archivo de parámetros que creamos
    fsm_aebs_params_file = os.path.join(
        get_package_share_directory('charlie_aebs'), 'config', 'fsm_parameters_aebs.yaml'
    )

    # Ruta al archivo launch original del fabricante del LiDAR
    sllidar_launch_file = os.path.join(
        get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py'
    )

    # --- 4. DEFINICIÓN DE NODOS Y LAUNCHES INCORPORADOS ---

    # A. Dirección + telemetría del hardware con ESC externa
    external_esc_node = Node(
        package='yb_eb_pkg',
        executable='yb_eb_external_esc_node',
        name='yb_eb_external_esc_node',
        output='screen',
        parameters=[
            external_esc_params_file,
            {'cmd_vel_topic': twist_cmd_topic_expr} 
        ]
    )

    # B. LiDAR (Llamando al launch del proveedor)
    lidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch_file),
        condition=IfCondition(enable_lidar) 
    )

    # C. FSM AEBS (Nuestro nuevo nodo de seguridad)
    fsm_aebs_node = Node(
        package='charlie_aebs', 
        executable='fsm_aebs_node',      # [CAMBIO] Nuevo ejecutable declarado en setup.py
        name='fsm_aebs_node',       # [CAMBIO] Nuevo nombre del nodo
        output='screen',
        condition=IfCondition(aebs_is_active_expr), 
        # [CAMBIO] Eliminé el "remappings" porque en nuestro código de fsm_aebs.py 
        # ya lo programamos para publicar directamente en '/cmd_vel_aebs'.
        parameters=[fsm_aebs_params_file] 
    )

    # D. ESC Custom FOC (Agregado desde comando de consola)
    esc_custom_foc_node = Node(
        package='yb_eb_pkg',
        executable='esc_node_custom_foc',
        name='esc_node_custom_foc',
        output='screen',
        parameters=[{
            'vehicle': 'hosim',
            'max_speed_rpm': 7000,
            'iq_limit_a': 20.0,
            'ol_target_rpm': 1600,
            'cl_mode': 'full',
            'observer_mode': 0,
            'max_linear_mps': 2.0,
            'cmd_hz': 20
        }]
    )

    # --- 5. RETORNO DEL LAUNCH ---
    return LaunchDescription([
        enable_lidar_arg,
        enable_aebs_arg,
        external_esc_node,
        lidar_include,
        fsm_aebs_node,
        esc_custom_foc_node
    ])