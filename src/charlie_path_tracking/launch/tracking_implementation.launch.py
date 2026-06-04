import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ===========================================================
    # PAQUETES
    # ===========================================================
    planner_pkg  = "charlie_path_planner"
    tracking_pkg = "charlie_path_tracking"

    # ===========================================================
    # RUTAS A LOS YAML DE CONFIGURACIÓN
    # ===========================================================
    waypoint_params = os.path.join(
        get_package_share_directory(planner_pkg),
        "config",
        "waypoints_obstacles.yaml",   # <-- corregido: con 's'
    )
    assert os.path.isfile(waypoint_params), (
        f"\n[ERROR] YAML no encontrado: {waypoint_params}\n"
        "Revisa el nombre del archivo y haz 'colcon build && source install/setup.bash'."
    )

    pure_pursuit_params = os.path.join(
        get_package_share_directory(tracking_pkg),
        "config",
        "pure_pursuit_ackermann.yaml",
    )
    assert os.path.isfile(pure_pursuit_params), (
        f"\n[ERROR] YAML no encontrado: {pure_pursuit_params}\n"
        "Revisa el nombre del archivo y haz 'colcon build && source install/setup.bash'."
    )

    # ===========================================================
    # ARGUMENTOS GLOBALES
    # ===========================================================

    declare_use_rviz_clicks = DeclareLaunchArgument(
        "use_rviz_clicks",
        default_value="false",
        description=(
            "true  → waypoints interactivos por RViz (2D Goal Pose). "
            "false → usa los static_waypoints definidos en el YAML."
        ),
    )

    declare_num_laps = DeclareLaunchArgument(
        "num_laps",
        default_value="2",
        description="Número de vueltas a la pista (sólo aplica con use_rviz_clicks=false).",
    )

    declare_path_topic = DeclareLaunchArgument(
        "path_topic",
        default_value="/smoothed_path",
        description="Tópico de ruta que consume el Pure Pursuit.",
    )

    declare_cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel_nav",
        description="Tópico de salida de velocidad del Pure Pursuit.",
    )

    # ===========================================================
    # REFERENCIAS A LOS ARGUMENTOS
    # ===========================================================
    use_rviz_clicks = LaunchConfiguration("use_rviz_clicks")
    num_laps        = LaunchConfiguration("num_laps")
    path_topic      = LaunchConfiguration("path_topic")
    cmd_vel_topic   = LaunchConfiguration("cmd_vel_topic")

    # ===========================================================
    # STAGE 1 — Waypoint Manager
    # Lee el YAML con la pista predefinida (o espera clics RViz)
    # y publica /current_active_path al ARA* para que genere
    # los segmentos de ruta.
    # ===========================================================
    waypoint_manager_node = Node(
        package=planner_pkg,           # registrado en charlie_path_planner
        executable="waypoint_manager", # entry_point real en setup.py
        name="waypoint_manager",
        output="screen",
        parameters=[
            waypoint_params,
            {
                # Permiten sobreescribir desde CLI sin tocar el YAML
                "use_rviz_clicks": use_rviz_clicks,
                "num_laps":        num_laps,
            },
        ],
    )

    # ===========================================================
    # STAGE 2 — Pure Pursuit Ackermann (controlador de seguimiento)
    # Se retrasa 3 s para darle tiempo al Waypoint Manager de
    # conectarse al servicio ARA* y publicar la primera ruta
    # antes de que el controlador empiece a buscar el path_topic.
    # ===========================================================
    pure_pursuit_node = Node(
        package=tracking_pkg,
        executable="pure_pursuit_ackermann",  # entry_point real en setup.py
        name="pure_pursuit_node",
        output="screen",
        parameters=[
            pure_pursuit_params,
            {
                # Permiten sobreescribir desde CLI sin tocar el YAML
                "path_topic":    path_topic,
                "cmd_vel_topic": cmd_vel_topic,
            },
        ],
    )

    pure_pursuit_delayed = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="[charlie_tracking] Iniciando Pure Pursuit Ackermann..."),
            pure_pursuit_node,
        ],
    )

    # ===========================================================
    # LAUNCH DESCRIPTION FINAL
    # ===========================================================
    return LaunchDescription([
        # -- Argumentos --
        declare_use_rviz_clicks,
        declare_num_laps,
        declare_path_topic,
        declare_cmd_vel_topic,

        # -- Stage 1: Waypoint Manager (inmediato) --
        LogInfo(msg="[charlie_tracking] Iniciando Waypoint Manager..."),
        waypoint_manager_node,

        # -- Stage 2: Pure Pursuit (delay 3 s) --
        pure_pursuit_delayed,
    ])