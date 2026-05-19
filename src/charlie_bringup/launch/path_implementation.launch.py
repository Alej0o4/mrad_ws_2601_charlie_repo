import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ===========================================================
    # PAQUETES
    # ===========================================================
    bringup_pkg    = "charlie_bringup"
    planner_pkg    = "charlie_path_planner"

    # ===========================================================
    # ARGUMENTOS GLOBALES (expuestos hacia afuera)
    # Se pasan hacia abajo a cada sub-launch que los necesite.
    # ===========================================================

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Usar reloj de simulación (Gazebo) si es true",
    )

    declare_map_name = DeclareLaunchArgument(
        "map_name",
        default_value="b_a_carrera_ser",
        description="Nombre del mapa serializado a cargar (sin extensión)",
        choices=[
            "walls_wolrd2_serialized",
            "RaceTrack",
            "RaceTrackObs",
            "demo_race_track",
            "walls_practice",
            "bloque_A_V2_ser",
            "bloque_c_ser",
            "salon_12_c_v2_ser",
            "s_16_a_ser",
            "b_a_ser",
            "b_a_2_ser",
            "b_a_19_ser",
            "b_a_carrera_ser"
        ],
    )

    declare_slam_params_file = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            get_package_share_directory(bringup_pkg),
            "config",
            "mapper_params_localization_v2.yaml",
        ),
        description="Ruta al archivo de parámetros del slam_toolbox",
    )

    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Iniciar slam_toolbox automáticamente",
    )

    declare_use_lifecycle_manager = DeclareLaunchArgument(
        "use_lifecycle_manager",
        default_value="false",
        description="Usar lifecycle manager externo para slam_toolbox",
    )

    declare_mapping = DeclareLaunchArgument(
        "mapping",
        default_value="false",
        description="Habilitar modo mappeo en lugar de localizacion",
    )

    # ===========================================================
    # REFERENCIAS A LOS ARGUMENTOS
    # ===========================================================
    use_sim_time         = LaunchConfiguration("use_sim_time")
    map_name             = LaunchConfiguration("map_name")
    slam_params_file     = LaunchConfiguration("slam_params_file")
    autostart            = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    mapping              = LaunchConfiguration("mapping")

    # ===========================================================
    # STAGE 1 — RSP + EKF + Joystick + LiDAR + Odometría
    # Se lanza primero: es la base del stack (TF, sensores, odometría).
    # ===========================================================
    rsp_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(bringup_pkg),
                "launch",
                "rsp_ekf.launch.py",
            ])
        ]),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ===========================================================
    # STAGE 2A — SLAM Toolbox (Localización) - default
    # Se retrasa 1 s para que rsp/EKF ya estén publicando TF
    # antes de que slam_toolbox intente suscribirse al árbol.
    # ===========================================================
    slam_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(bringup_pkg),
                "launch",
                "slam_localization.launch.py",
            ])
        ]),
        condition=IfCondition(NotSubstitution(mapping)),
        launch_arguments={
            "use_sim_time":          use_sim_time,
            "map_name":              map_name,
            "slam_params_file":      slam_params_file,
            "autostart":             autostart,
            "use_lifecycle_manager": use_lifecycle_manager,
        }.items(),
    )

    slam_localization_delayed = TimerAction(
        period=1.0,
        actions=[
            LogInfo(msg="[charlie_full_stack] Iniciando SLAM Toolbox (localización)..."),
            slam_localization_launch,
        ],
        condition=IfCondition(NotSubstitution(mapping)),
    )

    # ===========================================================
    # STAGE 2B — SLAM Toolbox (Mappeo) - si mapping:=True
    # Se retrasa 1 s para que rsp/EKF ya estén publicando TF
    # antes de que slam_toolbox intente suscribirse al árbol.
    # ===========================================================
    slam_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(bringup_pkg),
                "launch",
                "slam_mapping.launch.py",
            ])
        ]),
        condition=IfCondition(mapping),
        launch_arguments={
            "use_sim_time":          use_sim_time,
            "autostart":             autostart,
            "use_lifecycle_manager": use_lifecycle_manager,
        }.items(),
    )

    slam_mapping_delayed = TimerAction(
        period=1.0,
        actions=[
            LogInfo(msg="[charlie_full_stack] Iniciando SLAM Toolbox (mappeo)..."),
            slam_mapping_launch,
        ],
        condition=IfCondition(mapping),
    )

    # ===========================================================
    # STAGE 3 — Planificador ARA*
    # Se retrasa 2 s para que el mapa ya esté disponible vía SLAM
    # antes de que ara_star_node empiece a pedir rutas.
    # Solo se lanza en modo localización (no en mappeo).
    # ===========================================================
    ara_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(planner_pkg),
                "launch",
                "ara.launch.py",
            ])
        ]),
        condition=IfCondition(NotSubstitution(mapping)),
    )

    ara_delayed = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg="[charlie_full_stack] Iniciando ARA* planner..."),
            ara_launch,
        ],
        condition=IfCondition(NotSubstitution(mapping)),
    )

    # ===========================================================
    # STAGE 4 — Path Smoother (B-Splines)
    # Se lanza último: solo necesita /current_active_path,
    # que el planificador publica cuando ya tiene mapa y pose.
    # Solo se lanza en modo localización (no en mappeo).
    # ===========================================================
    path_smoother_node = Node(
        package=planner_pkg,
        executable="path_smoother_node",
        name="path_smoother_node",
        output="screen",
        condition=IfCondition(NotSubstitution(mapping)),
        parameters=[{
            "smoothing_factor":       2.0,
            "point_spacing":          0.05,
            "closed_loop_threshold":  0.5,
            "max_steering_angle_deg": 20.0,
            "wheelbase":              0.257,
        }],
    )

    path_smoother_delayed = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="[charlie_full_stack] Iniciando Path Smoother..."),
            path_smoother_node,
        ],
        condition=IfCondition(NotSubstitution(mapping)),
    )

    # ===========================================================
    # LAUNCH DESCRIPTION FINAL
    # ===========================================================
    return LaunchDescription([
        # -- Argumentos --
        declare_use_sim_time,
        declare_map_name,
        declare_slam_params_file,
        declare_autostart,
        declare_use_lifecycle_manager,
        declare_mapping,

        # -- Stage 1: base del robot (inmediato) --
        LogInfo(msg="[charlie_full_stack] Iniciando RSP + EKF + sensores..."),
        rsp_ekf_launch,

        # -- Stage 2: SLAM (localización o mappeo) (delay 1 s) --
        slam_localization_delayed,
        slam_mapping_delayed,

        # -- Stage 3: planificador (delay 6 s) --
        ara_delayed,

        # -- Stage 4: suavizador de trayectoria (delay 8 s) --
        path_smoother_delayed,
    ])