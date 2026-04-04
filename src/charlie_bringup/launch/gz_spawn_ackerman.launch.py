import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    gazebo_pkg_name = "charlie_gazebo"
    bringup_pkg_name = "charlie_bringup"
    description_pkg_name = "charlie_description"
    aebs_pkg_name = "charlie_aebs"
    ekf_pkg_name = "charlie_ekf"

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_name = LaunchConfiguration("map_name")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    joy = LaunchConfiguration("joy")

    # --- Robot description (xacro -> URDF XML string) ---
    xacro_file = os.path.join(get_package_share_directory(description_pkg_name), "ackermann_urdf", "robot.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": use_sim_time}],
    )

    # --- Launch Gazebo (GUI) ---
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(headless)
    )

    # --- Launch Gazebo (headless) ---
    gz_launch_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": ['-s -r -v4 ', world], 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(headless)
    )

    # --- Spawn entity into Gazebo from robot_description topic ---
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "ackermann_bot",
            "-topic", "robot_description",
            "-x", "-7.0", "-y", "-3.0", "-z", "2.0", "-Y", "-1.57"
        ],
    )

    bridge_params = os.path.join(get_package_share_directory(gazebo_pkg_name),'config','topic_bridge.yaml')


    bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    output="screen",
    arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
    )

    ackermann_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster_controller"],
    )

    joy_params = os.path.join(get_package_share_directory(bringup_pkg_name),'config','joystick.yaml')

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    joy_node = Node(package='joy', 
                    executable='joy_node',
                    parameters=[joy_params,{'use_sim_time': use_sim_time}],
                    condition=IfCondition(joy)
    )

    teleop_node = Node(package='teleop_twist_joy', 
                    executable='teleop_node',
                    name="teleop_node",
                    parameters=[joy_params,{'use_sim_time': use_sim_time}],
                    remappings=[('/cmd_vel','/cmd_vel_joy')],
                    condition=IfCondition(joy)
    )

    keyboard_teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        emulate_tty=True,
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
        condition=UnlessCondition(joy)
    )

    twist_mux_params = os.path.join(get_package_share_directory(bringup_pkg_name),'config','twist_mux.yaml')
    
    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': True}],
                    remappings=[('/cmd_vel_out','/cmd_vel_raw')]
    )

    aebs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(aebs_pkg_name), 'launch', 'aebs.launch.py')
        )
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(ekf_pkg_name), 'launch', 'ekf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
   

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            "map_name",
            default_value="walls_world2",
            description="Nombre del mapa/world a cargar (sin la extensión .sdf)",
            choices=[
                "walls_world2",
                "RaceTrack",
                "RaceTrackObs",
                "demo_race_track",
                "walls_practice",
            ],
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare(gazebo_pkg_name),
                "worlds",
                PythonExpression(["'", map_name, ".sdf'"])
            ]),
            description="Full path to world SDF file",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run Gazebo in headless mode if true",
        ),
        DeclareLaunchArgument(
            "joy",
            default_value="true",
            description="If true, use joystick teleop. If false, use keyboard teleop.",
        ),
        gz_launch,
        gz_launch_headless,
        rsp,
        spawn,
        bridge,
        ackermann_spawner,
        joint_broad_spawner,
        joy_node,
        teleop_node,
        # keyboard_teleop_node,
        twist_mux_node,
        aebs_launch,
        # ekf_launch,
    ])
