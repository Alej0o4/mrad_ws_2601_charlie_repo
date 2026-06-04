import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    bringup_pkg_name = "charlie_bringup"
    description_pkg_name = "charlie_description"
    odom_pkg_name = "charlie_odom"

    robot_name = "ackermann_urdf"
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ... Robot description (xacro -> URDF XML string) ...
    xacro_file = os.path.join(
        get_package_share_directory(description_pkg_name),
        robot_name,
        "robot.urdf.xacro",
    )
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
    )

    joy_params = os.path.join(get_package_share_directory(bringup_pkg_name),'config','joystick.yaml')

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    joy_node = Node(package='joy', 
                    executable='joy_node',
                    parameters=[joy_params],
    )

    teleop_node = Node(package='teleop_twist_joy', 
                    executable='teleop_node',
                    name="teleop_node",
                    parameters=[joy_params],
                    remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    twist_mux_params = os.path.join(get_package_share_directory(bringup_pkg_name),'config','twist_mux.yaml')
    
    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': False}],
                    remappings=[('/cmd_vel_out','/cmd_vel_raw')]
    )

    # Se suscribe a /scan_raw y publica en /scan
    lidar_node = Node(
        package='charlie_bringup',
        executable='scan_inverter_node', # El nombre que pusiste en setup.py
        name='scan_inverter',
        output='screen'
    )

    bicycle_parms = os.path.join(get_package_share_directory(odom_pkg_name),'config','bicycle_odom_params.yaml')

    # Odometría con modelo bicycle y sin corrección de deriva
    bicycle_odom = Node(
        package='charlie_odom',
        executable='bicycle_odom_node',
        name='bicycle_odom_node',
        parameters=[bicycle_parms,{'use_sim_time': False}],
        output='screen'
    )

    imu_parms = os.path.join(get_package_share_directory(odom_pkg_name),'config','imu_node_params.yaml')

    imu_processor_node = Node(
        package='charlie_odom', # Ajusta el paquete si lo guardaste en otro lado
        executable='imu_processor_node', 
        name='imu_processor_node',
        output='screen',
        parameters=[imu_parms, {'use_sim_time': False}] 
    )

    ekf_config_path = os.path.join(get_package_share_directory(odom_pkg_name), 'config', 'ekf_robot_localization.yaml')

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': False}]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulation (Gazebo) clock if true",
            ),
            rsp,
            joy_node,
            teleop_node,
            twist_mux_node,
            # lidar_node,
            bicycle_odom,
            imu_processor_node,
            start_robot_localization_cmd,
        ]
    )
