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
    aebs_pkg_name = "charlie_aebs"
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

    vicon_odom_node = Node(
        package='charlie_bringup', # Cambia esto al paquete donde guardes tu script python
        executable='vicon_to_odom_node', # Nombre configurado en setup.py
        name='vicon_to_odom',
        output='screen'
    )

    open_loop_odom_node = Node(
        package='charlie_odom',
        executable='open_loop_odom_node',
        name='open_loop_odom',
        output='screen'
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

    aebs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(aebs_pkg_name), 'launch', 'aebs.launch.py')
        )
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
        name='bicycle_odom',
        parameters=[bicycle_parms,{'use_sim_time': False}],
        output='screen'
    )

    laser_odom_params = os.path.join(get_package_share_directory(odom_pkg_name),'config','laser_icp_params.yaml')

    # Odometría con LiDAR
    lidar_odom = Node(
        package='charlie_odom',
        executable='laser_icp_odom_node',
        name='lidar_odom',
        parameters=[laser_odom_params,{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulation (Gazebo) clock if true",
            ),
            rsp,
            # vicon_odom_node,
            joy_node,
            teleop_node,
            twist_mux_node,
            lidar_node,
            # open_loop_odom_node,
            # bicycle_odom,
            # lidar_odom,

        ]
    )
