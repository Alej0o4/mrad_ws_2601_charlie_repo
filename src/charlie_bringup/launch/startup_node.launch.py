"""
Launch file for the Motor Startup Control Node.

This launch file starts the startup_node which controls motor startup
via joystick button press. Can be used standalone or included in other
launch files.

Usage (standalone):
  ros2 launch charlie_bringup startup_node.launch.py

Usage (with custom params file):
    ros2 launch charlie_bringup startup_node.launch.py \
        startup_params_file:=/absolute/path/startup_params.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    charlie_bringup_dir = get_package_share_directory('charlie_bringup')
    startup_params_file = os.path.join(charlie_bringup_dir, 'config', 'startup_params.yaml')

    declare_startup_params_file_cmd = DeclareLaunchArgument(
        'startup_params_file',
        default_value=startup_params_file,
        description='Full path to startup node parameters file'
    )
    
    # ===== LAUNCH ARGUMENTS =====
    # Parameters come from YAML by default.

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # ===== LOG INFO =====
    log_startup_node_info = LogInfo(
        msg="Starting Motor Startup Control Node"
    )
    
    # ===== NODE DEFINITION =====
    startup_node = Node(
        package='charlie_bringup',
        executable='startup_node',
        name='startup_node',
        output='screen',
        parameters=[
            LaunchConfiguration('startup_params_file'),  # YAML is source of truth
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            # Optional remappings can be added here if needed
        ]
    )
    
    # ===== LAUNCH DESCRIPTION =====
    return LaunchDescription([
        log_startup_node_info,
        declare_startup_params_file_cmd,
        declare_use_sim_time_cmd,
        startup_node,
    ])
