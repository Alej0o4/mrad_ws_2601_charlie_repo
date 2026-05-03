"""
Launch file for the Motor Startup Control Node.

This launch file starts the startup_node which controls motor startup
via joystick button press. Can be used standalone or included in other
launch files.

Usage (standalone):
  ros2 launch charlie_bringup startup_node.launch.py

Usage (with custom parameters):
  ros2 launch charlie_bringup startup_node.launch.py \
    joy_button_idx:=3 hold_button_required:=false startup_cmd_value:=0.25
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
    
    # ===== LAUNCH ARGUMENTS =====
    # These can be overridden from command line or parent launch files
    
    declare_joy_button_idx_cmd = DeclareLaunchArgument(
        'joy_button_idx',
        default_value='0',
        description='Joystick button index (0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB)'
    )
    
    declare_hold_button_required_cmd = DeclareLaunchArgument(
        'hold_button_required',
        default_value='true',
        description='If true, button must be held throughout startup. If false, single press is enough.'
    )
    
    declare_startup_cmd_value_cmd = DeclareLaunchArgument(
        'startup_cmd_value',
        default_value='0.3',
        description='Fixed linear velocity command during startup (m/s, forward only)'
    )
    
    declare_startup_timeout_cmd = DeclareLaunchArgument(
        'startup_timeout_s',
        default_value='6.0',
        description='Timeout for startup sequence in seconds'
    )
    
    declare_publish_hz_cmd = DeclareLaunchArgument(
        'publish_hz',
        default_value='20',
        description='Frequency for publishing motor commands (Hz)'
    )
    
    declare_joy_topic_cmd = DeclareLaunchArgument(
        'joy_topic',
        default_value='/joy',
        description='Joystick input topic'
    )
    
    declare_cfoc_state_topic_cmd = DeclareLaunchArgument(
        'cfoc_state_topic',
        default_value='/esc/cfoc_state',
        description='ESC CFOC state topic'
    )
    
    declare_cmd_vel_topic_cmd = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel_start',
        description='Motor command output topic'
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
            startup_params_file,  # Load defaults from YAML
            {
                'joy_button_idx': LaunchConfiguration('joy_button_idx'),
                'hold_button_required': LaunchConfiguration('hold_button_required'),
                'startup_cmd_value': LaunchConfiguration('startup_cmd_value'),
                'startup_timeout_s': LaunchConfiguration('startup_timeout_s'),
                'publish_hz': LaunchConfiguration('publish_hz'),
                'joy_topic': LaunchConfiguration('joy_topic'),
                'cfoc_state_topic': LaunchConfiguration('cfoc_state_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            }
        ],
        remappings=[
            # Optional remappings can be added here if needed
        ]
    )
    
    # ===== LAUNCH DESCRIPTION =====
    return LaunchDescription([
        log_startup_node_info,
        declare_joy_button_idx_cmd,
        declare_hold_button_required_cmd,
        declare_startup_cmd_value_cmd,
        declare_startup_timeout_cmd,
        declare_publish_hz_cmd,
        declare_joy_topic_cmd,
        declare_cfoc_state_topic_cmd,
        declare_cmd_vel_topic_cmd,
        startup_node,
    ])
