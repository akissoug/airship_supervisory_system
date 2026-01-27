#!/usr/bin/env python3
"""
Launch file for ArduPilot Supervisory System
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ardupilot_supervisory')
    
    # Declare arguments
    sitl_mode_arg = DeclareLaunchArgument(
        'sitl_mode',
        default_value='true',
        description='Run in SITL simulation mode'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'supervisory_params.yaml'),
        description='Path to configuration file'
    )
    
    # Get launch configurations
    sitl_mode = LaunchConfiguration('sitl_mode')
    config_file = LaunchConfiguration('config_file')
    
    # Define nodes
    power_monitor_node = Node(
        package='ardupilot_supervisory',
        executable='power_monitor',
        name='power_monitor',
        parameters=[config_file, {'sitl_mode': sitl_mode}],
        output='screen'
    )
    
    fault_detector_node = Node(
        package='ardupilot_supervisory',
        executable='fault_detector',
        name='fault_detector',
        parameters=[config_file, {'sitl_mode': sitl_mode}],
        output='screen'
    )
    
    mission_supervisor_node = Node(
        package='ardupilot_supervisory',
        executable='mission_supervisor',
        name='mission_supervisor',
        parameters=[config_file, {'sitl_mode': sitl_mode}],
        output='screen'
    )
    
    energy_manager_node = Node(
        package='ardupilot_supervisory',
        executable='energy_manager',
        name='energy_manager',
        parameters=[config_file, {'sitl_mode': sitl_mode}],
        output='screen'
    )
    
    telemetry_handler_node = Node(
        package='ardupilot_supervisory',
        executable='telemetry_handler',
        name='telemetry_handler',
        parameters=[config_file, {'sitl_mode': sitl_mode}],
        output='screen'
    )
    
    emergency_manager_node = Node(
        package='ardupilot_supervisory',
        executable='emergency_manager',
        name='emergency_manager',
        parameters=[config_file, {'sitl_mode': sitl_mode}],
        output='screen'
    )
    
    return LaunchDescription([
        sitl_mode_arg,
        config_file_arg,
        power_monitor_node,
        fault_detector_node,
        mission_supervisor_node,
        energy_manager_node,
        telemetry_handler_node,
        emergency_manager_node,
    ])
