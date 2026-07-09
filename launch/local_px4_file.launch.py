#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_actions = []

    package_dir = get_package_share_directory('uav_control_mapping')
    bridge_config = os.path.join(package_dir, 'config', 'lidar_bridge2.yaml')

    px4_process = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_lidar_3d'],
        cwd='/home/fishman/PX4-Autopilot', # Replaces 'cd'
        additional_env={'PX4_GZ_WORLD': 'tugbot_warehouse'}, # Handles the environment variable
        output='screen'
    )

    lidar_timestamp_node = Node(
    package='uav_control_mapping',
    executable='lidar_time_injector',
    name='lidar_timestamp_node',
    parameters=[{
        'use_sim_time': True,        
    }],
    output='screen',
    )

    ros_gz_bridge_process = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
        output='screen'
    )
    
    launch_actions.append(px4_process)
    launch_actions.append(ros_gz_bridge_process)
    launch_actions.append(lidar_timestamp_node)

    return LaunchDescription(launch_actions)