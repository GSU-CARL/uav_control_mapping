#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch Gazebo simulator and PX4 SITL independently.
    """
    launch_actions = []
    
    # Dynamically resolve your package directory to find the world file
    package_dir = get_package_share_directory('uav_control_mapping')
    custom_world_path = os.path.join(package_dir, 'world', 'world/tugbot_warehouse.sdf')
    custom_model_path = os.path.join(package_dir, 'model')
    bridge_config = os.path.join(package_dir, 'config', 'lidar_bridge2.yaml')
    px4_world_path = '/home/fishman/PX4-Autopilot/Tools/simulation/gz/worlds/baylands.sdf'

    # Fix for ROS 2 Jazzy shadowing system Gazebo CLI
    env_gz_config = os.environ.get('GZ_CONFIG_PATH', '')
    if '/usr/share/gz' not in env_gz_config:
        env_gz_config = f"{env_gz_config}:/usr/share/gz" if env_gz_config else "/usr/share/gz"

    # 1. Start Gazebo Sim (Equivalent to Terminal 1)
    gz_sim_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', custom_world_path],
        output='screen',
        additional_env={
            # Setting the resource path explicitly to include local models and PX4 models
            'GZ_SIM_RESOURCE_PATH': f"{custom_model_path}:/home/fishman/PX4-Autopilot/Tools/simulation/gz/models",
            'GZ_CONFIG_PATH': env_gz_config
        }
    )

    # 2. Start PX4 SITL in standalone mode (Equivalent to Terminal 2)
    px4_sitl_process = ExecuteProcess(
        cmd=['make px4_sitl gz_x500_lidar_3d'],
        cwd='/home/fishman/PX4-Autopilot',
        output='screen',
        shell=True,  
        additional_env={
            # Passing the standalone flag and world name
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': 'tugbot_warehouse',
            'GZ_SIM_RESOURCE_PATH': f"{custom_model_path}:/home/fishman/PX4-Autopilot/Tools/simulation/gz/models",
            'GZ_CONFIG_PATH': env_gz_config
        }
    )

    # 3. Start ROS-GZ Bridge
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

    # 4. Time injection

    lidar_timestamp_node = Node(
    package='uav_control_mapping',
    executable='lidar_time_injector',
    name='lidar_timestamp_node',
    parameters=[{
        'use_sim_time': True,
        'input_topic': '/lidar_3d/points',
        'output_topic': '/lidar_3d/points_timestamped',
        'rpm': 600.0,
        'horizontal_samples': 440,
        'vertical_samples': 16,
        'intensity_scale': 255.0,
    }],
    output='screen',
    )

    lidar_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0',
               '0', '0', '0',
               'aft_mapped',
               'base_link'],
    )

    lidar_static_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '-0.4377',
               '3.14159', '0', '3.14159',
               'base_link',
               'x500_lidar_3d_0/link/lidar_3d'],
    )

    imu_corrector_node = Node(
        package='uav_control_mapping',
        executable='imu_corrector',
        name='imu_corrector_node',
        parameters=[{
            'use_sim_time': True,
        }],
    )

    launch_actions.append(gz_sim_process)
    launch_actions.append(
        TimerAction(
            period=5.0,
            actions=[px4_sitl_process]
        )
    )
    launch_actions.append(lidar_static_tf)
    launch_actions.append(imu_corrector_node)
    launch_actions.append(ros_gz_bridge_process)
    launch_actions.append(lidar_timestamp_node)
    launch_actions.append(lidar_tf)
    return LaunchDescription(launch_actions)

"""
mannually run 

export GZ_SIM_RESOURCE_PATH=/home/fishman/ros2_ws/install/uav_control_mapping/share/uav_control_mapping/model:/home/fishman/PX4-Autopilot/Tools/simulation/gz/models
gz sim -v 4 -r /home/fishman/ros2_ws/src/uav_control_mapping/world/tugbot_depot.sdf

cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=tugbot_depot GZ_SIM_RESOURCE_PATH=/home/fishman/ros2_ws/install/uav_control_mapping/share/uav_control_mapping/model:/home/fishman/PX4-Autopilot/Tools/simulation/gz/models make px4_sitl gz_x500
"""
