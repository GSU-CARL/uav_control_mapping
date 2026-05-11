#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch Gazebo simulator and PX4 SITL independently.
    """
    launch_actions = []
    
    # Dynamically resolve your package directory to find the world file
    package_dir = get_package_share_directory('uav_control_mapping')
    custom_world_path = os.path.join(package_dir, 'world', 'tugbot_depot.sdf')
    models_path = os.path.join(package_dir, 'models')

    # 1. Start Gazebo Sim (Equivalent to Terminal 1)
    gz_sim_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', custom_world_path],
        output='screen',
        additional_env={
            # Setting the resource path explicitly as in your export command
            'GZ_SIM_RESOURCE_PATH': '/home/fishman/PX4-Autopilot/Tools/simulation/gz/models'
            ':' + models_path
        }
    )

    # # 2. Start PX4 SITL in standalone mode (Equivalent to Terminal 2)
    # px4_sitl_process = ExecuteProcess(
    #     cmd=['make px4_sitl gz_x500_depth'],
    #     cwd='/home/fishman/PX4-Autopilot',
    #     output='screen',
    #     shell=False,
    #     additional_env={
    #         # Passing the standalone flag and world name
    #         'PX4_GZ_STANDALONE': '1',
    #         'PX4_GZ_WORLD': 'tugbot_depot',
    #     }
    # )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'x500_lidar_3d',
                   '-file', os.path.join(models_path, 'x500_lidar_3d', 'model.sdf'),
                   '-x', '0.0', '-y', '0.0', '-z', '0.2']
    )

    px4_sitl_process = ExecuteProcess(
    cmd=['make', 'px4_sitl', 'gz_x500'],
    cwd='/home/fishman/PX4-Autopilot',
    output='screen',
    shell=False,
    additional_env={
        'PX4_GZ_STANDALONE': '1',
        'PX4_GZ_WORLD': 'tugbot_depot',
        'PX4_GZ_MODEL_NAME': 'x500_lidar_3d',
        'GZ_SIM_RESOURCE_PATH': (
            '/home/fishman/PX4-Autopilot/Tools/simulation/gz/models'
            ':' + models_path
        )
    }
)

    launch_actions.append(gz_sim_process)
    
    # Wait for Gazebo to start before spawning the drone
    launch_actions.append(
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=gz_sim_process,
                on_start=[spawn_entity]
            )
        )
    )
    
    # Wait for the model to span before running PX4 SITL to attach to it
    launch_actions.append(
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=spawn_entity,
                on_start=[px4_sitl_process]
            )
        )
    )

    return LaunchDescription(launch_actions)

"""
mannually run 

export GZ_SIM_RESOURCE_PATH=/home/fishman/PX4-Autopilot/Tools/simulation/gz/models
gz sim -v 4 -r /home/fishman/ros2_ws/src/uav_control_mapping/world/tugbot_depot.sdf

cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=tugbot_depot make px4_sitl gz_x500
"""
