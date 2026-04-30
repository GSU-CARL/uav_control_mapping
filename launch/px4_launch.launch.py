#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch Gazebo simulator and PX4 SITL independently.
    """
    launch_actions = []
    
    # Dynamically resolve your package directory to find the world file
    package_dir = get_package_share_directory('uav_control_mapping')
    custom_world_path = os.path.join(package_dir, 'world', 'tugbot_depot.sdf')

    # 1. Start Gazebo Sim (Equivalent to Terminal 1)
    gz_sim_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', custom_world_path],
        output='screen',
        additional_env={
            # Setting the resource path explicitly as in your export command
            'GZ_SIM_RESOURCE_PATH': '/home/fishman/PX4-Autopilot/Tools/simulation/gz/models'
        }
    )

    # 2. Start PX4 SITL in standalone mode (Equivalent to Terminal 2)
    px4_sitl_process = ExecuteProcess(
        cmd=['make px4_sitl gz_x500_depth'],
        cwd='/home/fishman/PX4-Autopilot',
        output='screen',
        shell=True,  
        additional_env={
            # Passing the standalone flag and world name
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': 'tugbot_depot',
        }
    )

    launch_actions.append(gz_sim_process)
    launch_actions.append(px4_sitl_process)

    return LaunchDescription(launch_actions)

"""
mannually run 

export GZ_SIM_RESOURCE_PATH=/home/fishman/PX4-Autopilot/Tools/simulation/gz/models
gz sim -v 4 -r /home/fishman/ros2_ws/src/uav_control_mapping/world/tugbot_depot.sdf

cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=tugbot_depot make px4_sitl gz_x500
"""
