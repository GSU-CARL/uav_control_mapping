#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def generate_launch_description():
    launch_actions = []

    # print info message
    launch_actions.append(
        LogInfo(msg='\033[96m[INFO] Starting MicroXRCE Agent...\033[0m')
    )
    # Start MicroXRCE Agent
    uxrce_agent = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        shell=False,
    )
    launch_actions.append(uxrce_agent)

    # print info message
    launch_actions.append(
        LogInfo(msg='\033[96m[INFO] Starting MAVLink...\033[0m')
    )
    # Start MAVLink
    mavlink = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'launch', 'mavros', 'px4.launch', 'fcu_url:=udp://:14540@localhost:14557'],
        output='screen',
        shell=False,
    )
    launch_actions.append(mavlink)

    # Start PX4 SITL
    px4_sitl = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'launch', 'uav_control_mapping', 'px4_launch.launch.py'],
        output='screen'
    )
    launch_actions.append(px4_sitl)


    return LaunchDescription(launch_actions)



