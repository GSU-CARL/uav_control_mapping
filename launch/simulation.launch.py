#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnShutdown


def generate_launch_description():
    launch_actions = []

    launch_actions.append(
        LogInfo(msg='\033[96m[INFO] Starting Simulation in tmux...\033[0m')
    )

    # Launch everything in a tmux session and attach in ONE gnome-terminal
    # Using 'sleep infinity' to keep the ROS launch process alive so it doesn't auto-shutdown
    tmux_start = ExecuteProcess(
        cmd=[
            'sh', '-c',
            "tmux new-session -d -s sim_session 'MicroXRCEAgent udp4 -p 8888' && "
            "tmux split-window -t sim_session 'ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557' && "
            "tmux split-window -t sim_session 'ros2 launch uav_control_mapping px4_launch.launch.py' && "
            "tmux split-window -t sim_session 'ros2 launch uav_control_mapping rtabmap_px4.launch.py' && "
            "tmux select-layout -t sim_session tiled && "
            "gnome-terminal -- tmux attach-session -t sim_session ; "
            "sleep infinity"
        ],
        output='screen'
    )
    launch_actions.append(tmux_start)

    # Kill tmux session on shutdown
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(msg='\033[91m[INFO] Killing tmux session and all simulation nodes...\033[0m'),
                ExecuteProcess(
                    cmd=[
                        'sh', '-c', 
                        'tmux kill-session -t sim_session; '
                        'pkill -f MicroXRCEAgent; '
                        'pkill -f "ros2 launch mavros"; '
                        'pkill -f px4_launch.launch.py; '
                        'pkill -f px4'
                    ],
                    output='screen'
                )
            ]
        )
    )
    launch_actions.append(shutdown_handler)

    return LaunchDescription(launch_actions)



