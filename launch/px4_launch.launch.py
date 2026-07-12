#!/usr/bin/env python3
"""
PX4 SITL using the locally-tracked x500_lidar_3d_local model (this package's
models/ dir) instead of PX4-Autopilot's own copy, flying in this package's
local warehouse world (world/warehouse.sdf).

x500_lidar_3d_local/model.sdf is NOT copied into PX4-Autopilot's
Tools/simulation/gz/models/, so PX4's built-in "make px4_sitl gz_<model>"
spawn path (which always reads from PX4-Autopilot's own models dir) can't see
it. Instead this launch file starts Gazebo itself, spawns our local model.sdf
directly by file path, then starts PX4 with PX4_GZ_MODEL_NAME set so PX4
attaches to that already-spawned model instead of spawning its own
(see PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim).

The world file is world/warehouse.sdf, but its internal <world name> is still
'tugbot_warehouse', so the create service, PX4_GZ_WORLD, and the
config/lidar_bridge2.yaml gz topic paths all stay correct unchanged.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PX4_AUTOPILOT_DIR = '/home/fishman/PX4-Autopilot'
WORLD_NAME = 'tugbot_warehouse'          # internal <world name> in warehouse.sdf
WORLD_FILE = 'warehouse.sdf'             # the file we actually load
MODEL_NAME = 'x500_lidar_3d_local'
MODEL_INSTANCE = f'{MODEL_NAME}_0'       # 'x500_lidar_3d_local_0'


def generate_launch_description():
    package_dir = get_package_share_directory('uav_control_mapping')
    models_dir = os.path.join(package_dir, 'models')
    model_sdf_path = os.path.join(models_dir, MODEL_NAME, 'model.sdf')

    # Load the world from THIS package's world/ dir (not PX4's worlds dir).
    world_path = os.path.join(package_dir, 'world', WORLD_FILE)
    px4_gz_models_dir = os.path.join(
        PX4_AUTOPILOT_DIR, 'Tools', 'simulation', 'gz', 'models')

    bridge_config = os.path.join(package_dir, 'config', 'lidar_bridge2.yaml')

    # So the x500 and model://lidar_3d_local merge-includes in model.sdf resolve.
    gz_resource_path = ':'.join([models_dir, px4_gz_models_dir])

    gz_plugin_path = os.path.join(
        PX4_AUTOPILOT_DIR, 'build', 'px4_sitl_default',
        'src', 'modules', 'simulation', 'gz_plugins')

    # server.config supplies the server-side systems (Physics, Sensors,
    # SceneBroadcaster, UserCommands, Imu, ...). warehouse.sdf only declares GUI
    # plugins, so without this env var IMU/baro/mag/GPS never publish and EKF2
    # reports all sensors missing.
    gz_server_config = os.path.join(
        PX4_AUTOPILOT_DIR, 'src', 'modules', 'simulation', 'gz_bridge', 'server.config')

    gz_sim_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        additional_env={
            'GZ_SIM_RESOURCE_PATH': gz_resource_path,
            'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plugin_path,
            'GZ_SIM_SERVER_CONFIG_PATH': gz_server_config,
        },
        output='screen',
    )

    create_service = f'/world/{WORLD_NAME}/create'
    spawn_req = f'sdf_filename: "{model_sdf_path}", name: "{MODEL_INSTANCE}", allow_renaming: false'

    spawn_model_process = ExecuteProcess(
        cmd=['bash', '-c',
             f'''
            until gz service -i --service "{create_service}" 2>&1 | grep -q "Service providers"; do
                sleep 1
            done
            exec gz service -s "{create_service}" \\
                --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \\
                --req '{spawn_req}'
        '''],
        output='screen',
    )

    px4_process = ExecuteProcess(
        cmd=['make', 'px4_sitl', f'gz_{MODEL_NAME}'],
        cwd=PX4_AUTOPILOT_DIR,
        additional_env={
            'PX4_GZ_WORLD': WORLD_NAME,
            'PX4_GZ_MODEL_NAME': MODEL_INSTANCE,
        },
        output='screen',
    )

    lidar_timestamp_node = Node(
        package='uav_control_mapping',
        executable='lidar_time_injector',
        name='lidar_timestamp_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    ros_gz_bridge_process = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    # Sequencing is deliberate: PX4/bridge/injector only start after the spawn
    # command exits. Starting the bridge before the model exists leaves every
    # sensor topic unbridged.
    start_px4_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_model_process,
            on_exit=[px4_process, ros_gz_bridge_process, lidar_timestamp_node],
        )
    )

    return LaunchDescription([
        gz_sim_process,
        spawn_model_process,
        start_px4_after_spawn,
    ])
