#!/usr/bin/env python3
"""
Combined launch: PX4 SITL + ROS-GZ bridge + lidar_time_injector
(both via local_px4_file.launch.py) + static TF (base_link -> lidar_3d,
matching the real SDF offset) + Point-LIO (mapping_velody16.launch.py).

The lidar_time_injector is started inside local_px4_file.launch.py — do NOT
add a second instance here, its defaults (in: /drone/lidar_3d/points, out:
/drone/lidar_3d/points_timestamped, rpm=600, h=440, v=16) already match
lidar_bridge2.yaml and velody16.yaml. Running two instances double-publishes
every scan and was the cause of a hard-to-diagnose LIO drift issue.

All topic names below match lidar_bridge2.yaml and velody16.yaml exactly:
  bridge   -> /drone/lidar_3d/points, /drone/imu, /drone/imu2
  injector -> in: /drone/lidar_3d/points, out: /drone/lidar_3d/points_timestamped
  point_lio -> lid_topic: /drone/lidar_3d/points_timestamped, imu_topic: /drone/imu2
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    uav_control_mapping_dir = get_package_share_directory('uav_control_mapping')
    point_lio_dir = get_package_share_directory('point_lio')

    # 1. PX4 SITL + ROS-GZ bridge (reuse local_px4_file.launch.py as-is)
    px4_and_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uav_control_mapping_dir, 'launch', 'local_px4_file.launch.py')
        )
    )

    # 2. Static TF base_link -> lidar_3d sensor, matching x500_lidar_3d/model.sdf's
    #    LidarJoint (0, 0, 0.26) plus lidar_3d's own 0,0,0.0435 sensor pose ->
    #    (0, 0, 0.3035) with zero rotation (all SDF poses here are unrotated).
    #    NOTE: lidar was moved from x=-0.1 to x=0 to center its mass over
    #    base_link and stop the CoG-offset hover drift; keep this in sync.
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.3035',
                   '0', '0', '0',
                   'base_link',
                   'x500_lidar_3d_0/link/lidar_3d'],
    )

    # aft_mapped (Point-LIO's body frame) visualized coincident with base_link
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0',
                   '0', '0', '0',
                   'aft_mapped',
                   'base_link'],
    )

    # 3. Point-LIO (loads point_lio_ros2/config/velody16.yaml from its install share)
    point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_lio_dir, 'launch', 'mapping_velody16.launch.py')
        )
    )

    return LaunchDescription([
        px4_and_bridge,
        lidar_static_tf,
        lidar_tf,
        point_lio,
    ])
