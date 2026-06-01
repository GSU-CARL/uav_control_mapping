#!/usr/bin/env python3
"""
lidar_time_injector.py

Subscribes to the raw Gazebo gpu_lidar PointCloud2 (x, y, z only),
injects a per-point 'time' field set to 0.0 (float32), and republishes.

FAST-LIO's Velodyne preprocessor (lidar_type: 2) requires the 'time' field
to exist for motion deskewing. Gazebo's gpu_lidar does not publish it.

Usage (add to your launch file):
    Node(
        package='uav_control_mapping',
        executable='lidar_time_injector',
        name='lidar_time_injector',
        output='screen',
    )

Topics:
    Subscribes : /lidar_3d/points        (sensor_msgs/PointCloud2)
    Publishes  : /lidar_3d/points_deskew (sensor_msgs/PointCloud2)

Then set in your FAST-LIO config:
    lid_topic: "/lidar_3d/points_deskew"
"""

import rclpy
from rclpy.node import Node

import numpy as np
import struct

from sensor_msgs import msg
from sensor_msgs.msg import PointCloud2, PointField


class LidarTimeInjector(Node):
    def __init__(self):
        super().__init__('lidar_time_injector')

        self.sub = self.create_subscription(
            PointCloud2,
            '/lidar_3d/points',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            PointCloud2,
            '/lidar_3d/points_deskew',
            10
        )
        self.get_logger().info('lidar_time_injector ready — injecting time field')

    def callback(self, msg: PointCloud2):
        # ----------------------------------------------------------------
        # Original layout from Gazebo gpu_lidar:
        #   x      float32  offset 0
        #   y      float32  offset 4
        #   z      float32  offset 8
        #   point_step = 12 bytes
        #
        # New layout with time appended:
        #   x          float32  offset 0
        #   y          float32  offset 4
        #   z          float32  offset 8
        #   intensity  float32  offset 12
        #   ring       uint16   offset 16
        #   time       float32  offset 18   ← injected
        #   point_step = 22 bytes
        # ----------------------------------------------------------------

        num_points = msg.width * msg.height
        old_step   = msg.point_step
        new_step   = old_step + 10  # 4 (intensity) + 2 (ring) + 4 (time) = 10

        # Linear time spread across the scan: 0.0 → ~0.1 s (100 ms scan period)
        scan_duration = 1.0 / 10.0  # match scan_rate in velodyne.yaml
        times = np.linspace(0.0, scan_duration, num_points, dtype=np.float32)
        intensities = np.zeros(num_points, dtype=np.float32)
        rings = np.zeros(num_points, dtype=np.uint16)

        old_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(num_points, old_step)
        intensity_bytes = intensities.view(np.uint8).reshape(num_points, 4)
        ring_bytes = rings.view(np.uint8).reshape(num_points, 2)
        time_bytes = times.view(np.uint8).reshape(num_points, 4)
        
        new_data = np.concatenate([old_data, intensity_bytes, ring_bytes, time_bytes], axis=1).flatten()

        # Build new PointCloud2
        out = PointCloud2()
        out.header     = msg.header
        out.height     = msg.height
        out.width      = msg.width
        out.is_dense   = msg.is_dense
        out.is_bigendian = msg.is_bigendian
        out.point_step = new_step
        out.row_step   = new_step * msg.width
        out.data       = new_data.tobytes()

        # Copy original fields, then add 'intensity', 'ring', 'time'
        out.fields = list(msg.fields) + [
            PointField(
                name='intensity',
                offset=old_step,
                datatype=PointField.FLOAT32,  # 7
                count=1
            ),
            PointField(
                name='ring',
                offset=old_step + 4,
                datatype=PointField.UINT16,   # 4
                count=1
            ),
            PointField(
                name='time',
                offset=old_step + 6,
                datatype=PointField.FLOAT32,  # 7
                count=1
            )
        ]

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarTimeInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()