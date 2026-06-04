#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct

''' This is the problem. I can't save the map with Fast-lio

[fastlio_mapping-1] IMU Initial Done
[fastlio_mapping-1] [WARN] [1780591923.847427558] [laser_mapping]: No point, skip this scan!
[fastlio_mapping-1] 
[fastlio_mapping-1] [INFO] [1780591923.948416214] [laser_mapping]: Initialize the map kdtree

'''



class LidarTimeInjector(Node):
    def __init__(self):
        super().__init__('lidar_time_injector')

        self.declare_parameter('input_topic', '/lidar_3d/points')
        self.declare_parameter('output_topic', '/lidar_3d/points_timestamped')
        self.declare_parameter('rpm', 600.0)
        self.declare_parameter('horizontal_samples', 1800)
        self.declare_parameter('vertical_samples', 16)
        self.declare_parameter('intensity_scale', 255.0)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.rpm = float(self.get_parameter('rpm').value)
        self.h_samples = int(self.get_parameter('horizontal_samples').value)
        self.v_samples = int(self.get_parameter('vertical_samples').value)
        self.intensity_scale = float(self.get_parameter('intensity_scale').value)
        self.frame_duration = 60.0 / self.rpm  # 0.1 s for 600 RPM / 10 Hz

        # FAST-LIO compatible layout:
        # x,y,z (float32), intensity (float32), ring (uint16), time (float32)
        # time = relative offset from scan start (0.0 .. 0.1), NOT absolute ROS time
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
            PointField(name='time', offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        self.point_step = 24  # 4+4+4+4+2+2pad+4

        self.sub = self.create_subscription(PointCloud2, in_topic, self.callback, 10)
        self.pub = self.create_publisher(PointCloud2, out_topic, 10)

        self.get_logger().info(
            f"FAST-LIO injector: '{in_topic}' -> '{out_topic}' | "
            f"{self.h_samples}×{self.v_samples} @ {self.rpm} RPM"
        )

    def parse_cloud(self, msg: PointCloud2):
        """Unpack PointCloud2 into list of dicts by field name."""
        fmt_map = {
            PointField.INT8: 'b', PointField.UINT8: 'B',
            PointField.INT16: 'h', PointField.UINT16: 'H',
            PointField.INT32: 'i', PointField.UINT32: 'I',
            PointField.FLOAT32: 'f', PointField.FLOAT64: 'd',
        }
        specs = []
        for f in msg.fields:
            if f.datatype in fmt_map:
                specs.append((f.name, f.offset, fmt_map[f.datatype]))

        pts = []
        for i in range(msg.width * msg.height):
            base = i * msg.point_step
            row = {}
            for name, offset, fmt in specs:
                try:
                    row[name] = struct.unpack_from(fmt, msg.data, base + offset)[0]
                except struct.error:
                    pass
            pts.append(row)
        return pts

    def callback(self, msg: PointCloud2):
        rows = self.parse_cloud(msg)
        n = len(rows)
        if n == 0:
            return

        # Compute per-point relative time based on actual point count
        dt_col = self.frame_duration / self.h_samples

        buf = bytearray()
        for i, row in enumerate(rows):
            # Map point index to column proportionally, clamped to valid range
            col = min(i * self.h_samples // n, self.h_samples - 1)
            rel_time = col * dt_col  # 0.0 at scan start, max ≈ frame_duration

            x = float(row.get('x', 0.0))
            y = float(row.get('y', 0.0))
            z = float(row.get('z', 0.0))
            intensity = float(row.get('intensity', 1.0)) * self.intensity_scale
            intensity = min(255.0, max(0.0, intensity))
            ring = int(row.get('ring', 0))

            # Pack: x,y,z,intensity (float32), ring (uint16), 2-byte pad, time (float32)
            buf += struct.pack('<ffff', x, y, z, intensity)
            buf += struct.pack('<H', ring)
            buf += struct.pack('<H', 0)      # padding to keep 4-byte alignment
            buf += struct.pack('<f', rel_time)

        out_msg = PointCloud2()
        out_msg.header.stamp = msg.header.stamp          # use Gazebo's stamp, not ROS wall/sim clock
        out_msg.header.frame_id = msg.header.frame_id
        out_msg.fields = self.fields
        out_msg.is_bigendian = False
        out_msg.point_step = self.point_step
        out_msg.row_step = self.point_step * len(rows)
        out_msg.height = 1
        out_msg.width = len(rows)
        out_msg.is_dense = msg.is_dense
        out_msg.data = bytes(buf)

        self.pub.publish(out_msg)
        self.get_logger().debug(f"Published {len(rows)} points, t={rel_time:.4f}s")


def main(args=None):
    rclpy.init(args=args)
    node = LidarTimeInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



    
