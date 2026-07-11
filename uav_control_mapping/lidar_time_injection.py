#!/usr/bin/env python3
import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class LidarTimeInjector(Node):
    def __init__(self):
        super().__init__('lidar_time_injector')

        self.declare_parameter('input_topic',       '/drone/lidar_3d/points')
        self.declare_parameter('output_topic',      '/drone/lidar_3d/points_timestamped')
        self.declare_parameter('rpm',               600.0)
        self.declare_parameter('horizontal_samples', 440)
        self.declare_parameter('vertical_samples',   16)
        self.declare_parameter('intensity_scale',   255.0)
        self.declare_parameter('flatten_time',      True)

        in_topic            = self.get_parameter('input_topic').value
        out_topic           = self.get_parameter('output_topic').value
        self.rpm            = float(self.get_parameter('rpm').value)
        self.h_samples      = int(self.get_parameter('horizontal_samples').value)
        self.v_samples      = int(self.get_parameter('vertical_samples').value)
        self.intensity_scale = float(self.get_parameter('intensity_scale').value)
        # gz gpu_lidar fires the whole sweep in one instant each frame — it does
        # NOT scan gradually like a real spinning Velodyne. Azimuth-based fake
        # per-point time can make Point-LIO "undistort" motion that never
        # happened. flatten_time=true sends rel_time=0.0 for every point so
        # this can be A/B tested against the azimuth-based version.
        self.flatten_time   = bool(self.get_parameter('flatten_time').value)
        self.frame_duration = 60.0 / self.rpm   # 0.1 s at 600 RPM

        # Point-LIO / FAST-LIO compatible layout:
        # x, y, z (float32) | intensity (float32) | ring (uint16) | pad (uint16) | time (float32)
        # 'time' is a relative offset in SECONDS from the start of this scan (0.0 .. frame_duration).
        # velody16.yaml must have  timestamp_unit: 0  (seconds).
        self.fields = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring',      offset=16, datatype=PointField.UINT16,  count=1),
        PointField(name='time',      offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        self.point_step = 24  # 4+4+4+4+2+2(pad)+4

        self.sub = self.create_subscription(PointCloud2, in_topic,  self.callback, 10)
        self.pub = self.create_publisher(  PointCloud2, out_topic, 10)

        mode = "FLATTENED (rel_time=+0.1us per point)" if self.flatten_time else "azimuth-based"
        self.get_logger().info(
            f"{mode} injector: '{in_topic}' -> '{out_topic}' | "
            f"{self.h_samples}×{self.v_samples} @ {self.rpm} RPM | "
            f"frame_duration={self.frame_duration:.4f} s"
        )

    # ------------------------------------------------------------------
    # Main callback — azimuth-based time injection
    # ------------------------------------------------------------------
    def callback(self, msg: PointCloud2):
        total = msg.width * msg.height
        if total == 0:
            return

        # Direct per-field unpack (no per-point dicts) — the dict-based
        # parse_cloud() couldn't keep up at 10 Hz × ~7k points and built up
        # ~1 s of subscription-queue latency.
        fmt_map = {
            PointField.INT8: 'b', PointField.UINT8: 'B',
            PointField.INT16: 'h', PointField.UINT16: 'H',
            PointField.INT32: 'i', PointField.UINT32: 'I',
            PointField.FLOAT32: 'f', PointField.FLOAT64: 'd',
        }
        offs = {f.name: (f.offset, fmt_map[f.datatype])
                for f in msg.fields if f.datatype in fmt_map}
        ox = offs.get('x'); oy = offs.get('y'); oz = offs.get('z')
        oi = offs.get('intensity'); oring = offs.get('ring')
        data = msg.data
        step = msg.point_step

        dt_per_col = self.frame_duration / self.h_samples  # seconds per azimuth column

        buf = bytearray()
        kept = 0
        for i in range(total):
            base = i * step
            x = struct.unpack_from(ox[1], data, base + ox[0])[0] if ox else 0.0
            y = struct.unpack_from(oy[1], data, base + oy[0])[0] if oy else 0.0
            z = struct.unpack_from(oz[1], data, base + oz[0])[0] if oz else 0.0

            # gz gpu_lidar encodes no-return rays as inf/NaN coordinates.
            # point_lio's velodyne_handler has no isfinite() guard — its only
            # gate is `x²+y²+z² > blind²`, which inf PASSES — so these points
            # land in the ikd-tree and blow up the estimator (observed: fine
            # on the ground where every ray hits, ~630 m/s runaway once
            # airborne when rays start escaping the warehouse). Drop them here.
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            # -------------------------------------------------------
            # KEY FIX: derive column from the point's actual azimuth.
            #
            # atan2 returns (-π, +π].  We shift to [0, 2π) so that
            # the scan start (angle = -π) maps to column 0 and the
            # scan end (angle ≈ +π) maps to column N-1.
            #
            # Adjust the angle_offset if your LiDAR's scan start
            # direction differs from the -X axis.
            # -------------------------------------------------------
            azimuth = math.atan2(y, x)               # radians in (-π, π]
            azimuth_norm = (azimuth + math.pi) / (2.0 * math.pi)   # 0.0 .. 1.0
            col = int(azimuth_norm * self.h_samples)
            col = max(0, min(col, self.h_samples - 1))  # clamp

            # NOTE: point_lio_ros2's velodyne_handler (preprocess.cpp) checks
            # `points[last].time > 0` to decide whether per-point times were
            # given. rel_time=0.0 for every point makes that check false,
            # which sends it down a fallback path that fabricates per-point
            # times from azimuth + the 'ring' field -- and our bridged gz
            # gpu_lidar cloud has no real 'ring' (always defaults to 0), so
            # that fallback treats the whole scan as one ring and produces
            # garbage per-point times, corrupting motion-compensation once
            # the platform is actually moving.
            #
            # The times must also be strictly INCREASING, not merely > 0:
            # laserMapping groups points by identical timestamp
            # (time_compressing) and feeds each group to one ESEKF update,
            # whose gain needs an NxN HPHT.inverse() (esekfom.hpp) where N =
            # points in the group. A constant epsilon put ALL ~7k points in
            # ONE group -> ~1700x1700 inverse per scan -> 0.72 s/scan, an
            # ever-growing input backlog, and RViz minutes behind Gazebo.
            # A 0.1 us increment per kept point keeps every group at size 1
            # (Point-LIO's native point-by-point path, scalar inverse) while
            # staying physically negligible: full scan spans < 1 ms vs the
            # sensor's instantaneous capture.
            rel_time = (kept + 1) * 1e-7 if self.flatten_time else col * dt_per_col  # seconds

            intensity = (struct.unpack_from(oi[1], data, base + oi[0])[0]
                         if oi else 1.0) * self.intensity_scale
            intensity = max(0.0, min(255.0, intensity))
            ring = int(struct.unpack_from(oring[1], data, base + oring[0])[0]) if oring else 0

            # Pack: x y z intensity (float32) | ring (uint16) | pad (uint16) | time (float32)
            buf += struct.pack('<ffff', x, y, z, intensity)
            buf += struct.pack('<H', ring)
            buf += struct.pack('<H', 0)       # 2-byte pad for 4-byte alignment
            buf += struct.pack('<f', rel_time)
            kept += 1

        out_msg = PointCloud2()
        out_msg.header.stamp    = msg.header.stamp    # keep Gazebo's sim timestamp
        out_msg.header.frame_id = msg.header.frame_id
        out_msg.fields          = self.fields
        out_msg.is_bigendian    = False
        out_msg.point_step      = self.point_step
        out_msg.row_step        = self.point_step * kept
        out_msg.height          = 1
        out_msg.width           = kept
        out_msg.is_dense        = True   # non-finite points are filtered above
        out_msg.data            = bytes(buf)

        self.pub.publish(out_msg)
        self.get_logger().debug(f"Published {kept}/{total} pts | cols 0..{self.h_samples-1}")


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