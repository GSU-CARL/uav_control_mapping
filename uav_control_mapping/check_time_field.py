# check_time_field.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class Check(Node):
    def __init__(self):
        super().__init__('check')
        self.sub = self.create_subscription(
            PointCloud2, '/drone/lidar_3d/points_timestamped', self.cb, 1)
    def cb(self, msg):
        pts = list(pc2.read_points(msg, field_names=['time', 'ring'], skip_nans=True))
        times = [p[0] for p in pts]
        rings = [p[1] for p in pts]
        print(f"time  min={min(times):.6f}  max={max(times):.6f}")
        print(f"ring  min={min(rings)}  max={max(rings)}")
        print(f"First 5 times: {[round(t,6) for t in times[:5]]}")

rclpy.init()
rclpy.spin(Check())