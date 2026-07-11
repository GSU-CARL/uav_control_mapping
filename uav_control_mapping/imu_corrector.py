#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuCorrector(Node):
    def __init__(self):
        super().__init__('imu_corrector', automatically_declare_parameters_from_overrides=True)
        # self.declare_parameter('use_sim_time', True)  # ← ADD THIS
        self.pub = self.create_publisher(Imu, '/imu_corrected', 10)
        self.sub = self.create_subscription(Imu, '/imu', self.cb, 10)

    def cb(self, msg):
        msg.linear_acceleration.x *= -1.0
        msg.linear_acceleration.y *= -1.0
        msg.linear_acceleration.z *= -1.0
        msg.angular_velocity.x    *= -1.0
        msg.angular_velocity.y    *= -1.0
        msg.angular_velocity.z    *= -1.0
        # flip orientation too
        msg.orientation.x *= -1.0
        msg.orientation.y *= -1.0
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ImuCorrector())

if __name__ == '__main__':
    main()