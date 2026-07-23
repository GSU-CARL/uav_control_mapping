#!/usr/bin/env python3
"""Keyboard teleop for the tugbot ground robot (IJKL), direct Twist publish.

No MAVROS/offboard handshake here -- tugbot is driven by gz's DiffDrive system
plugin, which just consumes geometry_msgs/Twist on /tugbot/cmd_vel (bridged to
/model/tugbot/cmd_vel, see config/lidar_bridge2.yaml).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard as kb

LINEAR_SPEED = 0.6   # m/s
ANGULAR_SPEED = 1.0  # rad/s


class GroundManualControlNode(Node):
    def __init__(self):
        super().__init__('ground_manual_control_node')

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, '/tugbot/cmd_vel', 10)

        self.listener = kb.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.daemon = True
        self.listener.start()

        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = self.linear_x
        twist.angular.z = self.angular_z
        self.cmd_vel_pub.publish(twist)

    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char is not None:
                char = key.char.lower()
                if char == 'i':
                    self.linear_x = LINEAR_SPEED
                elif char == 'k':
                    self.linear_x = -LINEAR_SPEED
                elif char == 'j':
                    self.angular_z = ANGULAR_SPEED
                elif char == 'l':
                    self.angular_z = -ANGULAR_SPEED
            elif key == kb.Key.esc:
                self.linear_x = 0.0
                self.angular_z = 0.0
        except Exception:
            pass

    def on_release(self, key):
        try:
            if hasattr(key, 'char') and key.char is not None:
                char = key.char.lower()
                if char in ('i', 'k'):
                    self.linear_x = 0.0
                elif char in ('j', 'l'):
                    self.angular_z = 0.0
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GroundManualControlNode()

    node.get_logger().info(
        "Use I/K for forward/backward, J/L to turn left/right. ESC to stop."
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
