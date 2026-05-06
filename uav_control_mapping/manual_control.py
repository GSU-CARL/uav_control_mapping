#!/usr/bin/env python3

import rclpy
import math
from pynput import keyboard as kb

from geometry_msgs.msg import Twist

from uav_control_mapping.drone_base import DroneBaseNode

MAX_VEL = 2.0
MAX_YAW_RATE = 0.5
SPEED = 1.0
YAW_SPEED = 0.5

def quaternion_to_yaw(q) -> float:
    """Extract yaw (radians) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def body_to_world(vx_body: float, vy_body: float, yaw: float):
    """Rotate body-frame (forward/left) velocity into ENU world-frame."""
    vx_world = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
    vy_world = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)
    return vx_world, vy_world

def clamp(val, limit):
    return max(-limit, min(limit, val))

class ManualControlNode(DroneBaseNode):
    def __init__(self):
        super().__init__('manual_control_node')

        self.vx_body = 0.0
        self.vy_body = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10
        )

        self.get_logger().info('Services up. Starting keyboard listener and main timer.')

        # Input listener
        self.listener = kb.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.daemon = True
        self.listener.start()
        
        # Start 20 Hz timer
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)

    def land(self):
        self.get_logger().info('Emergency/Landing command received. Switching to AUTO.LAND...')
        self._publish_zero()
        self.set_mode('AUTO.LAND')

    def timer_callback(self):
        if not self.current_state.connected:
            return

        if not self.pose_received:
            self._publish_zero()
            return

        if self.setpoint_counter < 100:
            self._publish_zero()
            self.setpoint_counter += 1
            return

        now = self.get_clock().now()
        is_ready = self.manage_offboard_and_arming(now)

        self._publish_cmd()

    def _publish_zero(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def _publish_cmd(self):
        yaw = quaternion_to_yaw(self.current_pose.pose.orientation)
        vx_w, vy_w = body_to_world(self.vx_body, self.vy_body, yaw)

        twist = Twist()
        twist.linear.x = float(clamp(vx_w, MAX_VEL))
        twist.linear.y = float(clamp(vy_w, MAX_VEL))
        twist.linear.z = float(clamp(self.vz, MAX_VEL))
        twist.angular.z = float(clamp(self.yaw_rate, MAX_YAW_RATE))
        
        self.cmd_vel_pub.publish(twist)

    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char is not None:
                char = key.char.lower()
                if char == 'w':
                    self.vx_body = SPEED
                elif char == 's':
                    self.vx_body = -SPEED
                elif char == 'a':
                    self.vy_body = SPEED
                elif char == 'd':
                    self.vy_body = -SPEED
                elif char == 'q':
                    self.yaw_rate = YAW_SPEED
                elif char == 'e':
                    self.yaw_rate = -YAW_SPEED
                elif char == 'l':
                    self.land()
            else:
                if key == kb.Key.space:
                    self.vz = SPEED
                elif key in (kb.Key.ctrl, kb.Key.ctrl_l, kb.Key.ctrl_r):
                    self.vz = -SPEED
                elif key == kb.Key.esc:
                    self.vx_body = 0.0
                    self.vy_body = 0.0
                    self.vz = 0.0
                    self.yaw_rate = 0.0
                    self.land()
        except Exception:
            pass

    def on_release(self, key):
        try:
            if hasattr(key, 'char') and key.char is not None:
                char = key.char.lower()
                if char in ('w', 's'):
                    self.vx_body = 0.0
                elif char in ('a', 'd'):
                    self.vy_body = 0.0
                elif char in ('q', 'e'):
                    self.yaw_rate = 0.0
            else:
                if key in (kb.Key.space, kb.Key.ctrl, kb.Key.ctrl_l, kb.Key.ctrl_r):
                    self.vz = 0.0
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    
    node = ManualControlNode()
    
    node.get_logger().info("Waiting for FCU connection...")
    
    # Wait for FCU connection
    while rclpy.ok() and not node.current_state.connected:
        rclpy.spin_once(node, timeout_sec=0.1)
        
    node.get_logger().info("FCU connected! Try WASD keys to fly.")
    node.get_logger().info("Use SPACE/CTRL for up/down, Q/E for yaw, and L to land. ESC to stop and land immediately.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) -> landing...')
        node.land()
    except Exception as e:
        node.get_logger().error(f'Exception: {e}')
        node.land()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()