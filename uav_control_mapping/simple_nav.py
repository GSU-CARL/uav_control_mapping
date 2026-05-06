#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped

import rclpy
import math

from uav_control_mapping.drone_base import DroneBaseNode

class SimpleNav(DroneBaseNode):

    def __init__(self):
        super().__init__('simple_nav_node')
        
        self.goal_pose = None               # Target goal pose
        self.is_taken_off = False           # Flag to check if drone has taken off
        self.target_wp = PoseStamped()      # Current target waypoint

        # Goal pose subscriber
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_cb, 10)

        # Setpoint publishing MUST be faster than 2Hz
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)

    def goal_pose_cb(self, msg):
        self.goal_pose = msg
        self.get_logger().info(f'Received new goal_pose: x={msg.pose.position.x}, y={msg.pose.position.y}, z=fixed(3.0)')

    def get_distance_to_target(self):
        target = self.target_wp.pose.position
        current = self.current_pose.pose.position
        
        dx = target.x - current.x
        dy = target.y - current.y
        dz = target.z - current.z
        
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def timer_callback(self):
        """Execute the main control loop callback."""
        # Wait for Flight Controller connection
        if not self.current_state.connected:
            return

        # --- Wait for a valid pose before proceeding ---
        if not self.pose_received:
            return

        # Initialize target waypoint to current position with z=3.0 for takeoff
        if not self.is_taken_off and not hasattr(self, 'takeoff_initiated'):
            
            self.takeoff_initiated = True
            self.target_wp = PoseStamped()
            self.target_wp.pose.position.x = self.current_pose.pose.position.x
            self.target_wp.pose.position.y = self.current_pose.pose.position.y
            self.target_wp.pose.position.z = 3.0
            self.target_wp.pose.orientation = self.current_pose.pose.orientation

        # Send a few setpoints before starting OFFBOARD mode
        if self.setpoint_counter < 100:
            self.local_pos_pub.publish(self.target_wp)
            self.setpoint_counter += 1
            return

        current_time = self.get_clock().now()
        is_ready = self.manage_offboard_and_arming(current_time)

        # Determine logic when OFFBOARD and Armed
        if is_ready:
            current_z = self.current_pose.pose.position.z
            # Check if takeoff to 3m is complete
            if not self.is_taken_off and current_z > 2.5:
                self.get_logger().info('Takeoff complete, hovering at 3m. Ready for goal pose.')
                self.is_taken_off = True

            # If taken off and we have a goal pose, update target waypoint
            if self.is_taken_off and self.goal_pose is not None:
                self.target_wp = self.goal_pose
                self.target_wp.pose.position.z = 3.0 # Keep altitude fixed at 3m

        # Continue publishing the active setpoint
        self.local_pos_pub.publish(self.target_wp)

def main(args=None):
    rclpy.init(args=args)

    try:
        simple_nav = SimpleNav()

        # Wait for FCU connection
        simple_nav.get_logger().info('Waiting for FCU connection...')
        while rclpy.ok() and not simple_nav.current_state.connected:
            rclpy.spin_once(simple_nav, timeout_sec=0.1)

        simple_nav.get_logger().info('FCU connected! Starting OFFBOARD control...')

        rclpy.spin(simple_nav)

    except KeyboardInterrupt:
        simple_nav.get_logger().info('OFFBOARD control interrupted by user')
        simple_nav.get_logger().info('Landing...')
        simple_nav.land()
    except Exception as e:
        print(f'An error occurred: {e}')
        if 'simple_nav' in locals():
            simple_nav.get_logger().info('Landing due to error...')
            simple_nav.land()
    finally:
        if 'simple_nav' in locals():
            simple_nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()