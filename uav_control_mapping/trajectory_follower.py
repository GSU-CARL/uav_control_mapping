#!/usr/bin/env python3

import rclpy
import math
import json
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import Trigger

from uav_control_mapping.drone_base import DroneBaseNode

def euler_to_quaternion(roll, pitch, yaw) -> Quaternion:
    """Converts Euler angles to a Quaternion message."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

def get_yaw_from_quaternion(q: Quaternion) -> float:
    """Extracts yaw (Z-axis rotation) from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class TrajectoryFollower(DroneBaseNode):
    def __init__(self):
        super().__init__('trajectory_follower')

        self.state = 'TAKEOFF'
        self.waypoints = []
        self.current_wp_index = 0
        
        self.target_wp = PoseStamped()
        self.target_yaw = 0.0

        # Sub-state tracking
        self.takeoff_initiated = False
        self.path_requested = False
        self.yaw_tolerance = 0.1 # radians
        self.dist_tolerance = 0.3 # meters

        # Service client to get waypoints
        self.path_client = self.create_client(Trigger, '/generate_flight_path')
        self.clear_client = self.create_client(Trigger, '/clear_waypoints')

        # Service to trigger the mission
        self.start_srv = self.create_service(Trigger, '/start_mission', self.start_mission_cb)
        self.mission_start_requested = False

        # Main control loop running at 20Hz (required by PX4 offboard)
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)

    def start_mission_cb(self, request, response):
        if self.state in ['WAITING', 'DONE']:
            self.mission_start_requested = True
            response.success = True
            response.message = "Mission start requested."
        else:
            response.success = False
            response.message = f"Cannot start mission right now. Current state: {self.state}"
        return response

    def request_path(self):
        if not self.path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /generate_flight_path service...')
            return

        req = Trigger.Request()
        future = self.path_client.call_async(req)
        future.add_done_callback(self.path_response_callback)
        self.path_requested = True

    def path_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                pts = json.loads(response.message)
                self.waypoints = pts
                self.current_wp_index = 0
                self.get_logger().info(f"Received {len(self.waypoints)} waypoints. Proceeding to follow path.")
                self.state = 'TURN'
            else:
                self.get_logger().warn(f"Failed to get path: {response.message}")
                self.path_requested = False # Allow retrying
                self.state = 'WAITING' # Go back to waiting if we fail
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.path_requested = False
            self.state = 'WAITING'

    def distance_to_target(self, target_x, target_y):
        current = self.current_pose.pose.position
        dx = target_x - current.x
        dy = target_y - current.y
        # Ignore Z for 2D distance checks assuming fixed alt
        return math.sqrt(dx**2 + dy**2)

    def timer_callback(self):
        if not self.current_state.connected or not self.pose_received:
            return

        current_time = self.get_clock().now()

        # Bootstrap target_wp on first pass
        if not self.takeoff_initiated:
            self.takeoff_initiated = True
            self.target_wp = PoseStamped()
            self.target_wp.pose.orientation = self.current_pose.pose.orientation
            self.target_wp.pose.position.x = self.current_pose.pose.position.x
            self.target_wp.pose.position.y = self.current_pose.pose.position.y
            self.target_wp.pose.position.z = 2.5
            self.target_yaw = get_yaw_from_quaternion(self.current_pose.pose.orientation)

        # Send minimum setpoints before OFFBOARD transition
        if self.setpoint_counter < 100:
            self.local_pos_pub.publish(self.target_wp)
            self.setpoint_counter += 1
            return

        is_ready = self.manage_offboard_and_arming(current_time)
        if not is_ready:
            self.local_pos_pub.publish(self.target_wp)
            return

        # --- STATE MACHINE ---
        
        current_pos = self.current_pose.pose.position
        current_yaw = get_yaw_from_quaternion(self.current_pose.pose.orientation)

        if self.state == 'TAKEOFF':
            # Target is already properly initialized above
            if current_pos.z > 2.3:
                self.get_logger().info('Takeoff complete. Waiting for /start_mission.')
                self.state = 'WAITING'

        elif self.state == 'WAITING':
            # Hovers until user calls the /start_mission service
            if self.mission_start_requested:
                self.mission_start_requested = False
                self.get_logger().info('Mission requested. Requesting waypoints.')
                self.path_requested = False  # Reset flag before requesting
                self.state = 'GET_PATH'
                
        elif self.state == 'GET_PATH':
            if not self.path_requested:
                self.request_path()
            # Maintains hover position while waiting for service response
                
        elif self.state == 'TURN':
            if self.current_wp_index >= len(self.waypoints):
                self.get_logger().info("Mission Complete!")
                self.state = 'DONE'
                return

            target_point = self.waypoints[self.current_wp_index]
            tx, ty, _ = target_point
            
            # Calculate required yaw angle
            dx = tx - current_pos.x
            dy = ty - current_pos.y
            self.target_yaw = math.atan2(dy, dx)

            # Keep x, y locked to current position to rotate in place
            self.target_wp.pose.position.x = current_pos.x
            self.target_wp.pose.position.y = current_pos.y
            self.target_wp.pose.position.z = 2.5
            self.target_wp.pose.orientation = euler_to_quaternion(0, 0, self.target_yaw)

            # Check if yaw is close enough
            diff = abs(self.target_yaw - current_yaw)
            diff = min(diff, 2*math.pi - diff) # Normalize wrap-around

            if diff < self.yaw_tolerance:
                self.get_logger().info(f"Turn complete. Moving to WP {self.current_wp_index}.")
                self.state = 'MOVE'

        elif self.state == 'MOVE':
            target_point = self.waypoints[self.current_wp_index]
            tx, ty, tz = target_point

            # Update target position, keep orientation from previous TURN state
            self.target_wp.pose.position.x = tx
            self.target_wp.pose.position.y = ty
            self.target_wp.pose.position.z = 2.5 

            if self.distance_to_target(tx, ty) < self.dist_tolerance:
                self.get_logger().info(f"Reached WP {self.current_wp_index}.")
                self.current_wp_index += 1
                self.state = 'TURN'
                
        elif self.state == 'DONE':
            # Clear the old waypoints from the manager and RViz
            if self.clear_client.wait_for_service(timeout_sec=1.0):
                self.clear_client.call_async(Trigger.Request())
            self.get_logger().info('Waypoints cleared. Waiting for next mission.')
            self.state = 'WAITING'

        # Always publish the current setpoint loop
        self.local_pos_pub.publish(self.target_wp)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TrajectoryFollower()
        
        node.get_logger().info('Starting trajectory follower...')
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user, landing...')
        node.land()
    except Exception as e:
        print(f'An error occurred: {e}')
        if 'node' in locals():
            node.get_logger().info('Landing due to error...')
            node.land()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()