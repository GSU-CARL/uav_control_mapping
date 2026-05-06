#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class DroneBaseNode(Node):
    """Base class for MAVROS drone initialization and state management."""

    def __init__(self, node_name):
        super().__init__(node_name)

        # Current state
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.pose_received = False

        self.setpoint_counter = 0
        self.offboard_requested = False
        self.arm_requested = False
        self.last_req_time = self.get_clock().now()

        # State subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10)

        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_pos_cb, qos_profile_sensor_data)

        # Local position publisher
        self.local_pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        self.wait_for_services()
        self.get_logger().info('MAVROS services are ready!')

    def local_pos_cb(self, msg):
        """Local position callback."""
        self.current_pose = msg
        self.pose_received = True

    def state_cb(self, msg):
        """State callback."""
        self.current_state = msg

    def wait_for_services(self):
        """Wait for all essential MAVROS services to be available."""
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')

        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')

    def set_mode(self, mode: str):
        """Set flight mode asynchronously."""
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def arm(self):
        """Arm the vehicle asynchronously."""
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)

    def land(self):
        """Command the vehicle to land by switching to AUTO.LAND mode."""
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done():
            self.get_logger().info('Land command sent')
        else:
            self.get_logger().warn('Land command timed out')

    def manage_offboard_and_arming(self, current_time):
        """Handles the sequence to enter OFFBOARD mode and arm the drone. Returns True when in offboard and armed."""
        time_since_last_req = (current_time - self.last_req_time).nanoseconds / 1e9

        # Try to enable OFFBOARD mode
        if self.current_state.mode != 'OFFBOARD' and time_since_last_req > 5.0:
            self.set_mode('OFFBOARD')
            if not self.offboard_requested:
                self.get_logger().info('OFFBOARD mode requested')
                self.offboard_requested = True
            self.last_req_time = current_time

        # Try to arm the vehicle
        elif not self.current_state.armed and time_since_last_req > 5.0:
            self.arm()
            if not self.arm_requested:
                self.get_logger().info('Arming requested')
                self.arm_requested = True
            self.last_req_time = current_time
            
        return self.current_state.mode == 'OFFBOARD' and self.current_state.armed
