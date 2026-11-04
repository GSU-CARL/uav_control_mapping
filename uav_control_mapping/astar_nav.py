#!/usr/bin/env python3
"""Simple 3D A* path planning and navigation node for the PX4 drone."""

import heapq
import math
import struct

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

from uav_control_mapping.drone_base import DroneBaseNode

# Control constants (matching manual_control style)
MAX_VEL = 1.2          # m/s max linear velocity
MAX_YAW_RATE = 0.6     # rad/s max yaw rate
KP_POS = 1.0           # proportional gain for 3D position tracking
KP_YAW = 1.2           # proportional gain for yaw alignment
TAKEOFF_ALT = 1.5      # default hover altitude (m) after takeoff
MIN_FLIGHT_Z = 0.5     # minimum allowed target altitude (m)
WAYPOINT_TOL = 0.35    # distance (m) to consider a waypoint reached
GOAL_TOL = 0.25        # distance (m) to consider final goal reached

# 3D grid & obstacle constants
GRID_RES = 0.30            # voxel size (m)
INFLATION_XY = 4           # inflate obstacles by 4 voxels (1.20 m) in X/Y to seal rack holes
INFLATION_Z = 3            # inflate obstacles by 3 voxels (0.90 m) in Z to seal shelf gaps
MIN_OBSTACLE_Z = 0.25      # ignore floor points below this height (m)
SELF_FILTER_RADIUS = 0.65  # ignore lidar points on/near the drone body (m)
MAX_ASTAR_ITERS = 60000    # safety cap on A* node expansions


def quaternion_to_yaw(q) -> float:
    """Extract yaw (radians) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(val: float, limit: float) -> float:
    """Clamp val to [-limit, limit]."""
    return max(-limit, min(limit, val))


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# Precompute 26-connected 3D neighbor offsets and step costs
NEIGHBORS_3D = [
    (dx, dy, dz, math.sqrt(dx * dx + dy * dy + dz * dz))
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0)
]

# Precompute inflation offsets around an occupied voxel
INFLATION_OFFSETS = [
    (dx, dy, dz)
    for dx in range(-INFLATION_XY, INFLATION_XY + 1)
    for dy in range(-INFLATION_XY, INFLATION_XY + 1)
    for dz in range(-INFLATION_Z, INFLATION_Z + 1)
    if dx * dx + dy * dy <= INFLATION_XY * INFLATION_XY + 1
]


class AStarNavNode(DroneBaseNode):
    """Flies the drone in 3D to RViz /clicked_point goals using A* obstacle avoidance."""

    def __init__(self):
        super().__init__('astar_nav_node')

        # Obstacle storage: raw occupied voxels and inflated obstacle voxels
        self.raw_obstacles = set()
        self.obstacles = set()

        # Navigation state
        self.takeoff_initialized = False
        self.is_taken_off = False
        self.hover_pos = [0.0, 0.0, TAKEOFF_ALT]
        self.goal_world = None
        self.waypoints = []
        self.wp_idx = 0

        # Publishers (same velocity topic as manual_control.py)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10
        )
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # Subscribers (accepts both live Point-LIO scans and OctoMap voxel centers)
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/cloud_registered', self.cloud_cb, 10
        )
        self.octomap_sub = self.create_subscription(
            PointCloud2, '/octomap_point_cloud_centers', self.octomap_cb, 10
        )
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.octomap_latched_sub = self.create_subscription(
            PointCloud2, '/octomap_point_cloud_centers', self.octomap_cb, latched_qos
        )
        self.clicked_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_cb, 10
        )

        # 20 Hz control loop required for PX4 OFFBOARD mode
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)
        self.get_logger().info(
            '3D A* Navigation Node started. Use RViz "Publish Point" (/clicked_point) to send 3D goals.'
        )

    # ------------------------------------------------------------------
    # Grid helpers
    # ------------------------------------------------------------------
    def world_to_grid(self, x: float, y: float, z: float):
        return (
            int(math.floor(x / GRID_RES)),
            int(math.floor(y / GRID_RES)),
            int(math.floor(z / GRID_RES)),
        )

    def grid_to_world(self, ix: int, iy: int, iz: int):
        return (
            (ix + 0.5) * GRID_RES,
            (iy + 0.5) * GRID_RES,
            (iz + 0.5) * GRID_RES,
        )

    # ------------------------------------------------------------------
    # PointCloud2 callbacks -> update 3D obstacle grid
    # ------------------------------------------------------------------
    def cloud_cb(self, msg: PointCloud2):
        self._ingest_pointcloud(msg, self_filter=True)

    def octomap_cb(self, msg: PointCloud2):
        self._ingest_pointcloud(msg, self_filter=True)

    def _ingest_pointcloud(self, msg: PointCloud2, self_filter: bool = True):
        total = msg.width * msg.height
        if total == 0:
            return

        # Fast numpy extraction of x, y, z
        offs = {f.name: f.offset for f in msg.fields if f.datatype == PointField.FLOAT32}
        if 'x' not in offs or 'y' not in offs or 'z' not in offs:
            return

        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(total, msg.point_step)
        x = raw[:, offs['x']:offs['x'] + 4].view(np.float32).reshape(-1)
        y = raw[:, offs['y']:offs['y'] + 4].view(np.float32).reshape(-1)
        z = raw[:, offs['z']:offs['z'] + 4].view(np.float32).reshape(-1)

        # Filter out non-finite points and ground points
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > MIN_OBSTACLE_Z)
        if not np.any(valid):
            return
        x, y, z = x[valid], y[valid], z[valid]

        # Self-filter points on/near the drone itself if pose is available
        if self_filter and self.pose_received:
            cp = self.current_pose.pose.position
            dist_sq = (x - cp.x) ** 2 + (y - cp.y) ** 2 + (z - cp.z) ** 2
            keep = dist_sq > (SELF_FILTER_RADIUS ** 2)
            if not np.any(keep):
                return
            x, y, z = x[keep], y[keep], z[keep]

        ix = np.floor(x / GRID_RES).astype(np.int32)
        iy = np.floor(y / GRID_RES).astype(np.int32)
        iz = np.floor(z / GRID_RES).astype(np.int32)

        vox_tuples = set(zip(ix.tolist(), iy.tolist(), iz.tolist()))
        new_voxels = vox_tuples - self.raw_obstacles
        if not new_voxels:
            return

        self.raw_obstacles.update(new_voxels)
        obs = self.obstacles
        for vx, vy, vz in new_voxels:
            for dx, dy, dz in INFLATION_OFFSETS:
                obs.add((vx + dx, vy + dy, vz + dz))

    # ------------------------------------------------------------------
    # RViz /clicked_point callback -> plan 3D A* path
    # ------------------------------------------------------------------
    def clicked_point_cb(self, msg: PointStamped):
        if not self.pose_received:
            self.get_logger().warn('Pose not received yet; ignoring clicked point.')
            return

        gx = float(msg.point.x)
        gy = float(msg.point.y)
        gz = max(float(msg.point.z), MIN_FLIGHT_Z)

        self.goal_world = (gx, gy, gz)
        self.get_logger().info(
            f'Clicked point received: ({gx:.2f}, {gy:.2f}, {gz:.2f}). Planning 3D A* path...'
        )
        self.plan_from_current_pose(gx, gy, gz, msg.header.frame_id or 'camera_init')

    def find_nearest_free_voxel(self, target_vox, ref_vox, max_radius=8):
        """Find nearest unoccupied voxel to target_vox, biased toward ref_vox (for points on walls)."""
        min_iz = int(math.floor(MIN_FLIGHT_Z / GRID_RES))
        if target_vox not in self.obstacles and target_vox[2] >= min_iz:
            return target_vox

        tx, ty, tz = target_vox
        rx, ry, rz = ref_vox
        best_vox = None
        best_score = float('inf')

        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    for dz in range(-r, r + 1):
                        if max(abs(dx), abs(dy), abs(dz)) != r:
                            continue
                        cand = (tx + dx, ty + dy, tz + dz)
                        if cand[2] < min_iz or cand in self.obstacles:
                            continue
                        dist_target = math.sqrt(dx * dx + dy * dy + dz * dz)
                        dist_ref = math.sqrt(
                            (cand[0] - rx) ** 2 + (cand[1] - ry) ** 2 + (cand[2] - rz) ** 2
                        )
                        score = dist_target + 0.15 * dist_ref
                        if score < best_score:
                            best_score = score
                            best_vox = cand
            if best_vox is not None:
                return best_vox

        return best_vox

    def astar_3d(self, start_vox, goal_vox):
        """Standard 3D A* search on the voxel grid."""
        if start_vox == goal_vox:
            return [start_vox]

        min_iz = int(math.floor(MIN_FLIGHT_Z / GRID_RES))
        gx, gy, gz = goal_vox
        obstacles = self.obstacles

        def heuristic(v):
            return math.sqrt((v[0] - gx) ** 2 + (v[1] - gy) ** 2 + (v[2] - gz) ** 2)

        open_heap = [(heuristic(start_vox), 0.0, start_vox)]
        came_from = {}
        g_score = {start_vox: 0.0}
        closed = set()

        iters = 0
        while open_heap and iters < MAX_ASTAR_ITERS:
            iters += 1
            _, current_g, current = heapq.heappop(open_heap)

            if current == goal_vox:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            if current in closed:
                continue
            closed.add(current)

            cx, cy, cz = current
            for dx, dy, dz, step_cost in NEIGHBORS_3D:
                nb = (cx + dx, cy + dy, cz + dz)
                if nb[2] < min_iz or nb in closed or nb in obstacles:
                    continue
                # Prevent diagonal corner-cutting through gaps between obstacles
                if (
                    (dx != 0 and (cx + dx, cy, cz) in obstacles)
                    or (dy != 0 and (cx, cy + dy, cz) in obstacles)
                    or (dz != 0 and (cx, cy, cz + dz) in obstacles)
                ):
                    continue

                tentative_g = current_g + step_cost
                if tentative_g < g_score.get(nb, float('inf')):
                    came_from[nb] = current
                    g_score[nb] = tentative_g
                    f_score = tentative_g + heuristic(nb)
                    heapq.heappush(open_heap, (f_score, tentative_g, nb))

        return None

    def plan_from_current_pose(self, gx: float, gy: float, gz: float, frame_id: str = 'camera_init'):
        cp = self.current_pose.pose.position
        raw_start = self.world_to_grid(cp.x, cp.y, max(cp.z, MIN_FLIGHT_Z))
        raw_goal = self.world_to_grid(gx, gy, gz)

        start_vox = self.find_nearest_free_voxel(raw_start, raw_goal, max_radius=8)
        if start_vox is None:
            start_vox = raw_start

        # Clicked points in RViz are usually on a wall/obstacle -> snap to nearest safe standoff voxel
        goal_vox = self.find_nearest_free_voxel(raw_goal, start_vox, max_radius=12)
        if goal_vox is None:
            self.get_logger().warn('No free voxel found near clicked point! Aborting plan.')
            return

        if goal_vox != raw_goal:
            sx, sy, sz = self.grid_to_world(*goal_vox)
            self.get_logger().info(
                f'Clicked point is on/near an obstacle; snapped goal to safe point ({sx:.2f}, {sy:.2f}, {sz:.2f}).'
            )

        vox_path = self.astar_3d(start_vox, goal_vox)
        if not vox_path:
            self.get_logger().warn('A* could not find a valid 3D path to the goal!')
            return

        self.waypoints = [self.grid_to_world(*v) for v in vox_path]
        self.wp_idx = 0
        self.hover_pos = list(self.waypoints[-1])

        self.get_logger().info(f'Path planned with {len(self.waypoints)} waypoints.')
        self.publish_path_vis(self.waypoints, frame_id)

    def publish_path_vis(self, waypoints, frame_id: str):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id
        for wx, wy, wz in waypoints:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.position.z = float(wz)
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

    # ------------------------------------------------------------------
    # 20 Hz flight control loop (same structure as manual_control.py)
    # ------------------------------------------------------------------
    def _publish_zero(self):
        self.cmd_vel_pub.publish(Twist())

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
        if not is_ready:
            self._publish_zero()
            return

        cp = self.current_pose.pose.position
        if not self.takeoff_initialized:
            self.takeoff_initialized = True
            target_z = cp.z if cp.z > 1.0 else TAKEOFF_ALT
            self.hover_pos = [cp.x, cp.y, target_z]
            self.get_logger().info(
                f'Holding initial position ({cp.x:.2f}, {cp.y:.2f}, {target_z:.2f})m.'
            )

        if not self.is_taken_off and cp.z >= TAKEOFF_ALT - 0.3:
            self.is_taken_off = True
            self.get_logger().info('Drone airborne and ready for 3D RViz clicked points!')

        # Determine active 3D target point
        if self.waypoints and self.wp_idx < len(self.waypoints):
            # Replan if a newly discovered obstacle now blocks our remaining path
            for future_wp in self.waypoints[self.wp_idx:]:
                if self.world_to_grid(*future_wp) in self.obstacles and self.goal_world is not None:
                    self.get_logger().info('Obstacle detected on active path! Replanning...')
                    self.plan_from_current_pose(*self.goal_world)
                    break

        if self.waypoints and self.wp_idx < len(self.waypoints):
            tx, ty, tz = self.waypoints[self.wp_idx]
            dist = math.sqrt((tx - cp.x) ** 2 + (ty - cp.y) ** 2 + (tz - cp.z) ** 2)
            tol = GOAL_TOL if self.wp_idx == len(self.waypoints) - 1 else WAYPOINT_TOL

            if dist < tol:
                if self.wp_idx == len(self.waypoints) - 1:
                    self.get_logger().info(
                        f'Reached 3D goal ({tx:.2f}, {ty:.2f}, {tz:.2f})! Hovering.'
                    )
                    self.hover_pos = [tx, ty, tz]
                    self.waypoints = []
                else:
                    self.wp_idx += 1
                    tx, ty, tz = self.waypoints[self.wp_idx]
        else:
            tx, ty, tz = self.hover_pos

        # Compute world-frame velocity command toward (tx, ty, tz)
        dx = tx - cp.x
        dy = ty - cp.y
        dz = tz - cp.z

        twist = Twist()
        twist.linear.x = float(clamp(KP_POS * dx, MAX_VEL))
        twist.linear.y = float(clamp(KP_POS * dy, MAX_VEL))
        twist.linear.z = float(clamp(KP_POS * dz, MAX_VEL))

        # Face direction of horizontal travel when moving laterally
        if math.hypot(dx, dy) > 0.35:
            current_yaw = quaternion_to_yaw(self.current_pose.pose.orientation)
            desired_yaw = math.atan2(dy, dx)
            yaw_err = wrap_angle(desired_yaw - current_yaw)
            twist.angular.z = float(clamp(KP_YAW * yaw_err, MAX_YAW_RATE))

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AStarNavNode()

    node.get_logger().info('Waiting for FCU connection...')
    while rclpy.ok() and not node.current_state.connected:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info('FCU connected! Taking off and waiting for /clicked_point from RViz.')

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
