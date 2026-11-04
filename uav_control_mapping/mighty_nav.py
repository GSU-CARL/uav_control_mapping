#!/usr/bin/env python3
"""MIGHTY (Hermite Spline-based Efficient Trajectory Planning) node for the PX4 UAV.

Replicates the MIGHTY planning pipeline from /home/fishman/Documents/internship/mighty:
1. MapUtil 3D Voxel Map + Static Boundary Heat Halo (map_util.hpp)
2. HGP (Hierarchical Global Planner):
   - Weighted 3D A* with static boundary heat cost ('astar_heat' in graph_search.cpp)
   - Capsule Line-of-Sight shortcutting (lineOfSightCapsule / shortCutByLoS)
   - Collinear removal + cost/heat-aware corner shortcutting (cleanUpPath / removeCornerPts)
3. Safe Corridor Halfspaces (A_s x <= b_s per segment) around the global path
4. Quintic Hermite Spline Local Trajectory Optimization (lbfgs_solver.cpp):
   - Turn-aware initial segment time allocation (allocate_times_with_vector_velocities)
   - Closed-form block-tridiagonal minimum-jerk initial guess (solve_minjerk_vel_acc_closedform)
   - GCOPTER C^2 time bijection tau <-> T and averaged knot-time scaling (Vhat, Ahat)
   - L-BFGS optimization over interior (P_k, Vhat_k, Ahat_k) and segment times tau_s
     with analytical jerk integral, time cost, GCOPTER smoothed-L1 corridor/obstacle hinge,
     anchor cost, and dynamic limit (velocity/acceleration/jerk) penalties
5. 3D Trajectory Tracker (position + velocity feedforward + filtered yaw rate over MAVROS)
"""

import heapq
import math
import time

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.optimize import minimize
from sensor_msgs.msg import PointCloud2, PointField

from uav_control_mapping.drone_base import DroneBaseNode

# =============================================================================
# MIGHTY Configuration Constants (matching config/mighty.yaml & warehouse scale)
# =============================================================================
V_MAX = 1.6                     # [m/s] Max velocity
A_MAX = 3.0                     # [m/s^2] Max acceleration
J_MAX = 8.0                     # [m/s^3] Max jerk
W_MAX = 0.7                     # [rad/s] Max yaw rate
TAKEOFF_ALT = 1.5               # [m] Initial takeoff hover altitude
MIN_FLIGHT_Z = 0.5              # [m] Minimum flight altitude
MAX_FLIGHT_Z = 4.5              # [m] Maximum flight altitude
GOAL_RADIUS = 0.30              # [m] Goal reached threshold
HORIZON = 12.0                  # [m] Local trajectory optimization horizon
NUM_N = 4                       # [-] Max number of spline segments in local horizon
MAX_DIST_VERTEXES = 2.0         # [m] Max distance between global path vertices

# Map & HGP Global Planner parameters
GRID_RES = 0.30                 # [m] Voxel resolution
INFLATION_XY = 4                # [cells] 1.20 m XY obstacle inflation (seals rack holes)
INFLATION_Z = 3                 # [cells] 0.90 m Z obstacle inflation (seals shelf gaps)
MIN_OBSTACLE_Z = 0.25           # [m] Filter floor points below this height
SELF_FILTER_RADIUS = 0.65       # [m] Ignore lidar returns on drone body
HEURISTIC_WEIGHT = 2.0          # [-] eps in weighted A* (global_planner_heuristic_weight)
HEAT_WEIGHT = 5.0               # [-] Weight on static boundary heat in A*
STATIC_HEAT_ALPHA = 2.0         # [-] Peak heat at obstacle boundary
STATIC_HEAT_P = 2               # [-] Radial falloff power
STATIC_HEAT_RADIUS_CELLS = 3    # [cells] Boundary heat halo radius beyond inflation
LOS_CELLS = 1                   # [cells] Capsule radius for LoS shortcutting
MIN_LEN = 0.8                   # [m] Minimum segment length after shortcutting
MIN_TURN_DEG = 10.0             # [deg] Minimum turn angle filter
MAX_ASTAR_EXPAND = 60000        # [-] Max A* expansions

# L-BFGS Hermite Spline Optimizer weights (from config/mighty.yaml)
TIME_WEIGHT = 50.0              # [-] Weight on total trajectory time sum(T)
JERK_WEIGHT = 0.2               # [-] Weight on integrated squared jerk
STAT_WEIGHT = 500.0             # [-] Weight on static corridor/obstacle violation
POS_ANCHOR_WEIGHT = 15.0        # [-] Weight keeping interior knots near HGP path
DYN_CONSTR_VEL_WEIGHT = 200.0   # [-] Weight on velocity limit violation
DYN_CONSTR_ACC_WEIGHT = 100.0   # [-] Weight on acceleration limit violation
DYN_CONSTR_JERK_WEIGHT = 50.0   # [-] Weight on jerk limit violation
PLANNER_CO = 0.20               # [m] Additional static corridor clearance margin
HINGE_MU = 1e-2                 # [-] GCOPTER smoothed-L1 mu
INTEGRAL_RES = 8                # [-] Quadrature samples per segment (integral_resolution)
LBFGS_MAX_ITER = 15             # [-] Max L-BFGS iterations per replan (warm-started from closed-form min-jerk)
TRAJ_DT = 0.05                  # [s] Trajectory sampling step (20 Hz control loop)
KP_TRACK = 1.1                  # [-] Position feedback gain added to spline velocity feedforward
KP_YAW = 1.2                    # [-] Yaw tracking proportional gain
ALPHA_FILTER_DYAW = 0.8         # [-] Yaw rate low-pass filter (from mighty.yaml)


# =============================================================================
# Precomputed 3D Neighborhoods
# =============================================================================
NEIGHBORS_26 = [
    (dx, dy, dz, math.sqrt(dx * dx + dy * dy + dz * dz))
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0)
]

NEIGHBORS_6 = [
    (1, 0, 0), (-1, 0, 0),
    (0, 1, 0), (0, -1, 0),
    (0, 0, 1), (0, 0, -1),
]

INFLATION_OFFSETS = [
    (dx, dy, dz)
    for dx in range(-INFLATION_XY, INFLATION_XY + 1)
    for dy in range(-INFLATION_XY, INFLATION_XY + 1)
    for dz in range(-INFLATION_Z, INFLATION_Z + 1)
    if dx * dx + dy * dy <= INFLATION_XY * INFLATION_XY + 1
]

# Precompute static boundary heat offsets: (dx, dy, dz, heat_value)
HEAT_OFFSETS = []
for _dx in range(-STATIC_HEAT_RADIUS_CELLS, STATIC_HEAT_RADIUS_CELLS + 1):
    for _dy in range(-STATIC_HEAT_RADIUS_CELLS, STATIC_HEAT_RADIUS_CELLS + 1):
        for _dz in range(-STATIC_HEAT_RADIUS_CELLS, STATIC_HEAT_RADIUS_CELLS + 1):
            _d = math.sqrt(_dx * _dx + _dy * _dy + _dz * _dz)
            if 0 < _d <= STATIC_HEAT_RADIUS_CELLS:
                _u = min(1.0, _d / float(STATIC_HEAT_RADIUS_CELLS))
                _h = STATIC_HEAT_ALPHA * ((1.0 - _u) ** STATIC_HEAT_P)
                if _h > 1e-4:
                    HEAT_OFFSETS.append((_dx, _dy, _dz, _h))


# =============================================================================
# MIGHTY Mathematical Helpers (from lbfgs_solver.cpp & hermite_test_unconstrained_opt_3d)
# =============================================================================
def quaternion_to_yaw(q) -> float:
    """Extract yaw (radians) from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(val: float, limit: float) -> float:
    return max(-limit, min(limit, val))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def gcopter_tau_to_T(tau: np.ndarray) -> np.ndarray:
    """GCOPTER C^2 bijection tau -> T > 0 (lbfgs_solver.cpp: tau_to_T)."""
    tau = np.asarray(tau, dtype=float)
    T = np.empty_like(tau)
    pos = tau >= 0.0
    T[pos] = 0.5 * tau[pos] ** 2 + tau[pos] + 1.0
    den = 0.5 * tau[~pos] ** 2 - tau[~pos] + 1.0
    T[~pos] = 1.0 / den
    return T


def gcopter_T_to_tau(T: np.ndarray) -> np.ndarray:
    """Inverse GCOPTER bijection T -> tau (lbfgs_solver.cpp: T_to_tau)."""
    T = np.asarray(T, dtype=float)
    tau = np.empty_like(T)
    ge1 = T >= 1.0
    tau[ge1] = np.sqrt(np.maximum(2.0 * T[ge1] - 1.0, 0.0)) - 1.0
    tau[~ge1] = 1.0 - np.sqrt(np.maximum(2.0 / T[~ge1] - 1.0, 0.0))
    return tau


def smoothed_l1(x: np.ndarray, mu: float = HINGE_MU) -> np.ndarray:
    """GCOPTER C^2 cubic hinge psi(x; mu) (lbfgs_solver.cpp: smoothed_l1)."""
    x = np.asarray(x, dtype=float)
    out = np.zeros_like(x)
    m1 = (x > 0.0) & (x < mu)
    m2 = x >= mu
    xd = x[m1] / mu
    out[m1] = (mu - 0.5 * x[m1]) * (xd * xd * xd)
    out[m2] = x[m2] - 0.5 * mu
    return out


def K_func(T: float) -> np.ndarray:
    """4x4 Hessian block for exact integrated squared jerk of a quintic Hermite segment."""
    T2 = T * T
    T3 = T2 * T
    return np.array([
        [192.0 / T3, 36.0 / T2, 168.0 / T3, -24.0 / T2],
        [36.0 / T2, 9.0 / T, 24.0 / T2, -3.0 / T],
        [168.0 / T3, 24.0 / T2, 192.0 / T3, -36.0 / T2],
        [-24.0 / T2, -3.0 / T, -36.0 / T2, 9.0 / T],
    ], dtype=float)


def k_const_func(T: float, P0: np.ndarray, P1: np.ndarray) -> np.ndarray:
    """4x3 linear term in (V0, A0, V1, A1) for integrated squared jerk of a quintic segment."""
    dP = P1 - P0  # shape (3,)
    T3 = T * T * T
    T4 = T3 * T
    return np.vstack([
        -360.0 * dP / T4,
        -60.0 * dP / T3,
        -360.0 * dP / T4,
        60.0 * dP / T3,
    ])


def solve_minjerk_vel_acc_closedform(
    wps: np.ndarray,
    T: np.ndarray,
    v0: np.ndarray,
    a0: np.ndarray,
    vf: np.ndarray,
    af: np.ndarray,
):
    """MIGHTY closed-form minimum-jerk solver for interior knot (V, A).

    Assembles the block-tridiagonal system H X = -b and solves for interior
    velocities V[1..M-1] and accelerations A[1..M-1] in closed form.
    """
    M, d = len(T), wps.shape[1]
    N = M - 1
    V = np.zeros((M + 1, d), dtype=float)
    A = np.zeros((M + 1, d), dtype=float)
    V[0], A[0] = v0, a0
    V[M], A[M] = vf, af

    if N <= 0:
        return V, A

    H = np.zeros((2 * N, 2 * N), dtype=float)
    b = np.zeros((2 * N, d), dtype=float)

    for r in range(M):
        Tr = max(float(T[r]), 0.1)
        Kr = K_func(Tr)
        kr = k_const_func(Tr, wps[r], wps[r + 1])

        if r >= 1:
            i = r - 1
            Ii = slice(2 * i, 2 * i + 2)
            H[Ii, Ii] += Kr[0:2, 0:2]
            b[Ii, :] += kr[0:2, :]

        if r <= M - 2:
            j = r
            Jj = slice(2 * j, 2 * j + 2)
            H[Jj, Jj] += Kr[2:4, 2:4]
            b[Jj, :] += kr[2:4, :]

        if 1 <= r <= M - 2:
            Ii = slice(2 * (r - 1), 2 * (r - 1) + 2)
            Jj = slice(2 * r, 2 * r + 2)
            H[Ii, Jj] += Kr[0:2, 2:4]
            H[Jj, Ii] += Kr[2:4, 0:2]

    K0 = K_func(max(float(T[0]), 0.1))
    v0a0 = np.vstack([v0, a0])
    b[0:2, :] += K0[2:4, 0:2] @ v0a0

    KM = K_func(max(float(T[-1]), 0.1))
    vfaf = np.vstack([vf, af])
    idx = 2 * (M - 2)
    b[idx:idx + 2, :] += KM[0:2, 2:4] @ vfaf

    # Regularize slightly for numerical stability
    H += 1e-9 * np.eye(2 * N)
    X = np.linalg.solve(H, -b)

    for i in range(1, M):
        V[i] = X[2 * (i - 1)]
        A[i] = X[2 * (i - 1) + 1]

    return V, A


def allocate_times_with_vector_velocities(
    gwp: np.ndarray,
    v_max: float,
    v0_norm: float,
    vf_norm: float,
    min_speed_frac: float = 0.25,
) -> np.ndarray:
    """MIGHTY turn-aware initial segment time allocation."""
    M = gwp.shape[0] - 1
    deltas = gwp[1:] - gwp[:-1]
    dists = np.linalg.norm(deltas, axis=1)
    dirs = np.zeros_like(deltas)
    nz = dists > 1e-6
    dirs[nz] = deltas[nz] / dists[nz, None]
    dirs[~nz] = np.array([1.0, 0.0, 0.0])

    v_min = v_max * min_speed_frac
    s = np.empty(M + 1, dtype=float)
    s[0] = v0_norm
    s[-1] = vf_norm
    if M > 1:
        cosangs = np.einsum('ij,ij->i', dirs[:-1], dirs[1:])
        cosangs = np.clip(cosangs, -1.0, 1.0)
        angles = np.arccos(cosangs)
        factors = 1.0 - angles / math.pi
        s[1:-1] = np.maximum(v_min, factors * v_max)

    v_vec = np.zeros((M + 1, 3), dtype=float)
    v_vec[0] = dirs[0] * s[0]
    v_vec[-1] = dirs[-1] * s[-1]
    for i in range(1, M):
        bis = dirs[i - 1] + dirs[i]
        n = np.linalg.norm(bis)
        bis = bis / n if n > 1e-6 else dirs[i]
        v_vec[i] = bis * s[i]

    if M >= 2:
        v_vec[-2] *= 0.9

    v_avg = 0.5 * (v_vec[:-1] + v_vec[1:])
    speeds = np.maximum(np.linalg.norm(v_avg, axis=1), v_min)
    Tinit = np.maximum(dists / speeds, 0.25)
    return Tinit


def quintic_hermite_coeffs(
    P0: np.ndarray,
    V0: np.ndarray,
    A0: np.ndarray,
    P1: np.ndarray,
    V1: np.ndarray,
    A1: np.ndarray,
    T: float,
):
    """Closed-form quintic polynomial coefficients c0..c5 in R^3 for segment duration T."""
    T2 = T * T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T
    dP = P1 - P0
    c0 = P0
    c1 = V0
    c2 = 0.5 * A0
    c3 = (20.0 * dP - (8.0 * V1 + 12.0 * V0) * T - (3.0 * A0 - A1) * T2) / (2.0 * T3)
    c4 = (-30.0 * dP + (14.0 * V1 + 16.0 * V0) * T + (3.0 * A0 - 2.0 * A1) * T2) / (2.0 * T4)
    c5 = (12.0 * dP - (6.0 * V1 + 6.0 * V0) * T - (A0 - A1) * T2) / (2.0 * T5)
    return c0, c1, c2, c3, c4, c5


def exact_segment_jerk_sq_integral(
    c3: np.ndarray, c4: np.ndarray, c5: np.ndarray, T: float
) -> float:
    """Analytical integral int_0^T ||j(t)||^2 dt where j(t) = 6*c3 + 24*c4*t + 60*c5*t^2."""
    T2 = T * T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T
    c3_sq = float(np.dot(c3, c3))
    c4_sq = float(np.dot(c4, c4))
    c5_sq = float(np.dot(c5, c5))
    c3_c4 = float(np.dot(c3, c4))
    c3_c5 = float(np.dot(c3, c5))
    c4_c5 = float(np.dot(c4, c5))
    return (
        36.0 * c3_sq * T
        + 144.0 * c3_c4 * T2
        + (192.0 * c4_sq + 240.0 * c3_c5) * T3
        + 720.0 * c4_c5 * T4
        + 720.0 * c5_sq * T5
    )


# =============================================================================
# Main MIGHTY ROS 2 Node
# =============================================================================
class MightyNavNode(DroneBaseNode):
    """MIGHTY 3D global + Hermite spline local planner for PX4 via MAVROS."""

    def __init__(self):
        super().__init__('mighty_nav_node')

        # 3D Voxel Map & Static Heat Map (MapUtil equivalent)
        self.raw_obstacles = set()
        self.obstacles = set()
        self.static_heat = {}  # (ix, iy, iz) -> float heat value

        # Flight & Trajectory State
        self.takeoff_initialized = False
        self.is_taken_off = False
        self.hover_pos = np.array([0.0, 0.0, TAKEOFF_ALT], dtype=float)
        self.current_vel_est = np.zeros(3, dtype=float)
        self.prev_pos_for_vel = None
        self.prev_pos_time = None
        self.filtered_dyaw = 0.0

        # Active trajectory plan: list of (t_rel, pos_3d, vel_3d, acc_3d)
        self.traj_start_time = 0.0
        self.traj_duration = 0.0
        self.traj_pos = np.zeros((0, 3), dtype=float)
        self.traj_vel = np.zeros((0, 3), dtype=float)
        self.traj_times = np.zeros(0, dtype=float)
        self.terminal_goal = None
        self.active_global_path = []

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10
        )
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.hgp_path_pub = self.create_publisher(Path, '/hgp_global_path', 10)

        # Subscribers (Point-LIO live cloud + OctoMap occupied centers + RViz clicked point)
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

        # 20 Hz control + replan monitor timer
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)
        self.get_logger().info(
            'MIGHTY Navigation Node started (HGP astar_heat + Quintic Hermite Spline L-BFGS).'
        )

    # ------------------------------------------------------------------
    # MapUtil: Voxel Grid, Inflation, and Static Boundary Heat Halo
    # ------------------------------------------------------------------
    def world_to_grid(self, x: float, y: float, z: float):
        return (
            int(math.floor(x / GRID_RES)),
            int(math.floor(y / GRID_RES)),
            int(math.floor(z / GRID_RES)),
        )

    def grid_to_world(self, ix: int, iy: int, iz: int) -> np.ndarray:
        return np.array([
            (ix + 0.5) * GRID_RES,
            (iy + 0.5) * GRID_RES,
            (iz + 0.5) * GRID_RES,
        ], dtype=float)

    def is_occupied_world(self, pt: np.ndarray) -> bool:
        if pt[2] < MIN_FLIGHT_Z or pt[2] > MAX_FLIGHT_Z:
            return True
        return self.world_to_grid(float(pt[0]), float(pt[1]), float(pt[2])) in self.obstacles

    def get_heat_grid(self, vox) -> float:
        return self.static_heat.get(vox, 0.0)

    def cloud_cb(self, msg: PointCloud2):
        self._ingest_pointcloud(msg, self_filter=True)

    def octomap_cb(self, msg: PointCloud2):
        self._ingest_pointcloud(msg, self_filter=True)

    def _ingest_pointcloud(self, msg: PointCloud2, self_filter: bool = True):
        total = msg.width * msg.height
        if total == 0:
            return

        offs = {f.name: f.offset for f in msg.fields if f.datatype == PointField.FLOAT32}
        if 'x' not in offs or 'y' not in offs or 'z' not in offs:
            return

        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(total, msg.point_step)
        x = raw[:, offs['x']:offs['x'] + 4].view(np.float32).reshape(-1)
        y = raw[:, offs['y']:offs['y'] + 4].view(np.float32).reshape(-1)
        z = raw[:, offs['z']:offs['z'] + 4].view(np.float32).reshape(-1)

        valid = (
            np.isfinite(x)
            & np.isfinite(y)
            & np.isfinite(z)
            & (z > MIN_OBSTACLE_Z)
            & (z < MAX_FLIGHT_Z + 1.0)
        )
        if not np.any(valid):
            return
        x, y, z = x[valid], y[valid], z[valid]

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
        new_raw = vox_tuples - self.raw_obstacles
        if not new_raw:
            return

        self.raw_obstacles.update(new_raw)
        obs = self.obstacles
        newly_inflated = []
        for vx, vy, vz in new_raw:
            for dx, dy, dz in INFLATION_OFFSETS:
                cell = (vx + dx, vy + dy, vz + dz)
                if cell not in obs:
                    obs.add(cell)
                    newly_inflated.append(cell)

        # Update static obstacle boundary heat halo (map_util.hpp step 10c)
        heat = self.static_heat
        for cx, cy, cz in newly_inflated:
            heat.pop((cx, cy, cz), None)
            # 6-neighborhood boundary test
            is_boundary = False
            for nx, ny, nz in NEIGHBORS_6:
                if (cx + nx, cy + ny, cz + nz) not in obs:
                    is_boundary = True
                    break
            if not is_boundary:
                continue
            for dx, dy, dz, h_val in HEAT_OFFSETS:
                nb = (cx + dx, cy + dy, cz + dz)
                if nb not in obs:
                    if h_val > heat.get(nb, 0.0):
                        heat[nb] = h_val

    # ------------------------------------------------------------------
    # Goal Sanitization (MIGHTY::sanitizeTerminalGoal)
    # ------------------------------------------------------------------
    def sanitize_voxel(self, target_vox, ref_vox, max_radius: int = 12):
        """Relocate an occupied start/goal voxel to the nearest free voxel biased toward ref_vox."""
        min_iz = int(math.floor(MIN_FLIGHT_Z / GRID_RES))
        max_iz = int(math.floor(MAX_FLIGHT_Z / GRID_RES))
        if target_vox not in self.obstacles and min_iz <= target_vox[2] <= max_iz:
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
                        if not (min_iz <= cand[2] <= max_iz) or cand in self.obstacles:
                            continue
                        dist_t = math.sqrt(dx * dx + dy * dy + dz * dz)
                        dist_r = math.sqrt(
                            (cand[0] - rx) ** 2 + (cand[1] - ry) ** 2 + (cand[2] - rz) ** 2
                        )
                        # Prefer low-heat, close-to-target, drone-side standoff cells
                        h_pen = 0.25 * self.get_heat_grid(cand)
                        score = dist_t + 0.15 * dist_r + h_pen
                        if score < best_score:
                            best_score = score
                            best_vox = cand
            if best_vox is not None:
                return best_vox
        return best_vox

    # ------------------------------------------------------------------
    # Stage 1: HGP Global Planner (graph_search.cpp + hgp_planner.cpp)
    # ------------------------------------------------------------------
    def astar_heat_search(self, start_vox, goal_vox):
        """MIGHTY 'astar_heat' weighted 3D A* with static boundary heat & partial recovery."""
        if start_vox == goal_vox:
            return [start_vox]

        min_iz = int(math.floor(MIN_FLIGHT_Z / GRID_RES))
        max_iz = int(math.floor(MAX_FLIGHT_Z / GRID_RES))
        gx, gy, gz = goal_vox
        obs = self.obstacles
        heat = self.static_heat

        def heur(v):
            return HEURISTIC_WEIGHT * math.sqrt(
                (v[0] - gx) ** 2 + (v[1] - gy) ** 2 + (v[2] - gz) ** 2
            )

        h0 = heur(start_vox)
        open_heap = [(h0, 0.0, start_vox)]
        came_from = {}
        g_score = {start_vox: 0.0}
        closed = set()

        best_node = start_vox
        best_h = h0

        expansions = 0
        while open_heap and expansions < MAX_ASTAR_EXPAND:
            expansions += 1
            _, curr_g, curr = heapq.heappop(open_heap)

            if curr in closed:
                continue
            closed.add(curr)

            curr_h = heur(curr)
            if curr_h < best_h:
                best_h = curr_h
                best_node = curr

            if curr == goal_vox:
                best_node = curr
                break

            cx, cy, cz = curr
            for dx, dy, dz, step_len in NEIGHBORS_26:
                nb = (cx + dx, cy + dy, cz + dz)
                if not (min_iz <= nb[2] <= max_iz) or nb in closed or nb in obs:
                    continue
                # Prevent diagonal corner-cutting across obstacle blocks
                if (
                    (dx != 0 and (cx + dx, cy, cz) in obs)
                    or (dy != 0 and (cx, cy + dy, cz) in obs)
                    or (dz != 0 and (cx, cy, cz + dz) in obs)
                ):
                    continue

                # Edge cost: geometric distance + weighted static heat (graph_search.cpp)
                h_nb = heat.get(nb, 0.0)
                edge_cost = step_len * (1.0 + HEAT_WEIGHT * h_nb)
                tentative_g = curr_g + edge_cost

                if tentative_g < g_score.get(nb, float('inf')):
                    came_from[nb] = curr
                    g_score[nb] = tentative_g
                    heapq.heappush(open_heap, (tentative_g + heur(nb), tentative_g, nb))

        if best_node == start_vox and goal_vox not in came_from:
            return None

        # Recover full or best partial path (graph_search.cpp: recoverPath)
        path = [best_node]
        curr = best_node
        while curr in came_from:
            curr = came_from[curr]
            path.append(curr)
        path.reverse()
        return path

    def line_of_sight_capsule(self, a: np.ndarray, b: np.ndarray, radius_cells: int) -> bool:
        """MIGHTY HGPPlanner::lineOfSightCapsule (hgp_planner.cpp:56)."""
        d = b - a
        L = float(np.linalg.norm(d))
        if L < 1e-6:
            return True

        u = d / L
        step = max(0.25 * GRID_RES, 0.08)
        radial = max(1, int(radius_cells))
        max_r = radius_cells * GRID_RES

        s = 0.0
        while s <= L:
            c = a + s * u
            if self.is_occupied_world(c):
                return False
            if radius_cells > 0:
                for ix in range(-radial, radial + 1):
                    for iy in range(-radial, radial + 1):
                        for iz in range(-radial, radial + 1):
                            if ix == 0 and iy == 0 and iz == 0:
                                continue
                            off = np.array([ix * GRID_RES, iy * GRID_RES, iz * GRID_RES])
                            if np.linalg.norm(off) > max_r:
                                continue
                            if self.is_occupied_world(c + off):
                                return False
            s += step
        return True

    def shortcut_by_los(self, path: list, radius_cells: int) -> list:
        """MIGHTY HGPPlanner::shortCutByLoS (hgp_planner.cpp:89)."""
        if len(path) < 2:
            return path
        out = [path[0]]
        i = 0
        while i + 1 < len(path):
            best = i + 1
            for j in range(i + 2, len(path)):
                if self.line_of_sight_capsule(path[i], path[j], radius_cells):
                    best = j
                else:
                    break
            out.append(path[best])
            i = best
        return out

    def remove_line_pts(self, path: list) -> list:
        """MIGHTY HGPPlanner::removeLinePts (hgp_planner.cpp:285)."""
        if len(path) < 3:
            return path
        out = [path[0]]
        for i in range(1, len(path) - 1):
            p = (path[i + 1] - path[i]) - (path[i] - path[i - 1])
            if float(np.sum(np.abs(p))) > 1e-2:
                out.append(path[i])
        out.append(path[-1])
        return out

    def seg_cost_and_peak_heat(self, a: np.ndarray, b: np.ndarray):
        """MIGHTY HGPPlanner::removeCornerPts segment cost with heat integral & peak guardrail."""
        if not self.line_of_sight_capsule(a, b, LOS_CELLS):
            return float('inf'), float('inf')
        d = b - a
        L = float(np.linalg.norm(d))
        if L < 1e-9:
            return 0.0, 0.0
        u = d / L
        ds = max(0.5 * GRID_RES, 0.08)
        heat_int = 0.0
        peak_h = 0.0
        s = 0.0
        while s <= L:
            p = a + s * u
            vox = self.world_to_grid(float(p[0]), float(p[1]), float(p[2]))
            h = self.get_heat_grid(vox)
            peak_h = max(peak_h, h)
            heat_int += h * ds
            s += ds
        return L + HEAT_WEIGHT * heat_int, peak_h

    def remove_corner_pts(self, path: list) -> list:
        """MIGHTY HGPPlanner::removeCornerPts (hgp_planner.cpp:165)."""
        if len(path) < 3:
            return path
        opt = [path[0]]
        prev_pose = path[0]
        cost1, peak1 = self.seg_cost_and_peak_heat(path[0], path[1])

        for i in range(1, len(path) - 1):
            pose1 = path[i]
            pose2 = path[i + 1]
            cost2, peak2 = self.seg_cost_and_peak_heat(pose1, pose2)
            cost3, peak3 = self.seg_cost_and_peak_heat(prev_pose, pose2)

            accept = False
            if cost3 < cost1 + cost2:
                ref_peak = max(peak1, peak2)
                if peak3 <= ref_peak * 1.10 + 1e-3:
                    accept = True

            if accept:
                cost1, peak1 = cost3, peak3
            else:
                opt.append(pose1)
                prev_pose = pose1
                cost1, peak1 = cost2, peak2

        opt.append(path[-1])
        return opt

    def clean_up_path(self, path: list) -> list:
        """MIGHTY HGPPlanner::cleanUpPath (hgp_planner.cpp:571)."""
        path = self.remove_line_pts(path)
        path = self.remove_corner_pts(path)
        path = list(reversed(path))
        path = self.remove_corner_pts(path)
        path = list(reversed(path))
        return path

    def subdivide_long_segments(self, path: list, max_dist: float) -> list:
        """MIGHTY HGPManager segment subdivision (max_dist_vertexes)."""
        if len(path) < 2:
            return path
        out = [path[0]]
        for i in range(len(path) - 1):
            a, b = path[i], path[i + 1]
            dist = float(np.linalg.norm(b - a))
            n_sub = max(1, int(math.ceil(dist / max_dist)))
            for k in range(1, n_sub + 1):
                out.append(a + (k / float(n_sub)) * (b - a))
        return out

    def plan_hgp_global_path(self, start_pos: np.ndarray, goal_pos: np.ndarray):
        """Run full MIGHTY HGP global planning pipeline."""
        raw_start = self.world_to_grid(float(start_pos[0]), float(start_pos[1]), float(start_pos[2]))
        raw_goal = self.world_to_grid(float(goal_pos[0]), float(goal_pos[1]), float(goal_pos[2]))

        start_vox = self.sanitize_voxel(raw_start, raw_goal, max_radius=8)
        if start_vox is None:
            start_vox = raw_start

        goal_vox = self.sanitize_voxel(raw_goal, start_vox, max_radius=12)
        if goal_vox is None:
            self.get_logger().warn('HGP: Goal is completely surrounded by obstacles!')
            return None

        if goal_vox != raw_goal:
            snapped_w = self.grid_to_world(*goal_vox)
            self.get_logger().info(
                f'HGP sanitizeTerminalGoal: relocated wall goal to ({snapped_w[0]:.2f}, {snapped_w[1]:.2f}, {snapped_w[2]:.2f})'
            )

        vox_path = self.astar_heat_search(start_vox, goal_vox)
        if not vox_path:
            return None

        # Convert voxel path to continuous coordinates with exact start position
        raw_path = [self.grid_to_world(*v) for v in vox_path]
        raw_path[0] = start_pos.copy()

        # MIGHTY HGP post-processing pipeline
        path = self.shortcut_by_los(raw_path, LOS_CELLS)
        path = self.clean_up_path(path)
        path = self.subdivide_long_segments(path, MAX_DIST_VERTEXES)
        return path

    # ------------------------------------------------------------------
    # Stage 2: Safe Corridor Halfspaces (EllipsoidDecomp3D approximation)
    # ------------------------------------------------------------------
    def build_safe_corridor_halfspaces(self, wps: np.ndarray):
        """Build per-segment linear halfspace constraints A_s x <= b_s (lbfgs_solver.cpp).

        For each segment [P_s, P_{s+1}], finds nearby obstacle voxels in the local
        bounding box and constructs separating halfspaces + altitude bounds.
        """
        M = wps.shape[0] - 1
        A_stat = []
        b_stat = []

        # Collect occupied voxel world coordinates near the trajectory
        obs_pts = None
        if self.raw_obstacles:
            raw_arr = np.array(list(self.raw_obstacles), dtype=float)
            obs_pts = (raw_arr + 0.5) * GRID_RES

        clearance = INFLATION_XY * GRID_RES * 0.75 + PLANNER_CO

        for s in range(M):
            p0 = wps[s]
            p1 = wps[s + 1]
            mid = 0.5 * (p0 + p1)
            seg_vec = p1 - p0
            seg_len = float(np.linalg.norm(seg_vec))
            u = seg_vec / seg_len if seg_len > 1e-6 else np.array([1.0, 0.0, 0.0])

            planes_A = []
            planes_b = []

            # Floor and ceiling halfspaces: -z <= -MIN_FLIGHT_Z and z <= MAX_FLIGHT_Z
            planes_A.append(np.array([0.0, 0.0, -1.0]))
            planes_b.append(-MIN_FLIGHT_Z)
            planes_A.append(np.array([0.0, 0.0, 1.0]))
            planes_b.append(MAX_FLIGHT_Z)

            # Obstacle separating halfspaces around segment
            if obs_pts is not None and obs_pts.shape[0] > 0:
                # Filter obstacle points within local bounding box (3m around segment midpoint)
                diff_mid = np.abs(obs_pts - mid)
                box_rad = 0.5 * seg_len + 2.5
                near_mask = (diff_mid[:, 0] < box_rad) & (diff_mid[:, 1] < box_rad) & (diff_mid[:, 2] < 2.5)
                near_obs = obs_pts[near_mask]

                # Add up to 6 closest obstacle planes for this segment
                remaining = near_obs
                for _ in range(6):
                    if remaining.shape[0] == 0:
                        break
                    # Closest point on segment [p0, p1] to each obstacle point
                    rel = remaining - p0
                    proj = np.clip(rel @ u, 0.0, seg_len)
                    closest_on_seg = p0 + proj[:, None] * u[None, :]
                    vecs = remaining - closest_on_seg
                    dists = np.linalg.norm(vecs, axis=1)
                    min_idx = int(np.argmin(dists))
                    d_min = float(dists[min_idx])
                    if d_min > 2.5 or d_min < 1e-4:
                        break

                    # Normal pointing from segment toward obstacle: n^T x <= b
                    obs_pt = remaining[min_idx]
                    seg_pt = closest_on_seg[min_idx]
                    n = (obs_pt - seg_pt) / d_min
                    # Place plane at safety clearance from obstacle, ensuring p0 & p1 stay feasible
                    b_val = float(np.dot(n, obs_pt)) - clearance
                    b_val = max(b_val, float(np.dot(n, p0)) + 0.05, float(np.dot(n, p1)) + 0.05)

                    planes_A.append(n)
                    planes_b.append(b_val)

                    # Remove obstacles already behind this halfspace
                    keep = (remaining @ n) < (b_val + clearance)
                    keep[min_idx] = False
                    remaining = remaining[keep]

            A_stat.append(np.vstack(planes_A))
            b_stat.append(np.array(planes_b, dtype=float))

        return A_stat, b_stat

    # ------------------------------------------------------------------
    # Stage 3: MIGHTY Local Trajectory Optimizer (Quintic Hermite + L-BFGS)
    # ------------------------------------------------------------------
    def optimize_local_hermite_trajectory(
        self,
        global_wps: np.ndarray,
        v0: np.ndarray,
        a0: np.ndarray,
        vf: np.ndarray,
        af: np.ndarray,
    ):
        """MIGHTY SolverLBFGS: optimizes quintic Hermite spline (P, Vhat, Ahat, tau)."""
        # Ensure at least 3 knots (M >= 2 segments), matching mighty.cpp:1101
        wps = global_wps.copy()
        if wps.shape[0] == 2:
            mid = 0.5 * (wps[0] + wps[1])
            wps = np.vstack([wps[0], mid, wps[1]])

        # Truncate to NUM_N + 1 knots for fast local horizon optimization (mighty.cpp:1094)
        if wps.shape[0] > NUM_N + 1:
            wps = wps[:NUM_N + 1].copy()

        M = wps.shape[0] - 1
        N_int = M - 1  # number of free interior knots

        # 1) Build safe corridor halfspaces A_stat x <= b_stat
        A_stat, b_stat = self.build_safe_corridor_halfspaces(wps)
        P_anchor = wps.copy()

        # 2) Initial guess: turn-aware segment times + closed-form min-jerk (V, A)
        v0_norm = float(np.linalg.norm(v0))
        vf_norm = float(np.linalg.norm(vf))
        T_init = allocate_times_with_vector_velocities(wps, V_MAX, v0_norm, vf_norm)
        V_init, A_init = solve_minjerk_vel_acc_closedform(wps, T_init, v0, a0, vf, af)

        # Averaged knot times Tbar (lbfgs_solver.cpp: build_Tbar)
        Tbar_init = np.empty(M + 1, dtype=float)
        Tbar_init[0] = T_init[0]
        Tbar_init[1:M] = 0.5 * (T_init[:-1] + T_init[1:])
        Tbar_init[M] = T_init[-1]

        # Encode decision vector z0 = [P_1, Vhat_1, Ahat_1, ..., P_{M-1}, Vhat_{M-1}, Ahat_{M-1}, tau_0..tau_{M-1}]
        z_knots = np.zeros((N_int, 9), dtype=float)
        for k in range(1, M):
            z_knots[k - 1, 0:3] = wps[k]
            z_knots[k - 1, 3:6] = V_init[k] * Tbar_init[k]
            z_knots[k - 1, 6:9] = A_init[k] * (Tbar_init[k] ** 2)
        tau_init = gcopter_T_to_tau(T_init)
        z0 = np.concatenate([z_knots.ravel(), tau_init])

        x0_fixed = wps[0].copy()
        xf_fixed = wps[-1].copy()
        u_samples = np.linspace(0.0, 1.0, INTEGRAL_RES + 1)

        def decode_z(z: np.ndarray):
            """Reconstruct (P, V, A, T) from unconstrained decision vector z."""
            tau = z[9 * N_int:]
            T = gcopter_tau_to_T(tau)

            Tbar = np.empty(M + 1, dtype=float)
            Tbar[0] = T[0]
            Tbar[1:M] = 0.5 * (T[:-1] + T[1:])
            Tbar[M] = T[-1]

            P = np.zeros((M + 1, 3), dtype=float)
            V = np.zeros((M + 1, 3), dtype=float)
            A = np.zeros((M + 1, 3), dtype=float)

            P[0], V[0], A[0] = x0_fixed, v0, a0
            P[M], V[M], A[M] = xf_fixed, vf, af

            if N_int > 0:
                knots = z[:9 * N_int].reshape((N_int, 9))
                P[1:M] = knots[:, 0:3]
                V[1:M] = knots[:, 3:6] / Tbar[1:M, None]
                A[1:M] = knots[:, 6:9] / (Tbar[1:M, None] ** 2)

            return P, V, A, T

        def mighty_objective(z: np.ndarray) -> float:
            P, V, A, T = decode_z(z)

            # 1) Total time cost + soft minimum segment duration
            cost = TIME_WEIGHT * float(np.sum(T))
            short_viol = 0.20 - T
            if np.any(short_viol > 0.0):
                cost += 500.0 * float(np.sum(smoothed_l1(short_viol)))

            # 2) Position anchor cost (keeps interior knots near HGP path)
            if N_int > 0 and POS_ANCHOR_WEIGHT > 0.0:
                diff_p = P[1:M] - P_anchor[1:M]
                cost += POS_ANCHOR_WEIGHT * float(np.sum(diff_p * diff_p))

            # 3) Vectorized per-segment jerk, corridor safety, and dynamic constraint costs
            Ts = np.maximum(T, 1e-4)  # (M,)
            T2 = Ts * Ts
            T3 = T2 * Ts
            T4 = T3 * Ts
            T5 = T4 * Ts

            P0, V0, A0 = P[:-1], V[:-1], A[:-1]  # (M, 3)
            P1, V1, A1 = P[1:], V[1:], A[1:]     # (M, 3)

            c1 = V0
            c2 = 0.5 * A0
            C0 = P1 - (P0 + V0 * Ts[:, None] + 0.5 * A0 * T2[:, None])
            C1 = V1 - (V0 + A0 * Ts[:, None])
            C2 = A1 - A0

            c3 = (10.0 * C0 - 4.0 * C1 * Ts[:, None] + 0.5 * C2 * T2[:, None]) / T3[:, None]
            c4 = (-15.0 * C0 + 7.0 * C1 * Ts[:, None] - 1.0 * C2 * T2[:, None]) / T4[:, None]
            c5 = (6.0 * C0 - 3.0 * C1 * Ts[:, None] + 0.5 * C2 * T2[:, None]) / T5[:, None]

            # 3a) Exact analytical integrated squared jerk cost across all M segments
            c33 = np.sum(c3 * c3, axis=1)
            c34 = np.sum(c3 * c4, axis=1)
            c35 = np.sum(c3 * c5, axis=1)
            c44 = np.sum(c4 * c4, axis=1)
            c45 = np.sum(c4 * c5, axis=1)
            c55 = np.sum(c5 * c5, axis=1)
            jerk_seg = (
                36.0 * c33 * Ts
                + 144.0 * c34 * T2
                + (240.0 * c35 + 192.0 * c44) * T3
                + 720.0 * c45 * T4
                + 720.0 * c55 * T5
            )
            cost += JERK_WEIGHT * float(np.sum(jerk_seg))

            # 3b) Hermite -> Quintic Bezier control points (M, 6, 3) for corridor check
            cp1 = P0 + (Ts[:, None] / 5.0) * V0
            cp2 = P0 + (2.0 * Ts[:, None] / 5.0) * V0 + (T2[:, None] / 20.0) * A0
            cp3 = P1 - (2.0 * Ts[:, None] / 5.0) * V1 + (T2[:, None] / 20.0) * A1
            cp4 = P1 - (Ts[:, None] / 5.0) * V1
            cps_all = np.stack([P0, cp1, cp2, cp3, cp4, P1], axis=1)  # (M, 6, 3)

            for s in range(M):
                viol_corridor = (cps_all[s] @ A_stat[s].T) - b_stat[s][None, :]
                if np.any(viol_corridor > 0.0):
                    cost += STAT_WEIGHT * float(np.sum(smoothed_l1(viol_corridor)))

            # 3c) Vectorized velocity, acceleration, jerk samples across (M, K_s, 3)
            t_loc = Ts[:, None] * u_samples[None, :]  # (M, K)
            t2 = t_loc * t_loc
            t3 = t2 * t_loc
            t4 = t3 * t_loc

            vel_s = (
                c1[:, None, :]
                + 2.0 * t_loc[:, :, None] * c2[:, None, :]
                + 3.0 * t2[:, :, None] * c3[:, None, :]
                + 4.0 * t3[:, :, None] * c4[:, None, :]
                + 5.0 * t4[:, :, None] * c5[:, None, :]
            )
            acc_s = (
                2.0 * c2[:, None, :]
                + 6.0 * t_loc[:, :, None] * c3[:, None, :]
                + 12.0 * t2[:, :, None] * c4[:, None, :]
                + 20.0 * t3[:, :, None] * c5[:, None, :]
            )
            jerk_s = (
                6.0 * c3[:, None, :]
                + 24.0 * t_loc[:, :, None] * c4[:, None, :]
                + 60.0 * t2[:, :, None] * c5[:, None, :]
            )

            dt_w = (Ts / float(INTEGRAL_RES))[:, None]  # (M, 1)
            v_sq_viol = np.sum(vel_s * vel_s, axis=2) - (V_MAX * V_MAX)
            a_sq_viol = np.sum(acc_s * acc_s, axis=2) - (A_MAX * A_MAX)
            j_sq_viol = np.sum(jerk_s * jerk_s, axis=2) - (J_MAX * J_MAX)

            if np.any(v_sq_viol > 0.0):
                cost += DYN_CONSTR_VEL_WEIGHT * float(np.sum(dt_w * smoothed_l1(v_sq_viol)))
            if np.any(a_sq_viol > 0.0):
                cost += DYN_CONSTR_ACC_WEIGHT * float(np.sum(dt_w * smoothed_l1(a_sq_viol)))
            if np.any(j_sq_viol > 0.0):
                cost += DYN_CONSTR_JERK_WEIGHT * float(np.sum(dt_w * smoothed_l1(j_sq_viol)))

            return cost

        # Run L-BFGS optimization
        res = minimize(
            mighty_objective,
            z0,
            method='L-BFGS-B',
            options={'maxiter': LBFGS_MAX_ITER, 'ftol': 1e-4, 'gtol': 1e-3},
        )
        z_opt = res.x if np.all(np.isfinite(res.x)) else z0
        P_opt, V_opt, A_opt, T_opt = decode_z(z_opt)

        # Sample dense trajectory at TRAJ_DT (MIGHTY::getGoalSetpoints)
        ends = np.cumsum(T_opt)
        total_time = float(ends[-1])
        num_samples = max(2, int(math.ceil(total_time / TRAJ_DT)) + 1)
        sample_times = np.linspace(0.0, total_time, num_samples)

        coeffs_list = [
            quintic_hermite_coeffs(P_opt[s], V_opt[s], A_opt[s], P_opt[s + 1], V_opt[s + 1], A_opt[s + 1], float(T_opt[s]))
            for s in range(M)
        ]

        pos_out = np.zeros((num_samples, 3), dtype=float)
        vel_out = np.zeros((num_samples, 3), dtype=float)

        for idx, t_val in enumerate(sample_times):
            seg = min(int(np.searchsorted(ends, t_val, side='right')), M - 1)
            t0_seg = 0.0 if seg == 0 else float(ends[seg - 1])
            tl = min(max(float(t_val) - t0_seg, 0.0), float(T_opt[seg]))
            tl2 = tl * tl
            tl3 = tl2 * tl
            tl4 = tl3 * tl
            tl5 = tl4 * tl
            c0, c1, c2, c3, c4, c5 = coeffs_list[seg]
            pos_out[idx] = c0 + c1 * tl + c2 * tl2 + c3 * tl3 + c4 * tl4 + c5 * tl5
            vel_out[idx] = c1 + 2.0 * c2 * tl + 3.0 * c3 * tl2 + 4.0 * c4 * tl3 + 5.0 * c5 * tl4

        return sample_times, pos_out, vel_out, total_time

    # ------------------------------------------------------------------
    # Full Replan Pipeline (MIGHTY::replan in mighty.cpp:710)
    # ------------------------------------------------------------------
    def replan_to_goal(self, goal_3d: np.ndarray, frame_id: str = 'camera_init'):
        t_start = time.perf_counter()
        cp = self.current_pose.pose.position
        start_pos = np.array([cp.x, cp.y, max(cp.z, MIN_FLIGHT_Z)], dtype=float)

        # Stage 1: HGP Global Path (astar_heat + LoS shortcut + corner cleanup)
        global_path = self.plan_hgp_global_path(start_pos, goal_3d)
        if not global_path or len(global_path) < 2:
            self.get_logger().warn('MIGHTY HGP failed to find a global path!')
            return False

        self.active_global_path = global_path
        self.publish_path_msg(self.hgp_path_pub, global_path, frame_id)

        # Stage 2 & 3: Safe Corridor + Quintic Hermite Spline L-BFGS Optimization
        gwp = np.vstack(global_path)
        dist_to_term = float(np.linalg.norm(gwp[-1] - start_pos))
        # Terminal velocity is 0 if final goal is within local horizon, else cruise along path
        if len(global_path) <= NUM_N + 1 or dist_to_term < 3.0:
            vf = np.zeros(3, dtype=float)
        else:
            last_seg = gwp[NUM_N] - gwp[NUM_N - 1]
            norm_s = float(np.linalg.norm(last_seg))
            vf = (last_seg / norm_s) * (0.5 * V_MAX) if norm_s > 1e-6 else np.zeros(3, dtype=float)

        v0 = np.clip(self.current_vel_est, -V_MAX, V_MAX)
        a0 = np.zeros(3, dtype=float)
        af = np.zeros(3, dtype=float)

        times, pos_traj, vel_traj, total_T = self.optimize_local_hermite_trajectory(
            gwp, v0, a0, vf, af
        )

        self.traj_times = times
        self.traj_pos = pos_traj
        self.traj_vel = vel_traj
        self.traj_duration = total_T
        self.traj_start_time = self.get_clock().now().nanoseconds / 1e9
        self.hover_pos = pos_traj[-1].copy()

        elapsed_ms = (time.perf_counter() - t_start) * 1000.0
        self.get_logger().info(
            f'MIGHTY replan succeeded in {elapsed_ms:.1f} ms | '
            f'HGP wps: {len(global_path)} | Spline T={total_T:.2f}s ({len(pos_traj)} setpoints)'
        )
        self.publish_path_msg(self.path_pub, pos_traj, frame_id)
        return True

    def clicked_point_cb(self, msg: PointStamped):
        if not self.pose_received:
            self.get_logger().warn('Pose not received yet; ignoring clicked point.')
            return

        gx = float(msg.point.x)
        gy = float(msg.point.y)
        gz = float(np.clip(msg.point.z, MIN_FLIGHT_Z, MAX_FLIGHT_Z))
        self.terminal_goal = np.array([gx, gy, gz], dtype=float)
        self.get_logger().info(
            f'Received RViz goal ({gx:.2f}, {gy:.2f}, {gz:.2f}) -> running MIGHTY planner...'
        )
        self.replan_to_goal(self.terminal_goal, msg.header.frame_id or 'camera_init')

    def publish_path_msg(self, pub, pts, frame_id: str):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id
        for pt in pts:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.position.z = float(pt[2])
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        pub.publish(path_msg)

    # ------------------------------------------------------------------
    # 20 Hz Trajectory Tracker & Replan Loop
    # ------------------------------------------------------------------
    def _publish_zero(self):
        self.cmd_vel_pub.publish(Twist())

    def timer_callback(self):
        if not self.current_state.connected:
            return

        if not self.pose_received:
            self._publish_zero()
            return

        now_ros = self.get_clock().now()
        now_sec = now_ros.nanoseconds / 1e9
        cp = self.current_pose.pose.position
        curr_pos = np.array([cp.x, cp.y, cp.z], dtype=float)

        # Estimate drone current velocity for smooth Hermite boundary condition v0
        if self.prev_pos_for_vel is not None and self.prev_pos_time is not None:
            dt = now_sec - self.prev_pos_time
            if 1e-3 < dt < 0.5:
                raw_vel = (curr_pos - self.prev_pos_for_vel) / dt
                self.current_vel_est = 0.7 * self.current_vel_est + 0.3 * raw_vel
        self.prev_pos_for_vel = curr_pos.copy()
        self.prev_pos_time = now_sec

        if self.setpoint_counter < 100:
            self._publish_zero()
            self.setpoint_counter += 1
            return

        if not self.manage_offboard_and_arming(now_ros):
            self._publish_zero()
            return

        if not self.takeoff_initialized:
            self.takeoff_initialized = True
            target_z = curr_pos[2] if curr_pos[2] > 1.0 else TAKEOFF_ALT
            self.hover_pos = np.array([curr_pos[0], curr_pos[1], target_z], dtype=float)
            self.get_logger().info(
                f'Holding initial position ({self.hover_pos[0]:.2f}, {self.hover_pos[1]:.2f}, {self.hover_pos[2]:.2f})m.'
            )

        if not self.is_taken_off and curr_pos[2] >= TAKEOFF_ALT - 0.3:
            self.is_taken_off = True
            self.get_logger().info('Drone airborne! Ready for 3D RViz /clicked_point goals.')

        # Check active Hermite trajectory
        if self.traj_pos.shape[0] > 0:
            t_elapsed = now_sec - self.traj_start_time

            # Check if any upcoming trajectory sample is now inside a newly discovered obstacle
            idx_now = int(np.searchsorted(self.traj_times, t_elapsed, side='left'))
            idx_check_end = min(len(self.traj_pos), idx_now + 40)
            collision_ahead = False
            for k in range(idx_now, idx_check_end, 3):
                if self.is_occupied_world(self.traj_pos[k]):
                    collision_ahead = True
                    break

            if collision_ahead and self.terminal_goal is not None:
                self.get_logger().info('MIGHTY: Obstacle detected on active spline! Replanning...')
                self.replan_to_goal(self.terminal_goal)
                t_elapsed = 0.0

            if t_elapsed >= self.traj_duration:
                # Reached end of current local horizon spline
                dist_end = float(np.linalg.norm(self.traj_pos[-1] - curr_pos))
                if self.terminal_goal is not None:
                    dist_term = float(np.linalg.norm(self.terminal_goal - curr_pos))
                    if dist_term > GOAL_RADIUS + 0.5 and len(self.active_global_path) > NUM_N + 1:
                        # Continue replanning next horizon segment toward terminal goal
                        self.replan_to_goal(self.terminal_goal)
                        t_elapsed = 0.0
                    elif dist_end < GOAL_RADIUS:
                        self.get_logger().info(
                            f'MIGHTY: Goal reached at ({curr_pos[0]:.2f}, {curr_pos[1]:.2f}, {curr_pos[2]:.2f})!'
                        )
                        self.hover_pos = self.traj_pos[-1].copy()
                        self.traj_pos = np.zeros((0, 3), dtype=float)
                        self.terminal_goal = None

        # Compute reference (p_ref, v_ref) from Hermite spline or hover setpoint
        if self.traj_pos.shape[0] > 0:
            t_elapsed = float(np.clip(now_sec - self.traj_start_time, 0.0, self.traj_duration))
            idx = min(int(np.searchsorted(self.traj_times, t_elapsed, side='right')), len(self.traj_pos) - 1)
            p_ref = self.traj_pos[idx]
            v_ref = self.traj_vel[idx]
        else:
            p_ref = self.hover_pos
            v_ref = np.zeros(3, dtype=float)

        # Feedforward spline velocity + proportional position feedback
        pos_err = p_ref - curr_pos
        v_cmd = v_ref + KP_TRACK * pos_err

        # Clamp total 3D speed to V_MAX
        speed = float(np.linalg.norm(v_cmd))
        if speed > V_MAX:
            v_cmd = v_cmd * (V_MAX / speed)

        twist = Twist()
        twist.linear.x = float(v_cmd[0])
        twist.linear.y = float(v_cmd[1])
        twist.linear.z = float(v_cmd[2])

        # Smooth yaw alignment along horizontal tangent (matching MIGHTY's alpha_filter_dyaw)
        horiz_dir = p_ref[:2] - curr_pos[:2]
        if float(np.linalg.norm(v_ref[:2])) > 0.25:
            horiz_dir = v_ref[:2]

        raw_dyaw = 0.0
        if float(np.linalg.norm(horiz_dir)) > 0.25:
            current_yaw = quaternion_to_yaw(self.current_pose.pose.orientation)
            desired_yaw = math.atan2(float(horiz_dir[1]), float(horiz_dir[0]))
            yaw_err = wrap_angle(desired_yaw - current_yaw)
            raw_dyaw = float(clamp(KP_YAW * yaw_err, W_MAX))

        self.filtered_dyaw = (
            (1.0 - ALPHA_FILTER_DYAW) * self.filtered_dyaw + ALPHA_FILTER_DYAW * raw_dyaw
        )
        twist.angular.z = float(self.filtered_dyaw)

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MightyNavNode()

    node.get_logger().info('Waiting for FCU connection...')
    while rclpy.ok() and not node.current_state.connected:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info('FCU connected! MIGHTY planner ready for RViz /clicked_point goals.')

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
