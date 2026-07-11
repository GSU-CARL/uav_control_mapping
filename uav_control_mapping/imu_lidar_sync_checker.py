#!/usr/bin/env python3
"""
imu_lidar_sync_checker.py
─────────────────────────
Diagnostic node for PX4-SITL + Gazebo Harmonic + ROS 2 Jazzy setups.

What it checks
──────────────
1. Timestamp delta between every LiDAR scan header and the nearest IMU message
   → If |delta| > threshold → sync problem
2. LiDAR arrival jitter (wall-clock gap between consecutive scans)
   → High jitter → bridge or sim is stuttering
3. IMU arrival rate (actual Hz vs expected 200 Hz)
4. Per-point 'time' field sanity:
   → min/max/range of relative times in each cloud
   → Should span [0, ~0.1 s] at 600 RPM / 10 Hz scans
5. Clock source check:
   → Warns if header stamps look like wall-clock (sim_time not used)
6. Prints a live summary table every N scans

Run
───
    ros2 run <your_pkg> imu_lidar_sync_checker

Or directly:
    python3 imu_lidar_sync_checker.py

Parameters (ros2 param set or launch)
──────────────────────────────────────
  lidar_topic          (str)   default: /lidar_3d/points_timestamped
  imu_topic            (str)   default: /imu
  expected_lidar_hz    (float) default: 10.0
  expected_imu_hz      (float) default: 200.0
  sync_warn_thresh_ms  (float) default: 15.0   # warn if |lidar_ts - nearest_imu_ts| > this
  report_every_n_scans (int)   default: 10
"""

import math
import struct
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField, Imu

# ──────────────────────────────────────────────────────────────────────────────
RESET   = '\033[0m'
BOLD    = '\033[1m'
RED     = '\033[91m'
YELLOW  = '\033[93m'
GREEN   = '\033[92m'
CYAN    = '\033[96m'
MAGENTA = '\033[95m'
# ──────────────────────────────────────────────────────────────────────────────


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def sec_to_ms(s: float) -> float:
    return s * 1e3


class ImuLidarSyncChecker(Node):
    def __init__(self):
        super().__init__('imu_lidar_sync_checker')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('lidar_topic',          '/drone/lidar_3d/points_timestamped')
        self.declare_parameter('imu_topic',            '/drone/imu2')
        self.declare_parameter('expected_lidar_hz',    10.0)
        self.declare_parameter('expected_imu_hz',      250.0)
        self.declare_parameter('sync_warn_thresh_ms',  15.0)
        self.declare_parameter('report_every_n_scans', 10)

        lidar_topic     = self.get_parameter('lidar_topic').value
        imu_topic       = self.get_parameter('imu_topic').value
        self.exp_lidar  = self.get_parameter('expected_lidar_hz').value
        self.exp_imu    = self.get_parameter('expected_imu_hz').value
        self.warn_ms    = self.get_parameter('sync_warn_thresh_ms').value
        self.report_n   = self.get_parameter('report_every_n_scans').value

        # ── State ─────────────────────────────────────────────────────────────
        # Circular buffer of recent IMU stamps (sim time, seconds)
        self._imu_stamps: deque[float] = deque(maxlen=500)
        # Per-scan diagnostics
        self._scan_count     = 0
        self._last_scan_wall = None          # wall-clock time of last scan
        self._last_scan_sim  = None          # sim time of last scan

        # Running stats
        self._deltas_ms:     deque[float] = deque(maxlen=200)  # lidar-IMU delta
        self._jitter_ms:     deque[float] = deque(maxlen=200)  # inter-scan wall gap
        self._imu_gaps_ms:   deque[float] = deque(maxlen=500)  # inter-IMU wall gap
        self._pt_time_spans: deque[float] = deque(maxlen=200)  # per-cloud time field span
        self._imu_last_wall  = None

        # ── QoS ───────────────────────────────────────────────────────────────
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._imu_sub   = self.create_subscription(Imu,          imu_topic,   self._imu_cb,   best_effort)
        self._lidar_sub = self.create_subscription(PointCloud2,  lidar_topic, self._lidar_cb, best_effort)

        self.get_logger().info(
            f"\n{BOLD}{CYAN}IMU-LiDAR Sync Checker started{RESET}\n"
            f"  LiDAR : {lidar_topic}  (expected {self.exp_lidar} Hz)\n"
            f"  IMU   : {imu_topic}  (expected {self.exp_imu} Hz)\n"
            f"  Warn threshold : {self.warn_ms} ms\n"
            f"  Report every   : {self.report_n} scans\n"
        )

    # ── IMU callback ──────────────────────────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        now_wall = time.monotonic()
        sim_t    = stamp_to_sec(msg.header.stamp)
        self._imu_stamps.append(sim_t)

        if self._imu_last_wall is not None:
            gap = sec_to_ms(now_wall - self._imu_last_wall)
            self._imu_gaps_ms.append(gap)
        self._imu_last_wall = now_wall

    # ── LiDAR callback ────────────────────────────────────────────────────────
    def _lidar_cb(self, msg: PointCloud2):
        now_wall   = time.monotonic()
        scan_sim_t = stamp_to_sec(msg.header.stamp)
        self._scan_count += 1

        # ── 1. Nearest-IMU delta ───────────────────────────────────────────
        nearest_delta_ms = None
        if self._imu_stamps:
            nearest_imu = min(self._imu_stamps, key=lambda t: abs(t - scan_sim_t))
            nearest_delta_ms = sec_to_ms(scan_sim_t - nearest_imu)
            self._deltas_ms.append(nearest_delta_ms)

        # ── 2. LiDAR arrival jitter ───────────────────────────────────────
        jitter_ms = None
        if self._last_scan_wall is not None:
            jitter_ms = sec_to_ms(now_wall - self._last_scan_wall)
            self._jitter_ms.append(jitter_ms)
        self._last_scan_wall = now_wall

        # ── 3. Sim time monotonicity check ───────────────────────────────
        sim_jump_ms = None
        if self._last_scan_sim is not None:
            sim_jump_ms = sec_to_ms(scan_sim_t - self._last_scan_sim)
        self._last_scan_sim = scan_sim_t

        # ── 4. Per-point 'time' field inspection ─────────────────────────
        pt_time_min, pt_time_max, pt_time_span = self._inspect_point_times(msg)

        if pt_time_span is not None:
            self._pt_time_spans.append(pt_time_span * 1e3)  # store in ms

        # ── 5. Clock source heuristic ────────────────────────────────────
        wall_now_approx = time.time()
        clock_suspicious = abs(scan_sim_t - wall_now_approx) < 2.0  # within 2 s of wall

        # ── Per-scan one-liner ────────────────────────────────────────────
        self._print_scan_line(
            scan_sim_t, nearest_delta_ms, jitter_ms,
            sim_jump_ms, pt_time_min, pt_time_max, pt_time_span,
            clock_suspicious
        )

        # ── Periodic summary ──────────────────────────────────────────────
        if self._scan_count % self.report_n == 0:
            self._print_summary()

    # ── Parse the 'time' field from a PointCloud2 ─────────────────────────────
    def _inspect_point_times(self, msg: PointCloud2):
        """Returns (min_t, max_t, span) in seconds, or (None,None,None) if field absent."""
        time_field = None
        for f in msg.fields:
            if f.name == 'time':
                time_field = f
                break
        if time_field is None:
            return None, None, None

        fmt_map = {
            PointField.FLOAT32: 'f',
            PointField.FLOAT64: 'd',
        }
        fmt = fmt_map.get(time_field.datatype)
        if fmt is None:
            return None, None, None

        offset  = time_field.offset
        step    = msg.point_step
        n_pts   = msg.width * msg.height
        data    = msg.data

        t_min =  math.inf
        t_max = -math.inf
        count = 0
        for i in range(n_pts):
            base = i * step + offset
            try:
                t = struct.unpack_from(fmt, data, base)[0]
            except struct.error:
                continue
            if math.isfinite(t):
                t_min = min(t_min, t)
                t_max = max(t_max, t)
                count += 1

        if count == 0:
            return None, None, None
        return t_min, t_max, t_max - t_min

    # ── One-liner per scan ────────────────────────────────────────────────────
    def _print_scan_line(
        self, sim_t, delta_ms, jitter_ms,
        sim_jump_ms, pt_tmin, pt_tmax, pt_span,
        clock_suspicious
    ):
        delta_str = f"{delta_ms:+8.2f} ms" if delta_ms is not None else "   N/A   "
        delta_col = GREEN
        if delta_ms is not None:
            if abs(delta_ms) > self.warn_ms:
                delta_col = RED
            elif abs(delta_ms) > self.warn_ms * 0.5:
                delta_col = YELLOW

        jitter_str = f"{jitter_ms:7.1f} ms" if jitter_ms is not None else "  N/A   "
        jitter_col = GREEN
        expected_period_ms = 1000.0 / self.exp_lidar
        if jitter_ms is not None:
            if abs(jitter_ms - expected_period_ms) > expected_period_ms * 0.3:
                jitter_col = YELLOW
            if abs(jitter_ms - expected_period_ms) > expected_period_ms * 0.6:
                jitter_col = RED

        span_str = f"{pt_span*1e3:6.2f} ms" if pt_span is not None else "  N/A "
        tmin_str = f"{pt_tmin*1e3:6.2f}" if pt_tmin is not None else " N/A "
        tmax_str = f"{pt_tmax*1e3:6.2f}" if pt_tmax is not None else " N/A "

        clock_warn = f" {RED}[WALL CLK?]{RESET}" if clock_suspicious else ""

        msg_str = (
            f"[{self._scan_count:5d}] "
            f"sim={sim_t:12.4f}s | "
            f"Δ(lidar-IMU)={delta_col}{delta_str}{RESET} | "
            f"arrival_gap={jitter_col}{jitter_str}{RESET} | "
            f"pt.time [{tmin_str}..{tmax_str}] span={span_str}"
            f"{clock_warn}"
        )

        if delta_ms is not None and abs(delta_ms) > self.warn_ms:
            msg_str += f"  {RED}← SYNC GAP!{RESET}"

        if sim_jump_ms is not None and sim_jump_ms < 0:
            msg_str += f"  {RED}← SIM TIME WENT BACKWARD!{RESET}"

        self.get_logger().info(msg_str)

    # ── Periodic summary ──────────────────────────────────────────────────────
    def _print_summary(self):
        def stats(buf):
            if not buf:
                return "N/A"
            lst = list(buf)
            mean = sum(lst) / len(lst)
            variance = sum((x - mean) ** 2 for x in lst) / len(lst)
            std  = math.sqrt(variance)
            return f"mean={mean:+7.2f}  std={std:6.2f}  min={min(lst):+7.2f}  max={max(lst):+7.2f}"

        # IMU actual rate
        imu_hz_str = "N/A"
        if len(self._imu_gaps_ms) > 5:
            avg_gap = sum(self._imu_gaps_ms) / len(self._imu_gaps_ms)
            if avg_gap > 0:
                imu_hz = 1000.0 / avg_gap
                imu_col = GREEN if abs(imu_hz - self.exp_imu) / self.exp_imu < 0.1 else RED
                imu_hz_str = f"{imu_col}{imu_hz:.1f} Hz{RESET} (expected {self.exp_imu:.0f} Hz)"

        # LiDAR actual rate
        lidar_hz_str = "N/A"
        if len(self._jitter_ms) > 5:
            avg_gap = sum(self._jitter_ms) / len(self._jitter_ms)
            if avg_gap > 0:
                lidar_hz = 1000.0 / avg_gap
                lidar_col = GREEN if abs(lidar_hz - self.exp_lidar) / self.exp_lidar < 0.1 else RED
                lidar_hz_str = f"{lidar_col}{lidar_hz:.1f} Hz{RESET} (expected {self.exp_lidar:.0f} Hz)"

        # Expected pt.time span for 600 RPM / 10 Hz = 0.1 s = 100 ms
        expected_span_ms = 1000.0 / self.exp_lidar
        span_ok = True
        if self._pt_time_spans:
            avg_span = sum(self._pt_time_spans) / len(self._pt_time_spans)
            if abs(avg_span - expected_span_ms) > expected_span_ms * 0.1:
                span_ok = False

        sep = f"{CYAN}{'─'*80}{RESET}"
        lines = [
            sep,
            f"{BOLD}{CYAN}  ── SYNC SUMMARY (last {self.report_n} scans, scan #{self._scan_count}) ──{RESET}",
            sep,
            f"  IMU actual rate  : {imu_hz_str}",
            f"  LiDAR actual rate: {lidar_hz_str}",
            "",
            f"  Δ(LiDAR hdr − nearest IMU) [ms]:  {stats(self._deltas_ms)}",
            f"  LiDAR wall-arrival gap [ms]:       {stats(self._jitter_ms)}",
            f"  Per-point time span [ms]:          {stats(self._pt_time_spans)}",
            f"  Expected pt.time span              {expected_span_ms:.1f} ms  → {'OK' if span_ok else RED+'MISMATCH — check rpm/h_samples'+RESET}",
        ]

        # Diagnosis hints
        lines.append("")
        lines.append(f"{BOLD}  Diagnosis hints:{RESET}")

        if self._deltas_ms:
            mean_d = sum(self._deltas_ms) / len(self._deltas_ms)
            max_d  = max(abs(x) for x in self._deltas_ms)
            if max_d > self.warn_ms:
                lines.append(
                    f"  {RED}✗ IMU-LiDAR timestamp offset is large ({max_d:.1f} ms).{RESET}\n"
                    f"    → Try setting  time_lag_imu_to_lidar: {mean_d/1000:.6f}  in velody16.yaml\n"
                    f"    → Or check that use_sim_time=true is set on BOTH nodes.\n"
                    f"    → Verify /clock is bridged (lidar_bridge.yaml has /clock entry)."
                )
            else:
                lines.append(f"  {GREEN}✓ IMU-LiDAR timestamp delta looks OK (<{self.warn_ms:.0f} ms){RESET}")

        if not span_ok and self._pt_time_spans:
            avg_span = sum(self._pt_time_spans) / len(self._pt_time_spans)
            lines.append(
                f"  {RED}✗ Per-point time span ({avg_span:.1f} ms) ≠ expected ({expected_span_ms:.1f} ms).{RESET}\n"
                f"    → Check rpm ({60.0/self.exp_lidar:.0f} s/rev) and horizontal_samples in lidar_time_injector.\n"
                f"    → Also check timestamp_unit: 0 (seconds) in velody16.yaml."
            )
        else:
            lines.append(f"  {GREEN}✓ Per-point time span looks reasonable{RESET}")

        if len(self._imu_stamps) < 5:
            lines.append(
                f"  {RED}✗ Very few IMU messages received ({len(self._imu_stamps)}).{RESET}\n"
                f"    → Check /imu bridge entry in lidar_bridge.yaml.\n"
                f"    → Check imu_corrector_node is running."
            )

        lines.append(sep)
        self.get_logger().info('\n' + '\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = ImuLidarSyncChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()