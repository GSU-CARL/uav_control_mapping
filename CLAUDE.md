# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

This is the `uav_control_mapping` package (its own git repo) inside the
`~/ros2_ws` colcon workspace. Workspace-level guidance (vendored SLAM stacks,
external toolchain, PX4 paths) lives in `../CLAUDE.md` — read that too. Rules
that carry over: **never push to git**, never edit `build/`/`install/`/`log/`.

## DO NOT DO!!

**Always ask before making any change outside this directory**
(`uav_control_mapping/`) — e.g. `~/PX4-Autopilot`, `../point_lio_ros2`,
`../lidarslam_ros2`, or any other sibling package/repo in `~/ros2_ws/src`.
Confirm with the user first, even for small edits.

If such an external change is made (with the user's go-ahead), document it in
`EXTERNAL_CHANGES.md` in this directory (create it if it doesn't exist yet) —
what file changed, where, and why — in addition to updating the
`external_files/` snapshot described below. Then update this CLAUDE.md's
`external_files/` section if the change affects what's tracked there.

## Build & test

Always from the workspace root, then source the overlay:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select uav_control_mapping
source install/setup.bash
```

`--symlink-install` matters: Python node edits take effect without rebuilding,
but adding/renaming entry points, launch files, models, or config files
requires a rebuild (setup.py `data_files` are re-globbed at build time).

Tests are lint-only (`ament_flake8`, `ament_pep257`, `ament_copyright`):

```bash
colcon test --packages-select uav_control_mapping && colcon test-result --verbose
```

## What this package does

Flies a PX4 SITL x500 quad with a custom 3D lidar in Ignition Gazebo
(tugbot_warehouse world), bridges lidar/IMU into ROS 2, and feeds Point-LIO
(vendored in `../point_lio_ros2`) for LiDAR-inertial mapping. Control goes
through MAVROS.

### Launch files — two different model-spawn paths

- `launch/px4_launch.launch.py` — **preferred, self-contained**: starts
  `gz sim` itself, spawns this package's own `models/x500_lidar_3d_local/`
  by file path via the world's `create` service, then starts PX4 with
  `PX4_GZ_MODEL_NAME` so PX4 *attaches* to the already-spawned model instead
  of spawning its own. Sequencing is deliberate: PX4/bridge/injector only
  start after the spawn command exits (a race here left every sensor topic
  unbridged). It also sets `GZ_SIM_SYSTEM_PLUGIN_PATH` (PX4's gz plugins) and
  `GZ_SIM_SERVER_CONFIG_PATH` (without it IMU/baro/mag/GPS never publish and
  EKF2 reports all sensors missing).
- `launch/local_px4_file.launch.py` — older path: plain
  `make px4_sitl gz_x500_lidar_3d`, which requires the model/airframe files to
  be copied into the PX4 tree (see `external_files/`). Also starts the bridge
  and the lidar_time_injector.
- `launch/point_lio_full.launch.py` — full mapping pipeline: includes
  `local_px4_file.launch.py` + static TFs + Point-LIO's
  `mapping_velody16.launch.py`. **Do not add a second `lidar_time_injector`
  here** — one already runs inside the included launch file; a duplicate
  double-publishes every scan and caused a hard-to-diagnose LIO drift bug.

### Nodes

All control nodes derive from `DroneBaseNode` (`uav_control_mapping/drone_base.py`),
which wraps the MAVROS state/arm/OFFBOARD handshake. Entry points (setup.py):
`manual_control_node`, `lidar_time_injector`,
`imu_lidar_sync_checker` (diagnostic; prints sync/jitter/per-point-time stats).

`lidar_time_injector` (`lidar_time_injection.py`) rewrites the gz point cloud
into the FAST-LIO/Point-LIO layout — x,y,z,intensity (float32), ring (uint16 +
2-byte pad), time (float32, **seconds**, so `timestamp_unit: 0` in
velody16.yaml), point_step 24. `flatten_time` defaults to true because gz
`gpu_lidar` captures the whole sweep instantaneously; azimuth-based fake
per-point times make Point-LIO "undistort" motion that never happened.

## Cross-file invariants (keep in sync when touching any one of them)

- **Topic contract**: bridge output (`config/lidar_bridge2.yaml`:
  `/drone/lidar_3d/points`, `/drone/imu2`) → injector defaults (in:
  `/drone/lidar_3d/points`, out: `/drone/lidar_3d/points_timestamped`) →
  Point-LIO's `velody16.yaml` (`lid_topic`, `imu_topic`). The gz-side topic
  names in the bridge yaml embed the model instance name
  (`x500_lidar_3d_local_0`) and world name (`tugbot_warehouse`).
- **Lidar mount pose** `(0, 0, 0.3035)` appears in three places: the model SDF
  (LidarJoint 0.26 + sensor pose 0.0435), the static TF in
  `point_lio_full.launch.py`, and `extrinsic_T` in
  `point_lio_ros2/config/velody16.yaml`. The lidar was deliberately centered at
  x=0 (was x=-0.1) to fix a CoG-offset hover drift — don't move it back.
- **Injector scan geometry** (rpm=600, 440×16 samples) must match the lidar
  SDF's sensor config.

## external_files/

Snapshot of everything required outside this repo: custom PX4 airframe
`4022_gz_x500_lidar_3d`, PX4 model/world files, failsafe-disable edits to
`4001_gz_x500`, and the local modifications to the vendored `point_lio_ros2`
(RViz-lag executor fix in `laserMapping.cpp`, config changes). Its README has
restore instructions and diffs (`px4-tracked-files.diff`,
`point-lio-changes.diff`). If you change any of those files in place
(`~/PX4-Autopilot` or `../point_lio_ros2`), update the snapshot here.

Note: the top-level `README.md` predates the Point-LIO work (mentions RTAB-Map
launch files and ROS Humble that no longer apply); this file and
`external_files/README.md` reflect the current state. Change history is in the
workspace-root `CHANGES*.md` files.
