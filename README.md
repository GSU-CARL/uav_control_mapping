# UAV Control and Mapping

Student: Saroeun Norakvitou (Intern) · Direct Mentor: Yong Ann Voeurn (PhD
Candidate) · Supervisor: Prof. Doyun Lee — Georgia Southern University
ROS Distribution: ROS 2 Jazzy | Simulator: Ignition/Gazebo Harmonic (`gz sim` 8.11)

## Overview

Flies a PX4 SITL x500 quadrotor carrying a custom 3D lidar in Gazebo (the
`tugbot_warehouse` world), bridges the lidar/IMU into ROS 2, and feeds
[Point-LIO](../point_lio_ros2) (a sibling package in this workspace) for
real-time LiDAR-inertial odometry and mapping. Flight control goes through
MAVROS. An `octomap_server` layer can turn Point-LIO's assembled cloud into a
3D occupancy grid. The same world also has a `tugbot` ground robot with its
own keyboard teleop, useful as a second sensor platform or for bring-up
without flying.

```
PX4 SITL (x500 + 3D lidar) ── gz sim ── ros_gz_bridge ─┬─> lidar_time_injector ─> Point-LIO ─> /cloud_registered ─> octomap_server
                                                        └─> /drone/imu2 ─────────────┘                                  │
                                                                                                                         v
                                              MAVROS ── manual_control_node / simple_nav_node        occupied-cells grid / RViz
```

This README covers the current state of the package. Day-to-day working
notes, cross-file invariants, and the "don't touch these files without
asking" rules live in `CLAUDE.md` — read that first if you're changing
anything here.

## Package layout

- `launch/` — see [Launch files](#launch-files) below.
- `uav_control_mapping/` — the ROS 2 Python nodes (entry points below).
- `config/lidar_bridge2.yaml` — `ros_gz_bridge` topic map: drone lidar/IMU
  and tugbot lidar/IMU, gz → ROS.
- `models/` — locally-tracked drone models (`x500_lidar_3d_local`, `x500`,
  `x500_base`, `lidar_3d_local`, `px4vision`) used by
  `launch/px4_launch.launch.py`'s self-contained spawn path.
- `world/warehouse.sdf` — the active world file (internal `<world name>` is
  still `tugbot_warehouse`, which is why topic/service names below keep that
  name). `world/tugbot_depot.sdf` is an older, currently-unused world.
- `external_files/` — snapshot of everything required *outside* this repo
  (PX4 airframe/model files, the local `point_lio_ros2` diffs). See its own
  README for restore instructions.
- `planning_ai/` — scratch design notes from earlier RTAB-Map-based planning;
  largely superseded by the Point-LIO pipeline described here.

## Launch files

There are two independent ways to get PX4 + Gazebo + the drone flying, plus
one to add full mapping and one to add an occupancy grid on top.

- **`px4_launch.launch.py`** — preferred, self-contained. Starts `gz sim`
  itself, spawns this package's own `models/x500_lidar_3d_local/` by file
  path via the world's `create` service, then starts PX4 with
  `PX4_GZ_MODEL_NAME` so PX4 *attaches* to the already-spawned model instead
  of spawning its own. Also starts the `ros_gz_bridge` and
  `lidar_time_injector`. Doesn't require copying anything into the PX4 tree.
- **`local_px4_file.launch.py`** — older path: plain
  `make px4_sitl gz_x500_lidar_3d`, which requires the model/airframe files
  from `external_files/` to already be copied into `~/PX4-Autopilot` (see
  [External prerequisites](#external-prerequisites)). Also starts the bridge
  and the `lidar_time_injector`.
- **`point_lio_full.launch.py`** — full mapping pipeline: includes
  `local_px4_file.launch.py`, adds the static TFs (`base_link -> lidar_3d`,
  `aft_mapped -> base_link`), and includes Point-LIO's
  `mapping_velody16.launch.py`. Do **not** add a second
  `lidar_time_injector` alongside this one — it double-publishes every scan
  and causes LIO drift.
- **`octomap.launch.py`** — run *after* `point_lio_full.launch.py` is
  already up. Subscribes to Point-LIO's `/cloud_registered` and publishes a
  3D occupancy grid (`/occupied_cells_vis_array`, `/octomap_binary`, ...) in
  a `map` frame that this launch file aliases (identity TF) to Point-LIO's
  `camera_init` frame.

MAVROS itself is **not** started by any launch file here — run it
separately, pointed at the PX4 SITL MAVLink port (SITL's default is
`udp://:14540@localhost:14557`).

## External prerequisites

`local_px4_file.launch.py` / `point_lio_full.launch.py` need PX4-side files
that don't live in this repo: the custom `4022_gz_x500_lidar_3d` airframe,
failsafe-disable edits to the stock `4001_gz_x500` airframe (SITL has no
RC/GCS, so datalink/RC-loss failsafes must be off), and the `lidar_3d` /
`x500_lidar_3d` model + `tugbot_warehouse.sdf` world files. All of it is
snapshotted with restore instructions in `external_files/README.md` — copy
`external_files/{ROMFS,Tools}` into `~/PX4-Autopilot` and rebuild SITL.

`point_lio_full.launch.py` also needs the vendored `../point_lio_ros2`
package built (`colcon build --symlink-install --packages-select point_lio
--cmake-args -DCMAKE_BUILD_TYPE=Release`); the local patches it needs on top
of upstream are documented in the same `external_files/README.md`.

`px4_launch.launch.py` avoids the PX4-tree copy step entirely, but still
needs `~/PX4-Autopilot` built (`make px4_sitl`) since it drives PX4's own gz
plugins and server config.

## Dependencies

- ROS 2 Jazzy
- PX4-Autopilot (SITL, in `~/PX4-Autopilot`)
- Point-LIO (vendored sibling package, `../point_lio_ros2`)
- Ignition/Gazebo Harmonic (`gz sim` 8.11)
- MAVROS (`ros-jazzy-mavros`, `ros-jazzy-mavros-extras`)
- `ros_gz_bridge`
- `octomap_server` (`ros-jazzy-octomap-server`) — only needed for
  `octomap.launch.py`
- `pynput` (keyboard teleop for `manual_control` / `ground_manual_control`)

## Build & test

Always from the workspace root, then source the overlay:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select uav_control_mapping
source install/setup.bash
```

`--symlink-install` matters: Python node edits take effect without
rebuilding, but adding/renaming entry points, launch files, models, or
config files requires a rebuild (`setup.py`'s `data_files` are re-globbed at
build time).

Tests are lint-only (`ament_flake8`, `ament_pep257`, `ament_copyright`):

```bash
colcon test --packages-select uav_control_mapping && colcon test-result --verbose
```

## Usage

1. Build and source as above.
2. Bring up PX4 + Gazebo + bridge (pick one):
   ```bash
   ros2 launch uav_control_mapping px4_launch.launch.py       # self-contained, preferred
   # or, if the PX4-tree files from external_files/ are in place:
   ros2 launch uav_control_mapping point_lio_full.launch.py   # PX4 + bridge + Point-LIO mapping
   ```
3. Start MAVROS separately (not launched by this package), pointed at PX4
   SITL's MAVLink port.
4. Optional — 3D occupancy grid, once Point-LIO is running:
   ```bash
   ros2 launch uav_control_mapping octomap.launch.py
   ```
5. Fly manually (new terminal, after MAVROS is up):
   ```bash
   ros2 run uav_control_mapping manual_control_node
   ```
   or send `PoseStamped` goals to `/goal_pose` for `simple_nav_node` to fly
   to (fixed altitude 3.0 m).
6. Ground robot teleop (no MAVROS/offboard handshake — tugbot is driven
   directly by gz's DiffDrive plugin):
   ```bash
   ros2 run uav_control_mapping ground_manual_control_node
   ```
   **Note:** this currently publishes `Twist` on `/tugbot/cmd_vel`, but that
   topic isn't in `config/lidar_bridge2.yaml` yet — add a
   `/tugbot/cmd_vel -> /model/tugbot/cmd_vel` bridge entry before it will
   actually move the robot in sim.

## Nodes (entry points)

All drone control nodes derive from `DroneBaseNode`
(`uav_control_mapping/drone_base.py`), which wraps the MAVROS
state/arm/OFFBOARD handshake (`/mavros/state`,
`/mavros/setpoint_position/local`, `/mavros/cmd/arming`, `/mavros/set_mode`).

| Entry point | Source | What it does |
|---|---|---|
| `manual_control_node` | `manual_control.py` | Keyboard teleop for the drone via MAVROS offboard setpoints. |
| `simple_nav_node` | `simple_nav.py` | Flies to `PoseStamped` goals published on `/goal_pose` (fixed z = 3.0 m). |
| `ground_manual_control_node` | `ground_manual_control.py` | Keyboard teleop for the tugbot ground robot, publishes `Twist` directly (no MAVROS). |
| `lidar_time_injector` | `lidar_time_injection.py` | Rewrites the gz point cloud into the FAST-LIO/Point-LIO layout (x,y,z,intensity,ring,time) the drone's lidar needs. |
| `ground_lidar_time_injector` | `ground_lidar_time_injection.py` | Same idea, for the tugbot's lidar. |
| `imu_lidar_sync_checker` | `imu_lidar_sync_checker.py` | Diagnostic: prints IMU/lidar sync, jitter, and per-point-time sanity stats. |

## Keyboard controls

`manual_control_node` (drone, once MAVROS is connected):

- **W / S** — forward / backward
- **A / D** — left / right
- **SPACE / CTRL** — up / down
- **Q / E** — yaw left / yaw right
- **L** — land
- **ESC** — stop immediately and land

`ground_manual_control_node` (tugbot):

- **I / K** — forward / backward
- **J / L** — turn left / right
- **ESC** — stop

## Notes on the current sim setup

These are one-time edits made in the PX4 tree; see `external_files/README.md`
for the exact diffs and how to reapply them on a fresh PX4 checkout.

- **RGB/depth aspect ratio** — no longer relevant to the current lidar-based
  pipeline; this only mattered for the earlier RTAB-Map/OakD-Lite setup.
- **Disabling RC/GCS failsafes** — SITL has no RC or ground control station
  connected, so `NAV_DLL_ACT`, `NAV_RCL_ACT`, and `COM_RCL_EXCEPT` are set to
  0/0/4 in the airframe so PX4 doesn't trigger a failsafe land/RTL.
- **Custom `x500_lidar_3d` airframe/model** — adds the 3D lidar to the x500
  frame; registered as PX4 airframe `4022_gz_x500_lidar_3d`.
