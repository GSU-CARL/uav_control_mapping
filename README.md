# UAV Control and Mapping

**Student:** Saroeun Norakvitou (Intern) · **Direct Mentor:** Yong Ann Voeurn (PhD Candidate) · **Supervisor:** Prof. Doyun Lee — Georgia Southern University  
**ROS Distribution:** ROS 2 Jazzy | **Simulator:** Gazebo Harmonic (`gz sim` 8.11) | **Autopilot:** PX4 SITL

---

## Overview

This package flies a PX4 SITL `x500` quadrotor equipped with a 3D LiDAR in Gazebo (`tugbot_warehouse`), bridges LiDAR/IMU data into ROS 2, runs real-time 3D SLAM with **Point-LIO**, builds a 3D occupancy map with **OctoMap**, and provides both manual teleop and autonomous 3D trajectory planning (`astar_nav_node` and `mighty_nav_node`) via **MAVROS**.

```text
PX4 SITL (x500 + 3D LiDAR) ──> ros_gz_bridge ──> lidar_time_injector ──> Point-LIO (/cloud_registered)
                                                                                  │
                                      ┌───────────────────────────────────────────┴───────────────────────┐
                                      ▼                                                                   ▼
                               octomap_server                                           MAVROS Autonomous / Manual Control
                         (/octomap_point_cloud_centers)                            (mighty_nav_node / astar_nav_node / manual_control_node)
```

---

## Demos

### MIGHTY Trajectory Planner (`mighty_nav_node`)
![MIGHTY Planner Demo](img/mighty_demo.gif)  
*Figure 1: Autonomous 3D warehouse navigation using the MIGHTY planner (Heat-Guided Global Planner + Quintic Hermite Spline L-BFGS trajectory optimization).*

### 3D A* Path Planner (`astar_nav_node`)
![3D A* Planner Demo](img/a_star_demo.gif)  
*Figure 2: Autonomous 3D warehouse navigation using the 3D A* path planner with obstacle inflation and line-of-sight smoothing.*

---

## 1. Setup (Choose Docker or Native)

### Option A: Docker Setup (Recommended for Reproducibility)

The repository includes a [`Dockerfile`](Dockerfile) and [`docker-compose.yml`](docker-compose.yml) that automatically install ROS 2 Jazzy, Gazebo Harmonic, MAVROS, OctoMap, PX4 SITL (`e17d81af21`), and Point-LIO (`dfloreaa/point_lio_ros2`), apply all patches from [`external_files/`](external_files/README.md), and build the workspace.

1. **Build and start the container:**
   ```bash
   cd ~/ros2_ws/src/uav_control_mapping
   xhost +local:docker
   docker compose up -d --build
   ```
   *(If using an NVIDIA GPU with `nvidia-container-toolkit`, uncomment the `deploy:` block in [`docker-compose.yml`](docker-compose.yml) before starting.)*

2. **Open shells inside the container:**
   For each terminal in [Section 2 (Quick Start)](#2-quick-start-step-by-step), attach to the running container first:
   ```bash
   docker exec -it uav_control_mapping bash
   ```
   *(ROS 2 Jazzy and `/home/fishman/ros2_ws/install/setup.bash` are automatically sourced inside the container.)*

3. **Stop the container when finished:**
   ```bash
   docker compose down
   ```

---

### Option B: Native Ubuntu 24.04 Setup

- **Dependencies:**
  - **ROS 2 Jazzy**, **Gazebo Harmonic** (`gz sim` 8.11), `ros-jazzy-ros-gz-bridge`
  - **PX4-Autopilot** (`~/PX4-Autopilot`)
  - **Point-LIO** (`~/ros2_ws/src/point_lio_ros2` from `https://github.com/dfloreaa/point_lio_ros2.git`)
  - **MAVROS** (`ros-jazzy-mavros`, `ros-jazzy-mavros-extras`)
  - **OctoMap** (`ros-jazzy-octomap-server`, `ros-jazzy-octomap-rviz-plugins`)
  - **Python packages:** `numpy`, `scipy`, `pynput`
- **One-time PX4 & Point-LIO patch setup:** Follow [`external_files/README.md`](external_files/README.md) to copy the custom `x500_lidar_3d` airframe/model files into `~/PX4-Autopilot` and apply the Point-LIO config patches.
- **Build the workspace:**
  ```bash
  cd ~/ros2_ws
  colcon build --symlink-install --packages-select point_lio uav_control_mapping --cmake-args -DCMAKE_BUILD_TYPE=Release
  source install/setup.bash
  ```

---

## 2. Quick Start (Step-by-Step)

Open 4 terminals (either native terminals with `source ~/ros2_ws/install/setup.bash`, or container shells via `docker exec -it uav_control_mapping bash`).

### Terminal 1 — Launch Gazebo + PX4 + Point-LIO Mapping
Starts Gazebo, spawns the drone, bridges LiDAR/IMU, and launches Point-LIO + RViz:

```bash
ros2 launch uav_control_mapping point_lio_full.launch.py
```
*(Alternative: `ros2 launch uav_control_mapping px4_launch.launch.py` launches PX4 + Gazebo + bridge without Point-LIO.)*

### Terminal 2 — Start MAVROS
Connects ROS 2 to PX4 SITL's MAVLink port:

```bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557
```

### Terminal 3 — Start OctoMap (Optional, for 3D Voxel Mapping)
Converts Point-LIO's `/cloud_registered` into a 3D occupancy grid:

```bash
ros2 launch uav_control_mapping octomap.launch.py
```

### Terminal 4 — Run a Control or Navigation Node (Pick One)

- **Option A: MIGHTY Autonomous Planner (HGP + Quintic Hermite Spline L-BFGS)**
  ```bash
  ros2 run uav_control_mapping mighty_nav_node
  ```
- **Option B: 3D A* Autonomous Planner**
  ```bash
  ros2 run uav_control_mapping astar_nav_node
  ```
- **Option C: Manual Keyboard Teleop**
  ```bash
  ros2 run uav_control_mapping manual_control_node
  ```

---

## 3. Sending 3D Goals in RViz (`mighty_nav_node` / `astar_nav_node`)

1. Wait until the node logs `Drone airborne! Ready for 3D RViz /clicked_point goals.`
2. In the **RViz** toolbar at the top, click **Publish Point**.
3. Click any point on the 3D point cloud or floor in RViz (publishes to `/clicked_point`).
4. To visualize paths in RViz, add **Path** displays for:
   - `/planned_path` — Active smoothed trajectory
   - `/hgp_global_path` — Global heat-guided path (when running `mighty_nav_node`)

---

## 4. Keyboard Controls (`manual_control_node`)

| Key | Action |
| :--- | :--- |
| **W / S** | Forward / Backward |
| **A / D** | Left / Right |
| **SPACE / CTRL** | Up / Down |
| **Q / E** | Yaw Left / Yaw Right |
| **L** | Land |
| **ESC** | Emergency Stop & Land |

---

## 5. Package Reference

### Launch Files (`launch/`)

| Launch File | Description |
| :--- | :--- |
| `point_lio_full.launch.py` | Full stack: PX4 SITL + Gazebo + `ros_gz_bridge` + `lidar_time_injector` + static TFs + Point-LIO + RViz. |
| `px4_launch.launch.py` | Self-contained PX4 SITL + Gazebo + `ros_gz_bridge` + `lidar_time_injector` using local `models/`. |
| `local_px4_file.launch.py` | Launches `make px4_sitl gz_x500_lidar_3d` + `ros_gz_bridge` + `lidar_time_injector`. |
| `octomap.launch.py` | Starts `octomap_server` on `/cloud_registered` and publishes `/occupied_cells_vis_array` and `/octomap_point_cloud_centers`. |

### Nodes (`uav_control_mapping/`)

| Node Entry Point | Source File | Description |
| :--- | :--- | :--- |
| `mighty_nav_node` | `mighty_nav.py` | MIGHTY 3D planner: Heat-Guided Global Planner (`astar_heat` + LoS shortcutting) + Quintic Hermite Spline L-BFGS trajectory optimizer. |
| `astar_nav_node` | `astar_nav.py` | 3D A* path planner with obstacle inflation, line-of-sight smoothing, and pure-pursuit velocity tracking. |
| `manual_control_node` | `manual_control.py` | Keyboard teleop for the quadrotor via MAVROS offboard setpoints. |
| `lidar_time_injector` | `lidar_time_injection.py` | Injects per-point `ring` and `time` fields into Gazebo's 3D LiDAR cloud for Point-LIO. |
| `imu_lidar_sync_checker` | `imu_lidar_sync_checker.py` | Diagnostic node that checks IMU/LiDAR timestamp synchronization and jitter. |
