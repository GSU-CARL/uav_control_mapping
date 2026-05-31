# UAV Control and Mapping

Student: Saroeun Norakvitou (Intern) Direct Mentor: Yong Ann Voeurn (PhD Candidate) Supervisor: Prof. Doyun Lee — Georgia Southern University ROS Distribution: ROS 2 Humble | Simulator: Ignition Gazebo 8.11 (Harmonic)

## Overview

This package provides nodes and launch files to control a simulated UAV, interface with a depth camera, and generate 3D maps using RTAB-Map within a simulated environment (`tugbot_depot.sdf`).

## Package Structure

- `launch/`: Contains ROS 2 launch files.
  - `px4_launch.launch.py`: Launch Gazebo Sim and PX4 SITL.
  - `rtabmap_px4.launch.py`: Launch RTAB-Map for VSLAM and 3D mapping, along with the ROS-Gazebo bridge and tf frames.
  - `simulation.launch.py`: Bring up the main simulation environment, `MicroXRCEAgent`, and `MAVROS` inside a tmux session.
- `uav_control_mapping/`: Contains ROS 2 Python nodes (e.g., `manual_control.py`).
- `config/`: Configuration files (e.g., `depth_cam_bridge.yaml`).
- `world/`: Simulation environment files (e.g., `tugbot_depot.sdf`).

## Dependencies

- ROS 2
- PX4 Autopilot
- RTAB-Map ROS 2 package (`rtabmap_ros`)
- Gazebo / Ignition
- MAVROS (`mavros`, `mavros_msgs`)
- MicroXRCEAgent (for PX4-ROS 2 communication)
- ROS-Gazebo Bridge (`ros_gz_bridge`)
- `pynput` (for manual keyboard control)
- `tmux` (for cleaner launch files)

## Usage

1. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select uav_control_mapping
   source install/setup.bash
   ```

2. Launch the simulation (starts Gazebo, PX4 SITL, MicroXRCEAgent, and MAVROS via tmux):
   ```bash
   ros2 launch uav_control_mapping simulation.launch.py
   ```

3. Launch RTAB-Map mapping:
   ```bash
   ros2 launch uav_control_mapping rtabmap_px4.launch.py
   ```

4. Run the manual control node (in a new terminal):
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run uav_control_mapping manual_control
   ```

## Keyboard Controls (`manual_control.py`)

Once the `manual_control` node is running and the FCU is connected, you can fly the UAV using the following keys:

- **W / S**: Move Forward / Backward
- **A / D**: Move Left / Right
- **SPACE / CTRL**: Move Up / Down
- **Q / E**: Yaw Left / Yaw Right
- **L**: Land
- **ESC**: Stop immediately and Land

## Rtap-map config

go to `PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf`

**make sure the rgb and depth has the same aspect ratio.**
```python
<sensor name="IMX214" type="camera">
        ............

        <width>320</width>      
        <height>240</height>    

        ............
</sensor>
<sensor name="StereoOV7251" type="depth_camera">
        ............

        <width>320</width>     
        <height>240</height>    

        ............
</sensor>
```
## Disable remote controller and GCC

got to `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500`

```python
param set-default NAV_DLL_ACT 0 # default is 2

param set NAV_RCL_ACT 0
param set COM_RCL_EXCEPT 4
```
## Add the new x500_lidar_3d model to px4_autopilot

got to `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/`

copy this file in there
`4022_gz_x500_lidar_3d`

rename the 4022 if needed

Edit the CMakeList.txt
add this line 
```
.
.
.
4019_gz_x500_gimbal
4020_gz_tiltrotor
4021_gz_x500_flow
4022_gz_x500_lidar_3d # add this

.
.
.
```