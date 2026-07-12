# external_files — files outside this package required to run the simulation

Snapshot (2026-07-11) of every file that lives **outside this package** (in
`~/PX4-Autopilot` or the vendored `point_lio_ros2` repo) but is required to run
the `point_lio_full` / `x500_lidar_3d` simulation. Sources: `CHANGES.md`,
`CHANGES_2026-07-11_hover-drift.md`,
`CHANGES_2026-07-11_rviz-lag-pointlio-backlog.md`, plus `git status` of the PX4
tree to catch anything the docs missed.

Directory layout mirrors the PX4 tree, so restoring onto a fresh checkout is:

```bash
cp -r ROMFS Tools ~/PX4-Autopilot/
# then rebuild SITL so the new airframe is picked up:
cd ~/PX4-Autopilot && make px4_sitl
```

Note: `Tools/simulation/gz` in PX4 is a **git submodule** — the model/world
files below are untracked additions inside it, so a submodule update won't
delete them, but a fresh clone won't have them either.

## Custom files (don't exist in stock PX4)

| File | What it is |
|---|---|
| `ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_lidar_3d` | Custom airframe: sources `4001_gz_x500`, sets `MPC_THR_HOVER 0.60` + `MPC_USE_HTE 1` for the +0.37 kg lidar (hover-drift fix). |
| `Tools/simulation/gz/models/lidar_3d/` | Custom 3D lidar sensor model (gpu_lidar + meshes). Lidar pose centered at `0 0 0.26` (hover-drift fix — keep in sync with the static TF in `point_lio_full.launch.py` and `extrinsic_T` in `point_lio_ros2/config/velody16.yaml`). |
| `Tools/simulation/gz/models/x500_lidar_3d/` | Drone frame: x500 base + lidar_3d include. |
| `Tools/simulation/gz/worlds/tugbot_warehouse.sdf` | Warehouse world the sim flies in. |

## Modified stock PX4 files

`px4-tracked-files.diff` holds the exact diff; the full modified copies are also
included. On a fresh PX4 checkout you can either copy the files over or
`git apply px4-tracked-files.diff` from the PX4 root.

| File | Change |
|---|---|
| `ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500` | Disabled datalink/RC failsafes for SITL without GCS/RC: `NAV_DLL_ACT 0` (was 2), `NAV_RCL_ACT 0`, `COM_RCL_EXCEPT 4`; also `MPC_THR_HOVER 0.60`. |
| `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` | Registers `4022_gz_x500_lidar_3d` in the ROMFS build. |

## Modified point_lio_ros2 files (vendored repo)

`point_lio_ros2/` mirrors the vendored repo's layout;
`point_lio_ros2/point-lio-changes.diff` holds the exact diff vs upstream.
Restore onto a fresh clone with either `cp -r` of the files or
`git apply point-lio-changes.diff` from the point_lio_ros2 root, then rebuild:
`colcon build --symlink-install --packages-select point_lio --cmake-args -DCMAKE_BUILD_TYPE=Release`.

| File | Change |
|---|---|
| `point_lio_ros2/src/laserMapping.cpp` | Executor + `add_node` hoisted out of the 5000 Hz main loop (was rebuilt every iteration — RViz-lag fix, part 1). |
| `point_lio_ros2/config/velody16.yaml` | `extrinsic_T` lever arm `[-0.1,0,0.3035]` → `[0,0,0.3035]` (hover-drift fix), topic remaps + sim tuning for the gz lidar. |
| `point_lio_ros2/config/horizon.yaml` | `acc_norm` 1.0 → 9.81 (IMU reports m/s², not g). |
| `point_lio_ros2/launch/mapping_velody16.launch.py` | `use_sim_time: True`; `runtime_pos_log_enable: True`. |

(Upstream `Log/*.txt` also show as modified in git — runtime output, not
snapshotted.)

## Not included (documented elsewhere)

Everything else the change docs touch already lives in `uav_control_mapping`
itself (e.g. `lidar_time_injection.py`, `point_lio_full.launch.py`) and is
versioned there.
