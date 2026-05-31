## Prompt

### this is my folder tree.
```
.
├── config
│   └── depth_cam_bridge.yaml
├── launch
│   ├── octomap.launch.py
│   ├── px4_launch.launch.py
│   ├── rtabmap_px4.launch.py
│   └── simulation.launch.py
├── map
├── model
│   ├── lidar_2d_v2
│   │   ├── meshes
│   │   │   ├── lidar_2d_v2.dae
│   │   │   └── lidar_2d_v2.jpg
│   │   ├── model.config
│   │   └── model.sdf
│   ├── lidar_3d
│   │   ├── meshes
│   │   │   └── lidar_2d_v2.dae
│   │   ├── model.config
│   │   └── model.sdf
│   ├── OakD-Lite
│   │   ├── LICENSE
│   │   ├── meshes
│   │   │   └── OakDLite.dae
│   │   ├── model.config
│   │   └── model.sdf
│   ├── r1_rover
│   │   ├── meshes
│   │   │   ├── antenna_link.STL
│   │   │   ├── battery_link.STL
│   │   │   ├── chassis_link.STL
│   │   │   ├── housing_link.STL
│   │   │   ├── top_link.STL
│   │   │   ├── wheel_link_simple.STL
│   │   │   └── wheel_link.STL
│   │   ├── model.config
│   │   └── model.sdf
│   ├── x500
│   │   ├── LICENSE
│   │   ├── model.config
│   │   └── model.sdf
│   ├── x500_base
│   │   ├── LICENSE
│   │   ├── materials
│   │   │   └── textures
│   │   │       ├── CF.png
│   │   │       ├── nxp.png
│   │   │       └── rd.png
│   │   ├── meshes
│   │   │   ├── 1345_prop_ccw.stl
│   │   │   ├── 1345_prop_cw.stl
│   │   │   ├── 5010Base.dae
│   │   │   ├── 5010Bell.dae
│   │   │   ├── CF.png
│   │   │   └── NXP-HGD-CF.dae
│   │   ├── model.config
│   │   └── model.sdf
│   ├── x500_depth
│   │   ├── LICENSE
│   │   ├── model.config
│   │   └── model.sdf
│   └── x500_lidar_2d
│       ├── model.config
│       └── model.sdf
├── package.xml
├── planning_ai
│   ├── manual_control_node_plan.md
│   ├── octomap_drone_navigation.md
│   ├── rtabmap_octomap_plan.md
│   ├── rtabmap_optimization.md
│   └── rtabmap_px4_mapping_plan.md
├── README.md
├── resource
│   └── uav_control_mapping
├── setup.cfg
├── setup.py
├── test
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── uav_control_mapping
│   ├── drone_base.py
│   ├── __init__.py
│   ├── manual_control.py
│   ├── __pycache__
│   │   └── trajectory_follower.cpython-310.pyc
│   ├── simple_nav.py
│   ├── trajectory_follower.py
│   └── waypoint_manager.py
└── world
    └── tugbot_depot.sdf
```

### Task:
1. create another px4_launch but for 3D lidar drone.
2. Create another x500_lidar_2d folder but for 3d lidar "x500_lidar_3d" (use the reference code from x500_lidar_2d)
