"""octomap_server on top of Point-LIO's assembled point cloud.

Run alongside point_lio_full.launch.py (this file does not start Point-LIO,
PX4, or the bridge itself -- it only builds a 3D occupancy grid from
/cloud_registered once that pipeline is already running).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Point-LIO's global frame is "camera_init" (odom_header_frame_id in
        # point_lio_ros2/config/velody16.yaml, default unset -> "camera_init";
        # see point_lio_ros2/rviz_cfg/loam_livox.rviz's Fixed Frame) -- there
        # is no "map" frame published anywhere else in this pipeline. Alias
        # it to "map" here so octomap_server's frame_id below resolves and so
        # future consumers (RViz, an OMPL planner) can use the conventional
        # "map" name without touching point_lio_ros2's own config.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_init',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,

                # Coordinate frames
                'frame_id': 'map',
                'base_frame_id': 'base_link',

                # Voxel size
                'resolution': 0.15,

                # CRITICAL: negative -- NOT 0.0 -- disables the max-range
                # check. octomap_server's insertScan() only marks a point
                # occupied when `max_range_ < 0.0 || range <= max_range_`
                # (ros-jazzy-octomap-server 2.3.1, octomap_server.cpp:538);
                # at exactly 0.0 that's false for every point with range > 0,
                # so *every* point gets collapsed onto the sensor origin as a
                # zero-length free ray and nothing is ever marked occupied --
                # silently empty map. /cloud_registered is Point-LIO's
                # already-assembled cloud in the global frame, and
                # octomap_server takes the message's own frame origin as the
                # sensor position for ray casting, so any finite max_range
                # here would also wrongly discard real points far from the
                # world origin -- keep it unlimited.
                # Also note the param is dotted ("sensor_model.max_range"),
                # not slash-separated -- a slash creates an unused parameter
                # that never reaches the node's actual max_range_ field.
                'sensor_model.max_range': -1.0,

                # Height limits (wide so nothing is clipped). Real param
                # names have an underscore between "point" and "cloud"
                # ("point_cloud_min_z/max_z") -- "pointcloud_..." is silently
                # ignored and the (already wide, -100/100) built-in default
                # applies instead.
                'point_cloud_min_z': -50.0,
                'point_cloud_max_z': 50.0,
                'occupancy_min_z': -50.0,
                'occupancy_max_z': 50.0,

                # No saved map to load; octomap_server logs one benign
                # "Could not open file" warning at startup either way since
                # this is also its built-in default.
                'octomap_path': '',

                'filter_ground_plane': False,
                'compress_map': True,
            }],
            remappings=[
                # Point-LIO (laserMapping.cpp publish_frame_world) publishes
                # each downsampled scan, already transformed into the global
                # frame, on /cloud_registered.
                ('cloud_in', '/cloud_registered'),
            ],
        ),
    ])
