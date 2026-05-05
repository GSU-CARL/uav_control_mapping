from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                # Coordinate frames
                'frame_id': 'map',
                'base_frame_id': 'base_link',

                # Voxel size
                'resolution': 0.15,

                # CRITICAL: must be 0.0 (infinite) when using RTAB-Map's assembled /cloud_map
                # because octomap_server treats the map origin (0,0,0) as the sensor.
                # Any point > 5 m from origin is discarded if you leave this at 5.0.
                'max_range': 0.0,
                'sensor_model/max_range': 0.0,

                # Height limits (make them wide so nothing is clipped)
                'pointcloud_min_z': -50.0,
                'pointcloud_max_z': 50.0,
                'occupancy_min_z': -50.0,
                'occupancy_max_z': 50.0,

                # Prevent the "Could not open file" warning
                'octomap_path': '/home/fishman/ros2_ws/test_map_full.ot',

                # Other
                'filter_ground_plane': False,
                'compress_map': True,
            }],
            remappings=[
                # RTAB-Map's launch.py uses namespace 'rtabmap' by default
                ('cloud_in', '/rtabmap/cloud_map'),
            ],
        ),
    ])