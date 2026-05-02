from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',  # or 'octomap_server' depending on version
            name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': 'map',
                'base_frame_id': 'base_link',

                'resolution': 0.15,              # Start with 15 cm, lower if CPU allows
                'max_range': 5.0,
                'sensor_model/max_range': 5.0,

                'occupancy_min_z': 0.05,
                'occupancy_max_z': 10.0,
                'point_cloud_min_z': 0.05,
                'point_cloud_max_z': 10.0,
                
                'filter_ground_plane': False,    # Set True if you want a 2D ground projection
                'ground_filter/distance': 0.05,
                'ground_filter/angle': 0.15,
                'ground_filter/plane_distance': 0.05,
                'compress_map': True,            # Save memory
            }],
            remappings=[
                # Subscribe to RTAB-Map's assembled map cloud
                ('cloud_in', '/rtabmap/cloud_map'),
                
                # Outputs you can use
                # '/octomap_server/octomap_binary'   → Full 3D octree
                # '/octomap_server/octomap_full'     → Full probability octree  
                # '/octomap_server/projected_map'    → 2D occupancy grid
                # '/occupied_cells_vis_array'        → MarkerArray for RViz
            ],
        ),
    ])