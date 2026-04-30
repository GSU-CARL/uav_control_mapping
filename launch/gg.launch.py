import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('uav_control_mapping')
    bridge_config = os.path.join(pkg_dir, 'config', 'depth_cam_bridge.yaml')
    return LaunchDescription([
        # Phase 4 - TF / Frame Setup
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_camera',
            arguments=['0', '0', '0', '-1.5707', '0', '-1.5707', 'base_link', 'camera_link']
        ),

        # Depth Camera Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{
                'config_file': bridge_config,
            }],
            output='screen'
        ),
        
        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'subscribe_depth': True,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'database_path': '/tmp/rtabmap.db',
                'Grid/3D': 'true',
                'Grid/CellSize': '0.05',
                'Grid/RangeMax': '5.0',
                'Grid/RangeMin': '0.3',
                'RGBD/LinearUpdate': '0.1',
                'RGBD/AngularUpdate': '0.05',
                'cloud_output_voxel_size': 0.05,
                'Rtabmap/TimeThr': '700',
                'Mem/STMSize': '30',
                'use_sim_time': True,
            }],
            remappings=[
                ('rgb/image',        '/camera/rgb/image_raw'),
                ('depth/image',      '/camera/depth/image_raw'),
                ('rgb/camera_info',  '/camera/rgb/camera_info'),
                ('odom',             '/mavros/local_position/odom'),
            ],
            arguments=['--delete_db_on_start']
        ),

        # Octomap server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'resolution': 0.05,
                'frame_id': 'map',
                'sensor_model/max_range': 5.0,
                'occupancy_min_z': -0.5,
                'occupancy_max_z': 5.0,
                'filter_speckles': True,
                'compress_map': True,
                'use_sim_time': True,
            }],
            remappings=[
                ('cloud_in', '/rtabmap/cloud_map'),
            ],
        ),
    ])

'''
go to PX4_Autopilot/tools/simulation/gz/models/"your depth camera model"/model.sdf
make sure the rgb and depth has the same aspect ratio.
'''
