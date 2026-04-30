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
        
        # Phase 3 - Configure and Launch RTAB-Map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
            ),
            launch_arguments={
                'rtabmap_args': '--delete_db_on_start',
                'rgb_topic': '/camera/rgb/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'camera_info_topic': '/camera/rgb/camera_info',
                'frame_id': 'base_link',
                'approx_sync': 'true',
                'qos': '2',
                'visual_odometry': 'true',
                'use_sim_time': 'true',
            }.items()
        ),
    ])

'''
go to PX4_Autopilot/tools/simulation/gz/models/"your depth camera model"/model.sdf
make sure the rgb and depth has the same aspect ratio.
'''
