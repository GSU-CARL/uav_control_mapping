from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_control_mapping'


def get_data_files():
    """Generate data_files list for package data."""
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Add world files
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        # Add config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ]
    
    # Recursively add model files and subdirectories
    for root, dirs, files in os.walk('model'):
        if files:  # Only add if directory has files
            target_dir = os.path.join('share', package_name, root)
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((target_dir, file_paths))
    
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishman',
    maintainer_email='norakvitou.s@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'manual_control_node = uav_control_mapping.manual_control:main',
            'simple_nav_node = uav_control_mapping.simple_nav:main',
            'waypoint_manager_node = uav_control_mapping.waypoint_manager:main',
            'trajectory_generator_node = uav_control_mapping.trajectory_generator:main',
            'trajectory_follower_node = uav_control_mapping.trajectory_follower:main',
            'lidar_time_injector = uav_control_mapping.lidar_time_injection:main',
        ],
    },
)
