"""Cartographer-only stack for bag replay (used by scripts/carto_replay.sh, not installed).
    ros2 launch scripts/replay.launch.py config_dir:=DIR config:=replay.lua"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf = open(os.path.join(get_package_share_directory('drone_description'), 'urdf', 'drone.urdf')).read()
    cfg_dir, cfg = LaunchConfiguration('config_dir'), LaunchConfiguration('config')
    sim = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('config_dir'),
        DeclareLaunchArgument('config', default_value='replay.lua'),
        Node(package='drone_slam', executable='imu_bias_node.py', name='imu_bias_node', parameters=[sim]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node', output='screen',
             arguments=['-configuration_directory', cfg_dir, '-configuration_basename', cfg],
             remappings=[('scan', '/scan'), ('imu', '/imu')], parameters=[sim]),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node', output='screen',
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'], parameters=[sim]),
    ])
