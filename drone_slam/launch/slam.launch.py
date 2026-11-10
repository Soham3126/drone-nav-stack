"""Cartographer 2D + occupancy grid (guidebook 2.5)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('drone_slam'), 'config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config', default_value='drone_2d.lua'),
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             output='screen',
             arguments=['-configuration_directory', config_dir,
                        '-configuration_basename', config],
             remappings=[('scan', '/scan'), ('imu', '/imu')],
             parameters=[{'use_sim_time': use_sim_time}]),
        # resolution / period are command-line flags of this node, not ROS parameters
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='cartographer_occupancy_grid_node', output='screen',
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
             parameters=[{'use_sim_time': use_sim_time}]),
    ])
