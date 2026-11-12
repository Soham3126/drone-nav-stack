"""Gyro bias removal + Cartographer 2D + occupancy grid (guidebook 2.5)."""
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
    scan_topic, imu_topic = LaunchConfiguration('scan_topic'), LaunchConfiguration('imu_topic')
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config', default_value='drone_2d.lua'),
        # gyro bias removal: /imu_raw -> /imu (the IMU driver, or the simulation, publishes /imu_raw)
        Node(package='drone_slam', executable='imu_bias_node.py', name='imu_bias_node', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             output='screen',
             arguments=['-configuration_directory', config_dir,
                        '-configuration_basename', config],
             remappings=[('scan', scan_topic), ('imu', imu_topic)],
             parameters=[{'use_sim_time': use_sim_time}]),
        # resolution / period are command-line flags of this node, not ROS parameters
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='cartographer_occupancy_grid_node', output='screen',
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
             parameters=[{'use_sim_time': use_sim_time}]),
    ])
