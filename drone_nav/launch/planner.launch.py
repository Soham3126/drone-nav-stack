"""Nav2 planner server + lifecycle manager only (guidebook 2.6)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planner_yaml = os.path.join(get_package_share_directory('drone_nav'), 'config', 'planner.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[planner_yaml, {'use_sim_time': use_sim_time}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['planner_server']}]),
    ])
