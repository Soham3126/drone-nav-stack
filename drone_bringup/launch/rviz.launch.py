"""RViz2 on sim time with the stack's displays (Grid, Map, LaserScan, TF)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('drone_bringup'), 'config', 'drone.rviz')
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', arguments=['-d', cfg],
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
