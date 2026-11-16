"""Workstream C stub world (guidebook 4.5, C4): stub world + planner + frontier server + mission brain.

    ros2 launch drone_mission stub.launch.py world:=w3_tree_maze start_index:=0 speed_factor:=5 rviz:=true
    ros2 service call /mission/start std_srvs/srv/Trigger
Everything runs on the stub's simulated clock (faster than real time with speed_factor > 1).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    L = LaunchConfiguration
    sim = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='w3_tree_maze'),
        DeclareLaunchArgument('start_index', default_value='0'),
        DeclareLaunchArgument('speed_factor', default_value='5.0'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='drone_sim', executable='stub_world.py', name='stub_world', output='screen',
             parameters=[{'world': L('world'), 'start_index': L('start_index'), 'speed_factor': L('speed_factor'),
                          'seed': L('seed')}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('drone_nav'), 'launch', 'planner.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        Node(package='drone_frontier', executable='frontier_server', name='frontier_server', output='screen',
             parameters=[sim]),
        Node(package='drone_mission', executable='mission_brain', name='mission_brain', output='screen',
             parameters=[sim]),
        Node(package='rviz2', executable='rviz2', name='rviz2', condition=IfCondition(L('rviz')),
             arguments=['-d', os.path.join(get_package_share_directory('drone_bringup'), 'config', 'drone.rviz')],
             parameters=[sim]),
    ])
