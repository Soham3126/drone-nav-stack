"""Simulation ROS stack, by profile (guidebook 4.2).

    ros2 launch drone_bringup sim.launch.py profile:=gps [slam:=true] [world:=w0_box_room]

Gazebo and ArduPilot SITL are started separately by scripts/sim_up.sh.

Profile   ROS nodes
gps       Gazebo bridge, robot_state_publisher, truth relay, Cartographer (slam:=true)
truth     + bridge (pose from /truth/pose)              <- needs drone_bridge (stage 3)
slam      + bridge (pose from Cartographer TF)          <- needs drone_bridge (stage 3)
full      + planner, frontier server, mission brain     <- needs stages 3-4
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

READY = {'gps'}
ALL = ('gps', 'truth', 'slam', 'full')


def setup(context):
    profile = LaunchConfiguration('profile').perform(context)
    slam = LaunchConfiguration('slam').perform(context).lower() in ('1', 'true', 'yes')
    world = LaunchConfiguration('world').perform(context)
    out_dir = LaunchConfiguration('out_dir').perform(context)
    if profile not in ALL:
        raise RuntimeError(f"unknown profile '{profile}'; choose one of {ALL}")
    if profile not in READY:
        raise RuntimeError(f"profile '{profile}' needs packages that are not built yet "
                           f"(drone_bridge etc.). Ready now: {sorted(READY)}")

    bringup = get_package_share_directory('drone_bringup')
    urdf_path = os.path.join(get_package_share_directory('drone_description'), 'urdf', 'drone.urdf')
    with open(urdf_path) as f:
        urdf = f.read()

    actions = [
        LogInfo(msg=f'[sim.launch] profile={profile} slam={slam} world={world} out_dir={out_dir}'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
             parameters=[{'config_file': os.path.join(bringup, 'config', 'gz_bridge.yaml')}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
        Node(package='drone_sim', executable='truth_relay.py', name='truth_relay', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='drone_sim', executable='imu_drift_sim.py', name='imu_drift_sim', output='screen',
             parameters=[{'use_sim_time': True}]),
    ]
    if not slam:   # slam.launch.py starts the bias node itself
        actions.append(Node(package='drone_slam', executable='imu_bias_node.py', name='imu_bias_node',
                            output='screen', parameters=[{'use_sim_time': True}]))
    if slam:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('drone_slam'), 'launch', 'slam.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='gps'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('world', default_value='w0_box_room'),
        DeclareLaunchArgument('out_dir', default_value=''),
        DeclareLaunchArgument('mangle', default_value='none'),
        OpaqueFunction(function=setup),
    ])
