"""Simulation ROS stack, by profile (guidebook 4.2).

    ros2 launch drone_bringup sim.launch.py profile:=gps [slam:=true] [world:=w0_box_room]

Gazebo and ArduPilot SITL are started separately by scripts/sim_up.sh.

Profile   ROS nodes
gps       Gazebo bridge, robot_state_publisher, truth relay, Cartographer (slam:=true)
truth     + bridge (pose from /truth/pose, or through pose_mangler when mangle:=<yaml>)   GPS off
slam      + bridge (pose from Cartographer TF)                                          GPS off
full      + planner, frontier server, mission brain     <- needs stages 3-4
"""
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

READY = {'gps', 'truth', 'slam'}
ALL = ('gps', 'truth', 'slam', 'full')


def setup(context):
    profile = LaunchConfiguration('profile').perform(context)
    slam = LaunchConfiguration('slam').perform(context).lower() in ('1', 'true', 'yes')
    world = LaunchConfiguration('world').perform(context)
    out_dir = LaunchConfiguration('out_dir').perform(context)
    gate = LaunchConfiguration('gate').perform(context).lower() in ('1', 'true', 'yes')
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
    slam_args = {'use_sim_time': 'true'}
    if gate:   # A1.11: Cartographer's inputs pass through pausable gates
        for name, typ, src in (('scan_gate', 'sensor_msgs/msg/LaserScan', '/scan'),
                               ('imu_gate', 'sensor_msgs/msg/Imu', '/imu')):
            actions.append(Node(package='drone_sim', executable='stream_gate.py', name=name, output='screen',
                                parameters=[{'type': typ, 'input': src, 'output': src + '_gated',
                                             'use_sim_time': True}]))
        slam_args.update(scan_topic='/scan_gated', imu_topic='/imu_gated')
    if profile in ('truth', 'slam', 'full'):
        actions += bridge_actions(profile, LaunchConfiguration('mangle').perform(context), gate)
    if slam:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('drone_slam'), 'launch', 'slam.launch.py')),
            launch_arguments=slam_args.items()))
    return actions


def bridge_actions(profile, mangle, gate=False):
    """The bridge, fed by ground truth (truth profile, optionally through pose_mangler and/or a pausable
    pose gate) or by Cartographer's TF (slam, full); plus the rangefinder relay (the simulated MTF-01P,
    on its own MAVLink link like a sensor wired to the flight controller)."""
    acts = [Node(package='drone_sim', executable='range_relay.py', name='range_relay', output='screen',
                 parameters=[{'use_sim_time': True}])]
    bp = {'use_sim_time': True, 'ground_clearance_m': 0.145}
    if profile == 'truth':
        bp.update(pose_source='topic', pose_topic='/truth/pose')
        if mangle and mangle != 'none':
            with open(os.path.expanduser(mangle)) as f:
                mp = yaml.safe_load(f) or {}
            # ROS parameters are typed: write 0 as 0.0 for the float ones, whatever the yaml says
            mp = {k: (float(v) if isinstance(v, int) and not isinstance(v, bool) and k != 'seed' else v)
                  for k, v in mp.items()}
            mp.update(input='/truth/pose', output='/pose_in', use_sim_time=True)
            acts.append(Node(package='drone_sim', executable='pose_mangler.py', name='pose_mangler',
                             output='screen', parameters=[mp]))
            bp.update(pose_topic='/pose_in', reset_topic='/pose_mangler/reset_counter')
            acts.append(LogInfo(msg=f'[sim.launch] pose_mangler: {mp}'))
        if gate:   # B6: pause only the pose stream
            acts.append(Node(package='drone_sim', executable='stream_gate.py', name='pose_gate', output='screen',
                             parameters=[{'type': 'geometry_msgs/msg/PoseStamped', 'input': bp['pose_topic'],
                                          'output': '/pose_gated', 'use_sim_time': True}]))
            bp.update(pose_topic='/pose_gated')
    else:
        bp.update(pose_source='tf')
    acts.append(Node(package='drone_bridge', executable='bridge_node', name='bridge', output='screen',
                     parameters=[bp]))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='gps'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('world', default_value='w0_box_room'),
        DeclareLaunchArgument('out_dir', default_value=''),
        DeclareLaunchArgument('mangle', default_value='none'),
        DeclareLaunchArgument('gate', default_value='false'),
        OpaqueFunction(function=setup),
    ])
