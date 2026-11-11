-- Cartographer 2D, local SLAM only (guidebook 2.5).
-- Humble note: cartographer_ros 2.0.900x requires the key use_pose_extrapolator
-- (missing from the book's listing); true = publish the pose every 5 ms using the
-- IMU-based prediction between scans, which is what the book describes.
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",             -- with an IMU, track the IMU's frame
  published_frame = "base_link",
  odom_frame = "odom",                     -- unused
  provide_odom_frame = false,              -- publish map -> base_link directly
  publish_frame_projected_to_2d = true,    -- level pose for the planner
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,     -- simulation; test 4 or 10 on real C1 data
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2

TRAJECTORY_BUILDER_2D.use_imu_data = true   -- ArduPilot's example uses false; we need tilt correction
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.
TRAJECTORY_BUILDER_2D.min_range = 0.20
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.min_z = -0.6          -- relative to the drone: rejects floor hits in flight
TRAJECTORY_BUILDER_2D.max_z = 0.6           -- rejects net hits
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.     -- A1.9 sweep: 0.2 -> 1 (with num_range_data 120, rotation_weight 80)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 80.     -- A1.9 sweep: 40 -> 80

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120    -- A1.9 sweep: 90 -> 120
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

POSE_GRAPH.optimize_every_n_nodes = 0       -- loop closure off

return options
