-- Cartographer settings for the Raspbot: 2D lidar ONLY.
-- The Raspbot V1 has no IMU and no wheel encoders, so Cartographer has to work
-- out how the robot moved purely by matching each new scan to the map.
-- Edit on the Pi:  ~/raspbot_lidar_ws/src/raspbot_mapping/config/cartographer_2d.lua

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",        -- the robot body
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,           -- we have no odometry, so Cartographer makes "odom"
  publish_frame_projected_to_2d = true,
  use_odometry = false,                -- no wheel encoders
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,                 -- listen to /scan
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
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

-- No IMU on the Raspbot
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- Which lidar points to use (metres)
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Without odometry, search harder for the best match of each scan
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Only add a scan to the map after the robot moved/turned a little
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- Size of each submap (number of scans)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

-- Loop closure (fixing drift when the robot returns to a known place)
POSE_GRAPH.optimize_every_n_nodes = 40
POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

return options
