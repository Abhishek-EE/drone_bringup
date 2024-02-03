include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "zed_imu_link", -- Assuming 'imu_link' is the frame provided by the IMU
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false, -- Set to true if odometry data from ZED or another source is reliable
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_3d = true

TRAJECTORY_BUILDER_3D = {
  num_accumulated_range_data = 1,
  min_range = 0.1,
  max_range = 30.0,
  submaps = {
    high_resolution = 0.2,
    low_resolution = 0.45,
    num_range_data = 160,
  },
  voxel_filter_size = 0.05,

  use_imu_data = true,
  imu_gravity_time_constant = 10.0,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,
    angular_search_window = math.rad(20.0),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },
  ceres_scan_matcher = {
    occupied_space_weight_0 = 1.0,
    occupied_space_weight_1 = 6.0,
    translation_weight = 5e2,
    rotation_weight = 4e2,
    only_optimize_yaw = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },
  motion_filter = {
    max_time_seconds = 5,
    max_distance_meters = 0.1,
    max_angle_radians = math.rad(0.1),
  },
  imu_gravity_time_constant = 10.0,
  pose_extrapolator = {
    use_imu_based = true,
    imu_gravity_time_constant = 10.0,
  },
}

POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

return options