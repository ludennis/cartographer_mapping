include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_imu",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_nav_sat = true,
  use_odometry = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  use_landmarks = false,
  publish_frame_projected_to_2d = false,
  fixed_frame_pose_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 15

-- no point of traying to SLAM over points on the vehicle 2019-01-18
TRAJECTORY_BUILDER_3D.min_range = 0
TRAJECTORY_BUILDER_3D.max_range = 150

-- 2019-01-18 commented
-- set these let ceres scan matcher to not trust the priors
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 1e-2
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1

-- 2019-01-18
-- Use more points for SLAMing and adapt a bit for the ranges of a car
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 5.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 250.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 8.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 400.

-- 2019-01-18
-- the submaps felt prety big - since the car moves faster, we want them to be
-- slightly smaller. Slamming at 10cm can be aggressive for cars and for the quality
-- of the rangefinder. Increased 'high_resolution', '*num_iterations' in the
-- various optimization problems also trades performance/quality
-- TRAJECTORY_BUILDER_3D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_3D.submaps.high_resolution = .25
TRAJECTORY_BUILDER_3D.submaps.low_resolution = .60
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50

-- 2019-01-18
-- Trying to loop close too often will cost CPU and not buy you a lot. There is
-- little point in trying more than once per submap
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 200
-- POSE_GRAPH.constraint_builder.min_score = 0.5


-- 2019-01-18
-- Crazy search window to force loop closure to work. All other changes are probably not needed
POSE_GRAPH.constraint_builder.max_constraint_distance = 2.5e2
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 1e2
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1e1
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(1e1)
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 5e1

-- increases weighting for gps 2019-03-13
-- suggestion: increase it more to prevent zhanglang bridge from squeezing
-- POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 2e1
-- POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 0

-- increase submap size for mapping on zhanglang bridge 2019-03-13
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 6e1

-- decrease rotation_weight to let ceres scan matcher not trust priors 2019-03-14
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 5


-- changes made to find loop closure
-- increases min score to get better loop closure at zhanglang bridge 2019-03-13
POSE_GRAPH.constraint_builder.min_score = 0.4
-- increases sampling ratio to allow finding loop closure 2019-03-15
POSE_GRAPH.constraint_builder.sampling_ratio = 0.01
-- decrease submap size for more flexibility 2019-03-14
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 8e2
POSE_GRAPH.optimize_every_n_nodes = 1e2
POSE_GRAPH.optimization_problem.huber_scale = 1e2

POSE_GRAPH.max_num_final_iterations = 2e3

-- added for using velodyne packets 2019-03-24
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1;

return options
