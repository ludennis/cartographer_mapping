-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

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
  fixed_frame_pose_sampling_ratio = 1.,
  landmarks_sampling_ratio = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  imu_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 12

POSE_GRAPH.optimization_problem.log_solver_summary = true


TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 5.
--
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
-- TRAJECTORY_BUILDER_3D.min_range = 2
TRAJECTORY_BUILDER_3D.max_range = 400.
-- TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 100
-- TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.02
--
--
-- TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.2
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(0.5)
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.translation_delta_cost_weight
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.rotation_delta_cost_weight
--
--
--
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.2
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 100.
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.8
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 10
--
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.global_sampling_ratio = 0.03
POSE_GRAPH.global_constraint_search_after_n_seconds = 10
-- POSE_GRAPH.matcher_translation_weight = 15
-- POSE_GRAPH.matcher_rotation_weight = 2.2e5 --default at 1.6e3
--
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.5
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.min_score = 0.5 -- 0.62 for fast correlative scan matcher
-- POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2.2e4
POSE_GRAPH.constraint_builder.max_constraint_distance = 10 -- for local constraints
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(5)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.5 -- 0.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.3
--
--
-- POSE_GRAPH.optimization_problem.acceleration_weight = 0.00001 -- IMU
-- POSE_GRAPH.optimization_problem.rotation_weight = 0.000001 -- IMU
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 100
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 0

return options
