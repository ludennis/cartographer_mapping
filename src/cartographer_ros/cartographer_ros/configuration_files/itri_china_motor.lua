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
  use_nav_sat = false,
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
  imu_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 12

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

-- set these let ceres scan matcher to not trust the priors
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 2e3

-- set the range of rangefinder to be used in scan matching
TRAJECTORY_BUILDER_3D.min_range = 2

-- other configurations for global slam
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 2e2

-- settings to find better pose graph solution
POSE_GRAPH.constraint_builder.min_score = 4e-1
POSE_GRAPH.constraint_builder.sampling_ratio = 3e-2

-- settings used for itri_basement
-- TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.8 --def 0.45, high res 0.15, mid 0.25
-- TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.05 -- default 0.10, high res 0.01, mid 0.03

-- increases frequency of loop closure attempts
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 1.2e2
POSE_GRAPH.optimize_every_n_nodes = 1.2e2
POSE_GRAPH.optimization_problem.huber_scale = 5e2

-- options to force loop closure to work
POSE_GRAPH.constraint_builder.max_constraint_distance = 2.5e2
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 5e1
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1e1
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(2e1)
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 5e1

return options
