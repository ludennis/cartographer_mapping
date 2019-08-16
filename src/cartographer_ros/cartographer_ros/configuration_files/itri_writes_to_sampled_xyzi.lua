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

include "transform.lua"

SAMPLING_RATIO = 0.003

options = {
  project_to_nav_sat = true,
  -- ITRI
  -- latitude_reference = 24.775084704,
  -- longitude_reference = 121.045888961,
  -- altitude_reference = 146.593,

  -- Shalun
  latitude_reference = 22.922378267,
  longitude_reference = 120.288798412,
  altitude_reference = 46.6147044534,
  tracking_frame = "base_imu",
  pipeline = {
    {
        action = "fixed_ratio_sampler",
        sampling_ratio = SAMPLING_RATIO,
    },
    {
        action = "min_max_range_filter",
        min_range = 3.0,
        max_range = 300.,
    },
    {
        action = "motion_filter",
        filter_speed_kmph = 1.5,
        filter_distance = 0.0,
    },
    {
        action = "intensity_to_color",
        min_intensity = 0.,
        max_intensity = 256.,
    },
    {
        action = "write_pcd",
        filename = "sampled-" .. SAMPLING_RATIO .. ".pcd",
    },
  }
}

return options
