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

options = {
  tracking_frame = "base_imu",
  pipeline = {
    {
        action = "min_max_range_filter",
        min_range = 1.,
        max_range = 200.,
    },
    {
        action = "motion_filter",
        filter_speed_kmph = 1.0,
        filter_distance = 0.1,
    },
    {
        action = "motion_filter",
        filter_speed_kmph = 0.1,
        filter_distance = 0.01,
    },
    {
        action = "intensity_to_color",
        min_intensity = 0.,
        max_intensity = 256.,
    },
    {
        --action = "grid_map_write_pcd",
        action = "write_pcd",
        filename = "full.pcd",
    },
  }
}

return options
