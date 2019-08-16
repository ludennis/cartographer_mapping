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
  project_to_nav_sat = true,
  latitude_reference = 24.775084704,
  longitude_reference = 121.045888961,
  altitude_reference = 146.593,
  tracking_frame = "base_imu",
  pipeline = {
    {
        action = "height_clamping",
        min_height = -200,
        max_height = -1.0,
    },
    {
        action = "min_max_range_filter",
        min_range = 2.0,
        max_range = 5.,
    },
    {
        action = "motion_filter",
        filter_speed_kmph = 1.5,
        filter_distance = 0.0,
    },
    {
        action = "write_pcd",
        filename = "full.pcd",
    },
  }
}

return options
