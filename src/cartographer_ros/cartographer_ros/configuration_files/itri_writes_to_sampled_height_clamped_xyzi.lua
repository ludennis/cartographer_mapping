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
    -- {
    --     action = "fixed_ratio_sampler",
    --     sampling_ratio = 1.0,
    -- },
    {
        action = "intensity_to_color",
        min_intensity = 0.,
        max_intensity = 256.,
    },
    {
        action = "min_max_range_filter",
        min_range = 0.,
        max_range = 100.,
    },
    {
        action = "height_clamping",
        min_height = -50,
        max_height = 0.0,
    },
    {
        action = "write_pcd",
        filename = "clamped.pcd",
    },
  }
}

return options
