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

include "slam_3d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- Frequent optimizations for localization purposes
POSE_GRAPH.optimize_every_n_nodes = 3

-- Speed up computation
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5 * POSE_GRAPH.constraint_builder.sampling_ratio
POSE_GRAPH.max_num_final_iterations = 1
--POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.3 * POSE_GRAPH.optimization_problem.odometry_rotation_weight

-- Frequently look for global matches
POSE_GRAPH.global_constraint_search_after_n_seconds = 5

-- Set higher weights for odometry in localization, to avoid jumps on pose estimation during navigation
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 1000
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1000

POSE_GRAPH.constraint_builder.min_score = 0.6
--POSE_GRAPH.constraint_builder.global_localization_min_score = 0.62

return options
