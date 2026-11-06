-- Cartographer in "pure localization" mode: load a saved map (.pbstream)
-- and only work out where the robot is on it. Nothing new is added to the map.
include "cartographer_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20

return options
