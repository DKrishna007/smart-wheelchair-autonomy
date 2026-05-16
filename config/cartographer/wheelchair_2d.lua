include "map_builder.lua"
include "trajectory_builder.lua"

  options = {
  map_builder = MAP_BUILDER,
      trajectory_builder = TRAJECTORY_BUILDER,
      map_frame = "map",
      tracking_frame = "imu_link",
      published_frame = "base_link",
      odom_frame = "odom",
      provide_odom_frame = true,
      use_imu_data = true,
      num_point_clouds = 1,
      pose_publish_period_sec = 5e-3,
    }

MAP_BUILDER.use_trajectory_builder_2d = true
    TRAJECTORY_BUILDER_2D.min_range = 0.3
    TRAJECTORY_BUILDER_2D.max_range = 30.0
    TRAJECTORY_BUILDER_2D.use_imu_data = true
    TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
    POSE_GRAPH.optimize_every_n_nodes = 35

    return options
