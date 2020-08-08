include "map_builder.lua"
include "trajectory_builder.lua"

options = {
    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,
    map_frame = "map",
    tracking_frame = "imu_bosch_link",
    published_frame = "base_link",
    odom_frame = "odom",
    provide_odom_frame = true,
    use_odometry = false,
    use_nav_sat = false,
    use_landmarks = false,
    publish_frame_projected_to_2d = true,
    num_laser_scans = 1,
    num_multi_echo_laser_scans = 0,
    num_subdivisions_per_laser_scan = 1,
    rangefinder_sampling_ratio = 1,
    odometry_sampling_ratio = 1,
    fixed_frame_pose_sampling_ratio = 1,
    imu_sampling_ratio = 1,
    landmarks_sampling_ratio = 1,
    num_point_clouds = 0,
    lookup_transform_timeout_sec = 0.2,
    submap_publish_period_sec = 0.3,
    pose_publish_period_sec = 5e-3,
    trajectory_publish_period_sec = 30e-3,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_2D.use_imu_data = true

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

POSE_GRAPH.optimization_problem.huber_scale = 1e2

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 20.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

--TRAJECTORY_BUILDER_2D.num_accumulated_range_data=2
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight=30
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 2



return options