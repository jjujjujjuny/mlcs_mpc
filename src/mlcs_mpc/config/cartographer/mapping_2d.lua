-- ============================================================================
--  Cartographer 2D 맵핑 — ./drive.sh mapping 이 쓴다
--
--  바탕: UNICORN Racing (UNIST) stack_master/config/mapping_2d.lua
--    https://github.com/HMCL-UNIST/unicorn-racing-stack
--
--  ★ UNICORN 과 같은 것
--    · 휠 오도메트리를 포즈그래프에 넣지 않는다 (use_odometry=false,
--      odometry_*_weight=0). 슬립이 생기면 오도메트리 제약이 맵을 휜다.
--    · voxel_filter 0.025, motion_filter 각도 0.1°, 루프클로저 창 1.5 m / 30°
--
--  ★ UNICORN 과 다른 것 — 우리 차 구조 때문
--    · IMU 를 안 쓴다. vesc_driver 의 /sensors/imu/raw 는 header.frame_id 가
--      비어 있고(vesc_driver.cpp 에서 설정 안 함), base_link→imu TF 도 없다.
--      Cartographer 는 IMU 프레임을 TF 로 찾으므로 그대로 넣으면 기동 실패.
--      → 1단계는 라이다 단독. IMU 를 붙이려면 frame_id + static TF 부터.
--    · odom 프레임을 Cartographer 가 만들지 않는다 (provide_odom_frame=false).
--      우리 bringup 의 vesc_to_odom 이 이미 odom→base_link 를 낸다
--      (f1tenth_stack/config/vesc.yaml publish_tf: true). 둘이 내면 TF 가 싸운다.
--      Cartographer 는 map→odom 만 낸다. 오도메트리 '값'은 여전히 안 쓴다.
--
--  IMU·오도메트리 없이 스캔매칭만으로 따라가므로 **천천히** 몬다
--  (drive.sh mapping 은 MLCS_SPEED 기본 1.5 m/s).
-- ============================================================================

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  publish_tracked_pose = true,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- UST-10LX: 정격 10 m. 실내 트랙이라 그 이상은 노이즈만 늘린다
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 10.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

-- IMU·오도메트리가 없으니 초기 추정을 상관 스캔매칭으로 잡는다 (필수)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.
POSE_GRAPH.optimize_every_n_nodes = 100
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)

return options
