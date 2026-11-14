-- 라이다 위치추정 + 오도메트리, 튐 억제판 (race.yaml loc_odom: strict, 2026-11-14 대회장)
--   대회장 track2a 주행 4개 (145229·145630·145822·150403): 직선 벽 긁힘이 모두 위치 튐과 겹쳤다.
--   · Cartographer 가 바퀴·자이로 오도메트리 대비 좌우로 0.3~0.7 m 씩 흘렀다가 1~2.8 m 로 튐
--   · 정지 중에도 129 cm·23°, 1 m·27° 튐 — 거울 모양 나선 트랙이라 먼 곳에 잘못 맞춤
--   바꾸는 것 (ekf 판 = localization_2d_odom.lua 와의 차이만):
--   1. 포즈그래프가 오도메트리를 믿는다 (0 → 1e4·1e4). 메인 PC 권고 (2026-11-14_1610 BCD 결과 C)
--   2. 맵과의 제약을 먼 곳에서 찾지 않는다: 창 1.5 m·30° → 0.5 m·10°, 전역 탐색 끔, 점수 기준 올림
include "localization_2d_odom.lua"
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e4
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e6
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.)
POSE_GRAPH.constraint_builder.min_score = 0.65
return options
