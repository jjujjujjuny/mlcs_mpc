-- 라이다 위치추정 + 오도메트리, 튐 억제판 (race.yaml loc_odom: strict, 2026-11-14 대회장)
--   대회장 track2a 주행 4개 (145229·145630·145822·150403): 직선 벽 긁힘이 모두 위치 튐과 겹쳤다.
--   · Cartographer 가 바퀴·자이로 오도메트리 대비 좌우로 0.3~0.7 m 씩 흘렀다가 1~2.8 m 로 튐
--   · 정지 중에도 129 cm·23°, 1 m·27° 튐 — 거울 모양 나선 트랙이라 먼 곳에 잘못 맞춤
--   바꾸는 것 (ekf 판 = localization_2d_odom.lua 와의 차이만):
--   1. (오도메트리 1e4 는 15:35 주행에서 되돌림 — 아래)
--   2. 맵과의 제약을 먼 곳에서 찾지 않는다: 창 1.5 m·30° → 1.0 m·20°, 전역 탐색 끔, 점수 기준 올림
include "localization_2d_odom.lua"
-- ★ 11-14 15:35 track1a (153533): 1e4 이면 랩 3개 (22.0·20.4·19.6 s) 는 깨끗했지만 Cartographer 가 맵에서 계속 멀어져
--   맵 고정 보정이 1~1.5 m, yaw 는 상한 10° 에 25 s 넘게 붙음 → 상한을 넘자 맞춤 0.04 → 위치 놓침. ekf 판 (0) 은 ±50 cm·3~5°.
--   → 오도메트리 가중치는 0 으로 되돌리고 먼 곳 맞춤만 막는다 (튐은 이것이 원인)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e6
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(20.)
POSE_GRAPH.constraint_builder.min_score = 0.65
return options
