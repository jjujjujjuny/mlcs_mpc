-- 라이다 위치추정 + 바퀴·자이로 오도메트리 (race.yaml loc_odom: ekf, 2026-11-11)
--   wheel_odom 노드의 /odometry/filtered 를 Cartographer 가 다음 위치 예측 (pose extrapolator) 에 쓴다
--   (drive.sh 가 odom:=/odometry/filtered 로 remap). 긴 직선에서 스캔이 앞뒤로 애매할 때 오도메트리가
--   '이만큼 왔다' 를 잡아 준다 (1109_3 직선에서 앞뒤 50 cm 미끄러짐).
--   포즈그래프 오도메트리 가중치는 0 그대로 (mapping_2d.lua) — 맵·그래프를 휘게 하지 않는다.
--   use_imu_data 는 false 그대로 — 자이로는 이미 wheel_odom 에 들어 있다.
include "localization_2d.lua"
options.use_odometry = true
return options
