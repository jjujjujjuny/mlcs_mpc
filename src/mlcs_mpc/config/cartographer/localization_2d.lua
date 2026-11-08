-- ============================================================================
--  Cartographer 2D 위치추정 (pure localization) — ./drive.sh localize 가 쓴다
--
--  저장한 맵(maps/<이름>.pbstream)을 얼려(frozen) 불러오고, 새 궤적은
--  맵에 붙여 위치만 낸다. 맵은 더 자라지 않는다.
--
--  설정은 mapping_2d.lua 를 그대로 쓰고 위치추정에 필요한 두 줄만 바꾼다
--  (Cartographer 공식 backpack_2d_localization.lua 와 같은 방식).
--
--  ★ 처음 몇 초는 위치가 맵에 안 붙어 있다. 새 궤적은 원점(맵핑 시작 자세)
--    에서 출발하고, 노드가 optimize_every_n_nodes 개 쌓여 첫 최적화가 돌아야
--    맵과의 제약이 생긴다. 맵핑을 시작한 자리 근처에서 출발하고, 조금 움직여
--    RViz 에서 스캔이 맵 벽에 겹치는지 본 뒤 쓴다.
-- ============================================================================

include "mapping_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20

return options
