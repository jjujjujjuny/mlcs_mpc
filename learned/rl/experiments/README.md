# RL 실험 스크립트

결과는 모두 `logs/rl/exp/<이름>/` (학습) 와 `logs/rl/exp/eval_dr/` (DR 강건성 평가), 영상은 `logs/rl/video/`.
스크립트는 저장소 루트로 `cd` 한 뒤 돈다 (`bash learned/rl/experiments/<이름>.sh`). 결과 해석은 `docs/RL_RACING.md`.

| 스크립트 | 기계 | 내용 |
|---|---|---|
| `run_all.sh`, `run_rest.sh`, `run_next.sh`, `run_final.sh`, `run_F.sh` | 맥 (MPS, 옛 python 경로) | A~F: 저수준 lag / vesc / none × 커리큘럼, E·F (VESC, DR 0.5) |
| `run_dr.sh` | PC (RTX 3060) | G (E → DR 1.0), H (E → DR 0) 동시 + DR 평가 |
| `run_eval_dr.sh` | PC | 정책 5개 DR 강건성 평가 (정책마다 프로세스, 조건 전부 한 배치) |
| `run_I.sh` | PC | I (F → DR 1.0) — 11-08 VESC 설정의 최종 정책 |
| `run_J.sh` | PC | VESC 11-24 설정: I 재평가 + J (I → DR 1.0) |
| `run_K.sh` | PC | GPU 4 프로세스 병렬. K_rl17_dr05 (DR 0.5). K_track1c/2a (rl17 정책을 대회 트랙에 이어 학습) 는 실패해 중단 |
| `run_L.sh` | PC | 대회 트랙 track1c·track2a 처음부터: lag + DR 커리큘럼 60M → VESC + DR 0.5 80M → 평가 |
| `run_M.sh` | PC (RTX 3080) | 연구실 mocap 트랙 1108_7 처음부터, 서보 τ 0.03·마찰 0.41 (11-25 재생 결과). `run_M.sh <태그> [true|kf|gyro]` = 관측 속도 모델 |
| `render_final.sh` | PC | 최종 영상 (대회 트랙 2 개, rl17 J vs I) |
