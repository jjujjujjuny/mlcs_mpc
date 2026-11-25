#!/usr/bin/env bash
# M: 연구실 mocap 트랙 (1108_7_track_mocap) 에서 처음부터 — run_L.sh 와 같은 두 단계, 차 값만 연구실 실측으로
#    서보 steering_tau 0.03 (11-10 mocap 개루프 재생: τ 0.13 이면 1 s 위치 오차 2 배), 연구실 마찰 0.41
#   1단계 M1_<태그>  lag 저수준, DR full 적응 커리큘럼, 처음부터 60M
#   2단계 M2_<태그>  1단계 best.pt 에서 VESC (11-24 설정) + DR 0.5 + 시작 분산 + 끊김 0.03, 80M
#   끝나면 DR 강건성 평가 (30 s — 연구실 트랙 둘레 약 15 m)
#   OBS 를 주면 관측 속도 모델 (race_env obs_vel — true | delay | kf | gyro) 을 바꿔 태그 뒤에 붙인다
# 끊기면: python3 learned/rl/train.py --resume logs/rl/exp/<이름> --device cuda:0
# 쓰기: bash learned/rl/experiments/run_M.sh [태그] [obs_vel]      (PY=파이썬 경로, 기본 python3)
cd "$(dirname "$0")/../../.."
E=logs/rl/exp; mkdir -p $E/eval_dr
T=1108_7_track_mocap
tag=${1:-lab}; obs=${2:-}
X=""; [ -n "$obs" ] && X="--obs-vel $obs"
PY=${PY:-python3}
P="$PY -u learned/rl/train.py --profile fast --device cuda:0 --track $T --car steering_tau=0.03 friction=0.41 $X"
$P --actuator lag --dr full --curriculum adaptive --total-steps 60e6 --out $E/M1_$tag > $E/M1_$tag.log 2>&1
$P --epochs 3 --batch-size 65536 --actuator vesc --dr full --curriculum none --dr-level 0.5 --start-spread 1 \
   --dr-drop 0.03 --num-envs 65536 --n-steps 32 --init $E/M1_$tag/best.pt --lr-start 3e-4 --det-eval-every 10 --sim-check \
   --total-steps 80e6 --out $E/M2_$tag > $E/M2_$tag.log 2>&1
$PY -u learned/rl/eval_vec.py $E/M2_$tag/best_det.pt $E/M2_$tag/policy.pt --labels M2_${tag}_det M2_${tag}_final --actuators vesc \
   --dr-scales 0 0.5 1 1.5 --drop 0 0.05 --starts 8 --reps 16 --time 30 --device cuda:0 -o $E/eval_dr/M2_$tag.md > $E/eval_dr/M2_$tag.log 2>&1
echo "=== M $tag 끝 $(date +%T)"
