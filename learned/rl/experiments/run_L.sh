#!/usr/bin/env bash
# L: 대회장 트랙에서 처음부터 — A → E 와 같은 두 단계 (rl17 에서 이어 학습한 K_track* 는 처음 보는 넓은 트랙에서
#    모든 정지 출발이 2 s 안에 벽 → 학습이 정체 (벽 3e-2/스텝, 랩 없음) 해서 중단, 2026-11-24)
#   1단계 L1_<트랙>  lag 저수준, DR full 적응 커리큘럼, 처음부터 60M (fast 기본 16384 × 64)
#   2단계 L2_<트랙>  1단계 best.pt 에서 VESC (11-24 설정) + DR 0.5 + 시작 분산 + 끊김 0.03, 80M
#   끝나면 DR 강건성 평가 (60 s — 트랙이 길다)
# 끊기면: python3 learned/rl/train.py --resume logs/rl/exp/<이름> --device cuda:0
cd "$(dirname "$0")/../../.."
E=logs/rl/exp
P="python3 -u learned/rl/train.py --profile fast --device cuda:0"
two () { t=$1; n=${t%_track}
  $P --track $t --actuator lag --dr full --curriculum adaptive --total-steps 60e6 --out $E/L1_$n > $E/L1_$n.log 2>&1
  $P --track $t --epochs 3 --batch-size 65536 --actuator vesc --dr full --curriculum none --dr-level 0.5 --start-spread 1 \
     --dr-drop 0.03 --num-envs 65536 --n-steps 32 --init $E/L1_$n/best.pt --lr-start 3e-4 --det-eval-every 10 --sim-check \
     --total-steps 80e6 --out $E/L2_$n > $E/L2_$n.log 2>&1
  python3 -u learned/rl/eval_vec.py $E/L2_$n/best_det.pt $E/L2_$n/policy.pt --labels L2_${n}_det L2_${n}_final --actuators vesc \
     --dr-scales 0 0.5 1 1.5 --drop 0 0.05 --starts 8 --reps 16 --time 60 --device cuda:0 -o $E/eval_dr/L2_$n.md > $E/eval_dr/L2_$n.log 2>&1
  echo "=== $t 끝 $(date +%T)"; }
two track1c_track &
two track2a_track &
wait
echo LDONE
