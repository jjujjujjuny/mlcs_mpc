#!/usr/bin/env bash
# K: GPU 최대 병렬 (RTX 3060 하나에 J 와 함께 4 프로세스) — VESC 설정 11-24, I (rl17, DR 1.0) 에서 이어
#   K_track1c   대회 결승 트랙 track1c (58.5 m) DR 1.0, 120M
#   K_track2a   대회 연습 트랙 track2a (54.6 m) DR 1.0, 120M
#   K_rl17_dr05 rl17 DR 0.5, 40M (J = DR 1.0 과 비교)
# 끝나면 각각 DR 강건성 평가 → logs/rl/exp/eval_dr/<이름>.md
# 끊기면: python3 learned/rl/train.py --resume logs/rl/exp/<이름> --device cuda:0
cd "$(dirname "$0")/../../.."
E=logs/rl/exp
tr () { name=$1; shift; python3 -u learned/rl/train.py --profile fast --device cuda:0 --epochs 3 --batch-size 65536 \
  --actuator vesc --dr full --curriculum none --start-spread 1 --dr-drop 0.03 --num-envs 65536 --n-steps 32 \
  --init $E/I_F_dr1/policy.pt --lr-start 2e-4 --det-eval-every 10 --sim-check "$@" --out $E/$name > $E/$name.log 2>&1
  python3 -u learned/rl/eval_vec.py $E/$name/policy.pt --labels $name --actuators vesc --dr-scales 0 0.5 1 1.5 2 \
  --drop 0 0.05 --starts 8 --reps 16 --time 60 --device cuda:0 -o $E/eval_dr/$name.md > $E/eval_dr/$name.log 2>&1
  echo "=== $name 끝 $(date +%T)"; }
tr K_track1c   --track track1c_track --dr-level 1.0 --total-steps 120e6 &
tr K_track2a   --track track2a_track --dr-level 1.0 --total-steps 120e6 &
tr K_rl17_dr05 --dr-level 0.5 --total-steps 40e6 &
wait
echo KDONE
