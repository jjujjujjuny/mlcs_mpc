#!/usr/bin/env bash
# J: VESC 설정 11-24 (docs/vesc6_f1tenth_1124.xml — PID 0.003/0.0002/0.0002, R 7.6 mΩ, 개루프 보조 5 A) 로 바뀐 뒤.
#   ① I (11-08 설정으로 학습한 최종 정책) 를 새 설정에서 DR 평가  ② I 에서 이어 새 설정으로 DR 1.0 40M  ③ J DR 평가
# 끊기면: python3 learned/rl/train.py --resume logs/rl/exp/J_I_vesc1124 --device cuda:0
cd "$(dirname "$0")/../../.."
E=logs/rl/exp
ev () { python3 -u learned/rl/eval_vec.py $2 --labels $1 --actuators vesc --dr-scales 0 0.5 1 1.5 2 --drop 0 0.05 \
  --starts 8 --reps 16 --time 40 --device cuda:0 -o $E/eval_dr/$1.md > $E/eval_dr/$1.log 2>&1; }
ev I_on_vesc1124 $E/I_F_dr1/policy.pt &
python3 -u learned/rl/train.py --profile fast --device cuda:0 --epochs 3 --batch-size 65536 --actuator vesc --dr full \
  --curriculum none --dr-level 1.0 --start-spread 1 --dr-drop 0.03 --num-envs 65536 --n-steps 32 \
  --init $E/I_F_dr1/policy.pt --lr-start 2e-4 --det-eval-every 5 --sim-check --total-steps 40e6 --out $E/J_I_vesc1124 > $E/J_I_vesc1124.log 2>&1
wait
ev J_I_vesc1124 $E/J_I_vesc1124/policy.pt
echo JDONE
