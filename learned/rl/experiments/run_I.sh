#!/usr/bin/env bash
# I: F (VESC 만, 가장 빠름) 에서 G 와 같은 DR 1.0 미세조정 40M → DR 평가. 끊기면 --resume logs/rl/exp/I_F_dr1 --device cuda:0
cd "$(dirname "$0")/../../.."
E=logs/rl/exp
python3 -u learned/rl/train.py --profile fast --device cuda:0 --epochs 3 --batch-size 65536 --actuator vesc --dr full \
  --curriculum none --dr-level 1.0 --start-spread 1 --dr-drop 0.03 --num-envs 65536 --n-steps 32 \
  --init $E/F_vesc_only/best_det.pt --lr-start 2e-4 --det-eval-every 5 --sim-check --total-steps 40e6 --out $E/I_F_dr1 > $E/I_F_dr1.log 2>&1
python3 -u learned/rl/eval_vec.py $E/I_F_dr1/best_det.pt --labels I_F_dr1 --actuators vesc --dr-scales 0 0.5 1 1.5 2 \
  --drop 0 0.05 --starts 8 --reps 16 --time 40 --device cuda:0 -o $E/eval_dr/I_F_dr1.md > $E/eval_dr/I_F_dr1.log 2>&1
echo IDONE
