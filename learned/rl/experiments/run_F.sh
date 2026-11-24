#!/usr/bin/env bash
# F: VESC 만으로 학습한 계열 — B (VESC 처음부터 60M) 에서 이어 120M (E 와 같은 조건). 끊기면 --resume logs/rl/exp/F_vesc_only
cd "$(dirname "$0")/../../.."
P=/opt/homebrew/Caskroom/miniforge/base/envs/mlcs_ros/bin/python
run () { name=$1; shift; echo "=== $name $(date +%T)"; $P -u learned/rl/train.py --profile fast --epochs 3 --batch-size 65536 "$@" --out logs/rl/exp/$name > logs/rl/exp/$name.log 2>&1; tail -2 logs/rl/exp/$name.log; }
run F_vesc_only --total-steps 120e6 --actuator vesc --dr full --curriculum none --dr-level 0.5 --start-spread 1 --dr-drop 0.03 --num-envs 65536 --n-steps 32 --init logs/rl/exp/B_vesc_cur/policy.pt --lr-start 3e-4 --det-eval-every 10 --sim-check
echo ALLDONE
