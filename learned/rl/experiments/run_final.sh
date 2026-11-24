#!/usr/bin/env bash
# E (A 에서 출발, VESC 환경, 빠른 1 ms 단계) → D. 끊기면: python learned/rl/train.py --resume logs/rl/exp/<이름>
cd "$(dirname "$0")/../../.."
P=/opt/homebrew/Caskroom/miniforge/base/envs/mlcs_ros/bin/python
run () { name=$1; shift; echo "=== $name $(date +%T)"; $P -u learned/rl/train.py --profile fast --epochs 3 --batch-size 65536 "$@" --out logs/rl/exp/$name > logs/rl/exp/$name.log 2>&1; tail -2 logs/rl/exp/$name.log; }
run E_vesc_fromA --total-steps 120e6 --actuator vesc --dr full --curriculum none --dr-level 0.5 --start-spread 1 --dr-drop 0.03 --num-envs 65536 --n-steps 32 --init logs/rl/exp/A_lag_cur/best.pt --lr-start 3e-4 --det-eval-every 10 --sim-check
run D_lag_fixed  --total-steps 40e6 --actuator lag --dr full --curriculum none --det-eval-every 10 --sim-check
echo ALLDONE
