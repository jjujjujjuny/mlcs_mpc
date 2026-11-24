#!/usr/bin/env bash
cd "$(dirname "$0")/../../.."
P=/opt/homebrew/Caskroom/miniforge/base/envs/mlcs_ros/bin/python
T=${T:-60e6}
run () { name=$1; shift; echo "=== $name $(date +%T)"; $P -u learned/rl/train.py --profile fast --total-steps $T "$@" --out logs/rl/exp/$name > logs/rl/exp/$name.log 2>&1; tail -2 logs/rl/exp/$name.log; }
run A_lag_cur   --actuator lag  --dr full --curriculum adaptive
run B_vesc_cur  --actuator vesc --dr full --curriculum adaptive
run C_none_cur  --actuator none --dr full --curriculum adaptive
run D_lag_fixed --actuator lag  --dr full --curriculum none
echo ALLDONE
