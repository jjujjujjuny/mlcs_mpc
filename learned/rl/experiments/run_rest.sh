#!/usr/bin/env bash
# 실험 B~E (A 는 run_all.sh 로 끝남). vesc 는 65536 환경 × 32 스텝 (MPS 커널 오버헤드를 큰 배치로 상쇄, 18k → 62k 스텝/s)
cd "$(dirname "$0")/../../.."
P=/opt/homebrew/Caskroom/miniforge/base/envs/mlcs_ros/bin/python
run () { name=$1; shift; echo "=== $name $(date +%T)"; $P -u learned/rl/train.py --profile fast "$@" --out logs/rl/exp/$name > logs/rl/exp/$name.log 2>&1; tail -2 logs/rl/exp/$name.log; }
run B_vesc_cur   --total-steps 60e6  --actuator vesc --dr full --curriculum adaptive --num-envs 65536 --n-steps 32
run C_none_cur   --total-steps 60e6  --actuator none --dr full --curriculum adaptive
run D_lag_fixed  --total-steps 60e6  --actuator lag  --dr full --curriculum none
run E_vesc_speed --total-steps 150e6 --actuator vesc --dr full --curriculum adaptive --dr-max 0.5 --start-spread 1 --dr-drop 0.03 --num-envs 65536 --n-steps 32
echo ALLDONE
