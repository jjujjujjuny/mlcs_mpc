#!/usr/bin/env bash
# DR 실험 (PC, RTX 3060 — CUDA, G·H 를 동시에 — 한 프로세스로는 GPU 60 % 라 병렬이 더 빠르다). E (VESC, DR 0.5 고정) 가 끝나길 기다렸다가 E 의 마지막 정책에서 이어:
#   G_vesc_dr1  DR 수준 1.0 (full)   H_vesc_dr0  DR 없음 (대조군)   — 둘 다 40M, 같은 시작 조건·끊김 설정
# 그 다음 A·E·G·H 를 학습 전체 랜덤화 배율 0/0.5/1/1.5/2 × 명령 끊김 0/0.05 에서 평가 (vec/vesc, 시작 8 곳 × 반복 16).
# 끊기면: python3 learned/rl/train.py --resume logs/rl/exp/<이름> --device cuda:0
cd "$(dirname "$0")/../../.."
P=python3
while pgrep -f "resume logs/rl/exp/E_vesc_fromA" >/dev/null; do sleep 20; done
tail -1 logs/rl/exp/E_vesc_fromA.log
run () { name=$1; shift; echo "=== $name $(date +%T)"; $P -u learned/rl/train.py --profile fast --device cuda:0 --epochs 3 --batch-size 65536 \
  --actuator vesc --dr full --curriculum none --start-spread 1 --dr-drop 0.03 --num-envs 65536 --n-steps 32 \
  --init logs/rl/exp/E_vesc_fromA/policy.pt --lr-start 2e-4 --det-eval-every 5 --sim-check "$@" --out logs/rl/exp/$name > logs/rl/exp/$name.log 2>&1; tail -2 logs/rl/exp/$name.log; }
run G_vesc_dr1 --total-steps 40e6 --dr-level 1.0 &
run H_vesc_dr0 --total-steps 40e6 --dr-level 0.0 &
wait
while pgrep -f "resume logs/rl/exp/F_vesc_only" >/dev/null; do sleep 20; done   # F 도 끝난 뒤 평가 (GPU 공유)
echo "=== eval $(date +%T)"
E=logs/rl/exp
$P -u learned/rl/eval_vec.py $E/A_lag_cur/best.pt $E/E_vesc_fromA/best_det.pt $E/G_vesc_dr1/best_det.pt $E/H_vesc_dr0/best_det.pt $E/F_vesc_only/best_det.pt \
  --labels A_lag E_dr0.5 G_dr1 H_dr0 F_vesc_only --actuators vesc --dr-scales 0 0.5 1 1.5 2 --drop 0 0.05 \
  --starts 8 --reps 16 --time 40 --device cuda:0 -o $E/eval_dr.md > $E/eval_dr.log 2>&1
tail -1 $E/eval_dr.log
echo ALLDONE
