#!/usr/bin/env bash
# DR 강건성 평가 — 정책마다 프로세스 하나 (GPU 공유), 조건 전부를 한 배치에 (DR 배율 5 × 끊김 2 × 시작 8 × 반복 16 = 1280 환경)
cd "$(dirname "$0")/../../.."
E=logs/rl/exp
ev () { lab=$1; ck=$2; python3 -u learned/rl/eval_vec.py $ck --labels $lab --actuators vesc --dr-scales 0 0.5 1 1.5 2 \
  --drop 0 0.05 --starts 8 --reps 16 --time 40 --device cuda:0 -o $E/eval_dr/$lab.md > $E/eval_dr/$lab.log 2>&1; }
ev A_lag $E/A_lag_cur/best.pt &
ev E_dr0.5 $E/E_vesc_fromA/best_det.pt &
ev F_vesc_only $E/F_vesc_only/best_det.pt &
ev G_dr1 $E/G_vesc_dr1/best_det.pt &
ev H_dr0 $E/H_vesc_dr0/best_det.pt &
wait
{ head -2 $E/eval_dr/A_lag.md; for f in A_lag E_dr0.5 F_vesc_only G_dr1 H_dr0; do sed -n '3,/^$/p' $E/eval_dr/$f.md | sed '/^$/d'; done
  echo; tail -1 $E/eval_dr/A_lag.md; } > $E/eval_dr.md
echo EVALDONE
