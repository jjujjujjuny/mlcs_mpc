#!/usr/bin/env bash
# 최종 영상 (2026-11-24) — run_L.sh·run_J.sh 가 끝난 뒤. casadi Simulator + VESC (11-24 설정), 정지 출발.
cd "$(dirname "$0")/../../.."
E=logs/rl/exp; V=logs/rl/video
while ! grep -q LDONE $E/run_L.out 2>/dev/null; do sleep 20; done
while ! grep -q JDONE $E/run_J.out 2>/dev/null; do sleep 20; done
r () { out=$1; shift; python3 learned/rl/render.py "$@" --sim vesc --laps 2 --max-time 50 -o $V/$out > $V/${out%.mp4}.log 2>&1 &
       }
r final_track1c_vesc.mp4 $E/L2_track1c/policy.pt $E/L1_track1c/best.pt --labels "L2 (VESC+DR, 최종)" "L1 (lag 학습)"
r final_track2a_vesc.mp4 $E/L2_track2a/policy.pt $E/L1_track2a/best.pt --labels "L2 (VESC+DR, 최종)" "L1 (lag 학습)"
r final_rl17_J_vs_I_vesc1124.mp4 $E/J_I_vesc1124/policy.pt $E/I_F_dr1/policy.pt --labels "J (VESC 11-24 학습)" "I (VESC 11-08 학습)"
wait
echo RENDERDONE
