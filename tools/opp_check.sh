#!/usr/bin/env bash
# ============================================================================
#  opp_check.sh — 추월 로직 오프라인 검증 (실제 솔버 + 가상 상대)
#
#     ./tools/opp_check.sh              전부 (현실 상대 표 + 안전 시험), 몇 분
#     ./tools/opp_check.sh quick        트랙 하나만
#     OPP_EXTRA="--recover-reverse" ./tools/opp_check.sh   offline_ocp 에 인자를 더 준다
#
#  acados 필요: 젯슨, 또는 ~/acados 를 설치한 메인 PC/노트북.
#  가상 상대는 레이싱 라인을 따라 코너에서 줄이고 (√(1.5/|κ|)), 라인이 ±0.15 m 흔들린다.
#  가상 상대는 우리를 피하지 않는다 → '뒤박힘' 은 서 있는 우리를 상대가 뒤에서 박은 것.
#  기록은 data/opp_check_<시각>/ 에 남는다.
# ============================================================================
cd "$(dirname "$0")/.." || exit 1
[ -d ~/acados/lib ] && export ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR:-~/acados} LD_LIBRARY_PATH=~/acados/lib:$LD_LIBRARY_PATH
OUT=data/opp_check_$(date +%m%d_%H%M); mkdir -p "$OUT"
TRACKS="1108_7 1109_3"; [ "$1" = quick ] && TRACKS="1108_7"

base() {  # 트랙
  echo "--scenario track --plant --params-yaml src/mlcs_mpc/config/hmpc.yaml --wheel-speed-tau 0.4 --plant-ws-tau 0.4
        --track lab_map/data/tracks/$1_track/prep_$1_track.csv --walls 0.22 --track-hard 0.5 --track-slack 1000
        --N 60 --v-max 2.0 ${OPP_EXTRA:-}"
}

summ() {  # 기록 파일 → 한 줄
  r=$(cat "$1")
  st=$(echo "$r" | grep -o "status .*" | head -1 | grep -o "4: [0-9]*" | cut -c4-)
  hit=$(echo "$r" | grep -o "겹침(충돌) [0-9]*" | grep -o "[0-9]*$")
  rear=$(echo "$r" | grep -o "뒤에서 박은 것 [0-9]*" | grep -o "[0-9]*$")
  printf "추월 %2s | 닿음 %3s 뒤박힘 %3s | 최소 %4s m | 벽밖 %5s | 풀이실패 %2s | 뒤따름정지 %2s | %s | GP %s\n" \
    "$(echo "$r" | grep -o "추월 [0-9]*번" | grep -o "[0-9]*")" "$(( ${hit:-0} - ${rear:-0} ))" "${rear:-0}" \
    "$(echo "$r" | grep -o "최소 중심 거리 [0-9.inf]*" | grep -o "[0-9.inf]*$")" \
    "$(echo "$r" | grep -o "벽 경계 밖 [0-9.]*%" | grep -o "[0-9.]*%")" "${st:-0}" \
    "$(echo "$r" | grep -o "뒤따름 정지 [0-9]*" | grep -o "[0-9]*$")" \
    "$(echo "$r" | grep -o "우리 [0-9.]* m/s · 상대 [0-9.]*" ) m/s" \
    "$(echo "$r" | grep -o "GP 사용 [0-9]*/[0-9]*" | grep -o "[0-9]*/[0-9]*")"
}

run() {  # 이름 트랙 인자...
  n=$1; T=$2; shift 2
  # shellcheck disable=SC2046
  python3 hypermpc/offline_ocp.py $(base "$T") "$@" > "$OUT/$n.log" 2>&1
}

jobs_max=$(( $(nproc) > 2 ? $(nproc) - 1 : 1 ))
throttle() { while [ "$(jobs -rp | wc -l)" -ge "$jobs_max" ]; do sleep 0.5; done; }

# 솔버를 한 번 먼저 빌드 (동시에 빌드하면 꼬인다)
run _build 1108_7 --laps 2 || { echo "✗ 빌드 실패: $OUT/_build.log"; tail -5 "$OUT/_build.log"; exit 1; }

names=()
for T in $TRACKS; do
  # ① 현실 상대: 속도 2 × 위치 3 × GP 켬/끔, 60 초
  for v in 1.0 1.4; do for d in -0.25 0.0 0.25; do for g in gp nogp; do
    n="${T}_v${v}_d${d}_${g}"; names+=("$n"); throttle
    run "$n" "$T" --laps 1800 --opponent "$v" --opp-d "$d" --opp-alat 1.5 --opp-weave 0.15 --opp-gap 3 \
        $( [ $g = nogp ] && echo --opp-no-gp ) &
  done; done; done
  # ② 안전: 한가운데 정지 장애물 (못 지나가면 부딪히지 말고 서야 한다), 30 초
  for gap in 3 7; do
    n="${T}_정지_가운데_${gap}m"; names+=("$n"); throttle
    run "$n" "$T" --laps 900 --opponent 0 --opp-d 0 --opp-gap "$gap" &
  done
done
wait

echo; echo "######## 요약 (이걸 복사) ########"
for n in "${names[@]}"; do printf "%-28s %s\n" "$n" "$(summ "$OUT/$n.log")"; done
echo; echo "기록: $OUT/"
