#!/usr/bin/env bash
# ============================================================================
#  bat.sh — VESC 로 배터리 잔량 확인
#
#  사용법:
#     ./bat.sh          한 번 보고 끝
#     ./bat.sh -w       2초마다 계속 (Ctrl+C 로 종료)
#
#  환경변수:
#     MLCS_WS=~/f1tenth_ws     워크스페이스 경로
#     MLCS_CELLS=3             셀 수 (기본 3S — 2026-11-08 실측 11.4V)
#
#  ★ 브링업(drive.sh)이 떠 있으면 그 /sensors/core 를 읽기만 한다.
#    안 떠 있으면 vesc_driver 를 잠깐 혼자 띄워서 읽고 내린다.
#    (시리얼은 배타적이라 둘을 동시에 띄우면 안 된다)
#
#  ★ 잔량(%)은 **쉬고 있을 때** 셀 전압 기준이다. 주행 중엔 전압이
#    처지므로 실제보다 낮게 나온다 — 판단은 세워 놓고 할 것.
# ============================================================================
set -u

WS="${MLCS_WS:-$HOME/f1tenth_ws}"
CELLS="${MLCS_CELLS:-3}"
WATCH=0
[ "${1:-}" = "-w" ] && WATCH=1

RED=$'\033[31m'; GRN=$'\033[32m'; YLW=$'\033[33m'; BLD=$'\033[1m'; RST=$'\033[0m'

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

LOG="${TMPDIR:-/tmp}/mlcs_bat.log"
DRV_PID=""
cleanup () {
    [ -n "$DRV_PID" ] && kill "$DRV_PID" 2>/dev/null && wait "$DRV_PID" 2>/dev/null
    DRV_PID=""
}
trap cleanup EXIT
trap 'echo; exit 130' INT TERM      # ★ exit 을 해야 -w 루프가 멈춘다 (EXIT 트랩이 정리)

if ! pgrep -f '/lib/vesc_driver/' >/dev/null 2>&1; then
    # ★ ros2 run 이 아니라 실행 파일을 직접 띄운다. ros2 run 을 kill 하면
    #   래퍼만 죽고 드라이버는 고아로 남아 시리얼을 계속 잡고 있다 (실측).
    "$WS/install/vesc_driver/lib/vesc_driver/vesc_driver_node" --ros-args \
        --params-file "$WS/src/f1tenth_system/f1tenth_stack/config/vesc.yaml" \
        -r __node:=vesc_bat >"$LOG" 2>&1 &
    DRV_PID=$!
fi

# 쉬고 있는 LiPo 셀 전압 → 잔량(%). 표 사이는 선형 보간.
soc () {
    awk -v v="$1" 'BEGIN {
        n = split("3.27 3.61 3.69 3.71 3.73 3.75 3.77 3.79 3.80 3.82 3.84 3.85 3.87 3.91 3.95 3.98 4.02 4.08 4.11 4.15 4.20", V, " ")
        split("0 5 10 15 20 25 30 35 40 45 50 55 60 65 70 75 80 85 90 95 100", P, " ")
        if (v <= V[1]) { print 0; exit }
        if (v >= V[n]) { print 100; exit }
        for (i = 1; i < n; i++)
            if (v < V[i+1]) { printf "%d", P[i] + (P[i+1]-P[i]) * (v-V[i]) / (V[i+1]-V[i]); exit }
    }'
}

read_voltage () {
    # 드라이버가 막 떴을 때 첫 메시지가 0 V 일 수 있어 0 이 아닐 때까지 본다
    local v
    for _ in 1 2 3 4 5; do
        v="$(timeout 5 ros2 topic echo /sensors/core --once --field state 2>/dev/null \
             | awk '/voltage_input:/ {print $2}')"
        if [ -n "$v" ] && awk -v v="$v" 'BEGIN{exit !(v > 1)}'; then
            echo "$v"; return 0
        fi
    done
    return 1
}

show () {
    local v cell pct bar color
    if ! v="$(read_voltage)"; then
        echo "${RED}✗ VESC 응답 없음${RST} — 전원·USB 확인 (로그: $LOG)"
        return 1
    fi
    cell="$(awk -v v="$v" -v n="$CELLS" 'BEGIN{printf "%.2f", v/n}')"
    pct="$(soc "$cell")"
    bar="$(awk -v p="$pct" 'BEGIN{for(i=0;i<20;i++) printf (i < p/5 ? "█" : "░")}')"
    if   [ "$pct" -ge 40 ]; then color="$GRN"
    elif [ "$pct" -ge 20 ]; then color="$YLW"
    else                         color="$RED"; fi
    printf '%s 배터리 %s%3d%%%s %s  %.2f V  (%sS, 셀당 %s V)\n' \
        "$(date +%H:%M:%S)" "$color$BLD" "$pct" "$RST" "$bar" "$v" "$CELLS" "$cell"

    # LiPo 는 셀당 3.3 V 아래로 내려가면 손상된다
    if awk -v c="$cell" 'BEGIN{exit !(c < 3.5)}'; then
        echo "  ${RED}${BLD}⚠ 셀당 3.5 V 미만 — 지금 충전하세요 (3.3 V 아래는 배터리 손상)${RST}"
    elif awk -v c="$cell" 'BEGIN{exit !(c > 4.25)}'; then
        echo "  ${YLW}⚠ 셀당 4.25 V 초과 — 셀 수가 맞나요? MLCS_CELLS=4 ./bat.sh${RST}"
    fi
}

if [ "$WATCH" = 1 ]; then
    while true; do show; sleep 2; done
else
    show
fi
