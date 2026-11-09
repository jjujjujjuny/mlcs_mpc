#!/usr/bin/env bash
# ============================================================================
#  laptop.sh — 노트북에서 젯슨 RViz 보기 (Zenoh 브리지 + RViz)
#
#  이 폴더(tools/zenoh)를 통째로 노트북에 복사해서 쓴다.
#
#  사용법:
#     ./laptop.sh                  젯슨 = mlcs-car.local
#     ./laptop.sh 192.168.0.7      IP 로 직접 (mDNS 가 안 될 때)
#
#  젯슨에서 먼저:  ./drive.sh bridge   (주행과 별도 창, tmux 안에서)
#  터미널은 따로: ssh -t mlcs@mlcs-car.local tmux new -A -s race
# ============================================================================
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST="${1:-mlcs-car.local}"
ZB="$HERE/x86_64/zenoh-bridge-ros2dds"

RED=$'\033[31m'; GRN=$'\033[32m'; YLW=$'\033[33m'; BLD=$'\033[1m'; RST=$'\033[0m'
log()  { echo "${BLD}[laptop]${RST} $*"; }
err()  { echo "${RED}[laptop] ✗ $*${RST}" >&2; }

[ -x "$ZB" ] || { err "브리지 실행 파일이 없습니다: $ZB"; exit 1; }
if [ -z "${ROS_DISTRO:-}" ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash || { err "ROS 2 Humble 이 없습니다"; exit 1; }
    set -u
fi
[ "${ROS_DISTRO:-}" = humble ] || \
    echo "${YLW}[laptop] ⚠ ROS_DISTRO=${ROS_DISTRO:-?} — 젯슨은 humble 이다. 메시지가 안 맞을 수 있다${RST}"

# 노트북 RViz 와 브리지는 노트북 안(localhost)에서만 만난다
export ROS_LOCALHOST_ONLY=1

if ! ping -c1 -W2 "$HOST" >/dev/null 2>&1; then
    err "젯슨($HOST)에 닿지 않습니다 — 같은 공유기에 붙었는지, 이름 대신 IP 로:"
    err "  ./laptop.sh 192.168.x.x     (젯슨에서 ./drive.sh bridge 가 주소를 보여준다)"
    exit 1
fi

BR_PID=""
cleanup () { [ -n "$BR_PID" ] && kill "$BR_PID" 2>/dev/null; }
trap cleanup EXIT
trap 'exit 130' INT TERM

LOG="${TMPDIR:-/tmp}/mlcs_laptop_bridge.log"
"$ZB" -c "$HERE/laptop.json5" -e "tcp/$HOST:7447" >"$LOG" 2>&1 &
BR_PID=$!
sleep 2
kill -0 "$BR_PID" 2>/dev/null || { err "브리지가 바로 죽었습니다 — $LOG"; tail -5 "$LOG"; exit 1; }
log "브리지 → ${GRN}tcp/$HOST:7447${RST}  (로그: $LOG)"
log "RViz — Fixed Frame=map. 2D Pose Estimate 로 찍으면 젯슨으로 간다"
rviz2 -d "$HERE/laptop.rviz"
