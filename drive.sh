#!/usr/bin/env bash
# ============================================================================
#  drive.sh — 실차 딸깍 실행 스크립트
#
#  사용법:
#     ./drive.sh joy                 조이스틱 수동 조종 (+ 차량 브링업)
#     ./drive.sh bench               거치대 위 조향 확인 (속도 0 고정)
#     ./drive.sh mpc <경로.csv>      MPC 자율주행 (조이스틱 병행 — LB 로 전환)
#     ./drive.sh record <이름>       조이스틱으로 몰면서 웨이포인트 기록
#     ./drive.sh cal <모드> [값]     캘리브레이션 (neutral|speed|steer)
#     ./drive.sh mocap               mocap 위치추정만 (연동 확인용)
#     ./drive.sh topics              토픽 상태 점검
#     ./drive.sh stop                비상 정지 + 전부 종료
#
#  환경변수:
#     MLCS_WS=~/f1tenth_ws          워크스페이스 경로
#     MLCS_SPEED=1.5                수동 조종 최고 속도 (m/s)
#     MLCS_NO_BRINGUP=1             브링업을 안 띄운다 (이미 떠 있을 때)
#     MLCS_DEBUG=1                  조이스틱 값 출력
# ============================================================================
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${MLCS_WS:-$HOME/f1tenth_ws}"
SPEED="${MLCS_SPEED:-1.5}"
DEBUG="${MLCS_DEBUG:-0}"

RED=$'\033[31m'; GRN=$'\033[32m'; YLW=$'\033[33m'; BLD=$'\033[1m'; RST=$'\033[0m'
log()  { echo "${BLD}[drive]${RST} $*"; }
warn() { echo "${YLW}[drive] ⚠ $*${RST}"; }
err()  { echo "${RED}[drive] ✗ $*${RST}" >&2; }

# ── ROS 환경 ────────────────────────────────────────────────────────────────
# ★ help 는 ROS 없이도 볼 수 있어야 한다. 개발 PC 에서 사용법만 확인하거나,
#   설치가 덜 된 젯슨에서 "무엇부터 해야 하나" 를 보려는 상황이 실제로 있다.
#   여기서 exit 하면 그때 아무것도 못 본다.
load_ros () {
    [ -f /opt/ros/humble/setup.bash ] || {
        err "ROS 2 Humble 이 없습니다 (/opt/ros/humble)"
        err "  설치: docs/SETUP_JETSON.md 1절"
        exit 1
    }
    # ★ set +u 로 감싼다 — ROS 의 setup.bash 는 미설정 변수를 참조한다
    #   (AMENT_TRACE_SETUP_FILES, COLCON_TRACE 등). 이 스크립트는 오타로
    #   인한 사고를 막으려고 set -u 를 켜두는데, 그 상태로 source 하면
    #       /opt/ros/humble/setup.bash: AMENT_TRACE_SETUP_FILES: unbound variable
    #   로 죽는다. ROS 쪽 파일이라 우리가 고칠 수 없으므로 이 구간만 끈다.
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    if [ -f "$WS/install/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "$WS/install/setup.bash"
        set -u
    else
        set -u
        err "워크스페이스가 빌드되지 않았습니다: $WS/install/setup.bash"
        err "  cd $WS && colcon build --symlink-install"
        err "  (다른 위치면 MLCS_WS=... ./drive.sh ...)"
        exit 1
    fi
}

# ── ★ bringup 의 joy_teleop 을 죽인다 ────────────────────────────────────────
#
#   f1tenth_stack 의 bringup_launch.py 는 **자기 joy_teleop 을 같이 띄운다.**
#   그게 우리 joystick_teleop 과 **같은 /teleop 토픽**에 발행하는데, 설정이
#   이렇다 (f1tenth_stack/config/joy_teleop.yaml):
#
#       human_control:
#         deadman_buttons: [4]        ← LB. 우리가 모드 토글로 쓰는 그 버튼
#         drive-speed: {axis: 1, scale: 5.0}
#
#   즉 LB 를 누른 채 왼쪽 스틱을 위로 밀면 **5 m/s 명령이 나간다.** 우리는
#   LB 를 수동↔자율 토글로 쓰므로, 모드를 바꾸려고 누를 때마다 저쪽이 같이
#   반응한다. 게다가 노드가 죽는 순간 VESC 에 마지막 명령이 얼어붙는다 —
#   VESC 는 새 명령이 없으면 마지막 속도를 유지하기 때문이다.
#
#   26-jetson 이 이 문제로 실제 폭주를 겪었고, launch 가 언제 띄울지 모르므로
#   "한 번 죽이기" 로는 안 되고 기동 구간 전체를 덮어야 했다.
kill_bringup_joy () {
    for _ in $(seq 1 20); do          # 0.4s × 20 = 8초
        pkill -9 -f '[j]oy_teleop' 2>/dev/null || true
        sleep 0.4
    done
}

# 우리 노드는 이름이 달라서( mlcs_mpc 의 joystick_teleop ) 위 패턴에 안 걸린다.
# 확인: pkill -f '[j]oy_teleop' 는 'joystick_teleop' 에 매칭되지 않는다
#       ('joy_teleop' 이 'joystick_teleop' 의 부분문자열이 아니다).

stop_all () {
    log "정지 명령 발행 중..."
    # ★ 한 번이 아니라 반복해서 보낸다. VESC 는 마지막 명령을 유지하므로
    #   "안 보내는 것" 으로는 안 선다.
    for _ in $(seq 1 10); do
        ros2 topic pub --once /drive ackermann_msgs/msg/AckermannDriveStamped \
            '{drive: {speed: 0.0, steering_angle: 0.0}}' >/dev/null 2>&1 || true
        ros2 topic pub --once /teleop ackermann_msgs/msg/AckermannDriveStamped \
            '{drive: {speed: 0.0, steering_angle: 0.0}}' >/dev/null 2>&1 || true
        sleep 0.1
    done
    ros2 topic pub --once /mpc/enabled std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1 || true
}

cleanup () {
    echo
    log "종료 중 — 정지 명령을 보냅니다"
    stop_all
    for p in ${PIDS:-}; do kill "$p" 2>/dev/null || true; done
    sleep 0.5
    for p in ${PIDS:-}; do kill -9 "$p" 2>/dev/null || true; done
}

PIDS=""
start_bringup () {
    if [ "${MLCS_NO_BRINGUP:-0}" = "1" ]; then
        log "브링업 생략 (MLCS_NO_BRINGUP=1)"
        return
    fi
    log "차량 브링업 (VESC + mux)..."
    ros2 launch f1tenth_stack bringup_launch.py >/tmp/mlcs_bringup.log 2>&1 &
    PIDS="$PIDS $!"
    kill_bringup_joy &               # ★ launch 와 동시에 시작 (위 주석 참고)
    PIDS="$PIDS $!"
    sleep 3
}

require_ws_pkg () {
    ros2 pkg list 2>/dev/null | grep -qx "$1" || {
        err "패키지 '$1' 를 찾을 수 없습니다. 빌드했나요?"
        err "  cd $WS && colcon build --symlink-install && source install/setup.bash"
        exit 1
    }
}

# ── 모드 ────────────────────────────────────────────────────────────────────
MODE="${1:-help}"
shift 2>/dev/null || true

case "$MODE" in

joy)
    load_ros; require_ws_pkg mlcs_mpc
    trap cleanup EXIT INT TERM
    start_bringup
    log "조이스틱 수동 조종  ${GRN}max_speed=${SPEED} m/s${RST}"
    echo
    echo "    RT 당김        전진 (놓으면 즉시 정지)"
    echo "    RB + RT        후진"
    echo "    L스틱          조향"
    echo "    START          비상 정지"
    echo
    warn "기동 직후 RT 에서 손을 떼세요 — 트리거 휴지값을 자동으로 잽니다"
    echo
    # joy:=false — bringup 이 joy_node 를 이미 띄웠다
    JOYARG="false"; [ "${MLCS_NO_BRINGUP:-0}" = "1" ] && JOYARG="true"
    ros2 launch mlcs_mpc joystick.launch.py \
        joy:="$JOYARG" max_speed:="$SPEED" allow_toggle:=false debug:="$([ "$DEBUG" = 1 ] && echo true || echo false)"
    ;;

bench)
    load_ros; require_ws_pkg mlcs_mpc
    trap cleanup EXIT INT TERM
    start_bringup
    log "${YLW}거치대 모드${RST} — 속도는 항상 0. 조향만 움직입니다."
    warn "바퀴가 땅에 닿지 않는지 확인하세요."
    echo
    JOYARG="false"; [ "${MLCS_NO_BRINGUP:-0}" = "1" ] && JOYARG="true"
    ros2 launch mlcs_mpc joystick.launch.py \
        joy:="$JOYARG" max_speed:=0.0 allow_toggle:=false debug:=true
    ;;

mpc)
    WP="${1:-}"
    [ -n "$WP" ] || { err "웨이포인트 파일이 필요합니다:  ./drive.sh mpc <경로.csv>"; exit 1; }
    [ -f "$WP" ] || { err "파일이 없습니다: $WP"; exit 1; }
    WP="$(cd "$(dirname "$WP")" && pwd)/$(basename "$WP")"   # 절대경로
    load_ros; require_ws_pkg mlcs_mpc
    trap cleanup EXIT INT TERM
    start_bringup

    log "MPC 자율주행  웨이포인트: ${GRN}$WP${RST}"
    ros2 launch mlcs_mpc car_mpc.launch.py \
        waypoints:="$WP" target_speed:="$SPEED" >/tmp/mlcs_mpc.log 2>&1 &
    PIDS="$PIDS $!"
    sleep 2

    echo
    warn "차는 아직 출발하지 않습니다. LB 로 자율 전환하세요."
    echo
    echo "    LB             수동 ↔ 자율 전환"
    echo "    START          비상 정지"
    echo "    RT/L스틱       수동 조종 (자율보다 우선)"
    echo
    log "MPC 로그: tail -f /tmp/mlcs_mpc.log"
    echo
    JOYARG="false"; [ "${MLCS_NO_BRINGUP:-0}" = "1" ] && JOYARG="true"
    ros2 launch mlcs_mpc joystick.launch.py \
        joy:="$JOYARG" max_speed:="$SPEED" allow_toggle:=true debug:="$([ "$DEBUG" = 1 ] && echo true || echo false)"
    ;;

record)
    NAME="${1:-track_$(date +%Y%m%d_%H%M%S)}"
    OUT="$SCRIPT_DIR/src/mlcs_mpc/waypoints/${NAME}.csv"
    load_ros; require_ws_pkg mlcs_mpc
    trap cleanup EXIT INT TERM
    start_bringup

    log "웨이포인트 기록 → ${GRN}$OUT${RST}"
    ros2 run mlcs_mpc waypoint_logger --ros-args \
        -p output:="$OUT" >/tmp/mlcs_wp.log 2>&1 &
    PIDS="$PIDS $!"
    sleep 1
    echo
    warn "조이스틱으로 트랙을 한 바퀴 돈 뒤 Ctrl+C 로 저장하세요."
    echo
    JOYARG="false"; [ "${MLCS_NO_BRINGUP:-0}" = "1" ] && JOYARG="true"
    ros2 launch mlcs_mpc joystick.launch.py \
        joy:="$JOYARG" max_speed:="$SPEED" allow_toggle:=false

    # 조이스틱이 끝나면 로거를 정리하고 평활화를 안내한다
    sleep 1
    if [ -f "$OUT" ]; then
        echo
        log "저장됨: $OUT"
        log "다음 — ${BLD}평활화를 건너뛰지 마세요${RST} (곡률이 튀면 속도가 들쭉날쭉합니다):"
        echo "    ros2 run mlcs_mpc smooth_path --ros-args \\"
        echo "        -p input:=$OUT -p output:=${OUT%.csv}_smooth.csv"
    fi
    ;;

cal)
    CMODE="${1:-}"
    CVAL="${2:-}"
    case "$CMODE" in
        neutral) ARGS="-p mode:=neutral" ;;
        speed)   ARGS="-p mode:=speed -p value:=${CVAL:-1.5}" ;;
        steer)   ARGS="-p mode:=steer -p value:=${CVAL:-0.2}" ;;
        *) err "사용법: ./drive.sh cal neutral|speed|steer [값]"
           echo "  순서가 중요합니다 — docs/CALIBRATION.md 참고:"
           echo "    ./drive.sh cal neutral         ① 조향 중립"
           echo "    ./drive.sh cal speed 1.5       ② 속도 게인"
           echo "    ./drive.sh cal steer 0.2       ③ 조향 게인 (좌우 양쪽)"
           exit 1 ;;
    esac
    load_ros; require_ws_pkg mlcs_mpc
    trap cleanup EXIT INT TERM
    start_bringup

    log "mocap 브릿지..."
    ros2 run mlcs_mpc mocap_bridge --ros-args \
        --params-file "$SCRIPT_DIR/src/mlcs_mpc/config/mocap.yaml" \
        >/tmp/mlcs_mocap.log 2>&1 &
    PIDS="$PIDS $!"
    sleep 2

    if ! timeout 5 ros2 topic echo /mpc/state --once >/dev/null 2>&1; then
        err "/mpc/state 가 오지 않습니다 — mocap 연동을 먼저 확인하세요."
        err "  ./drive.sh mocap   으로 점검하고 docs/SETUP_JETSON.md 5절 참고"
        exit 1
    fi

    echo
    warn "${BLD}킬스위치를 손에 드세요. 차가 움직입니다.${RST}"
    [ "$CMODE" = "steer" ] && warn "steer 모드는 차가 원을 그리며 돕니다 — 넓은 공간에서."
    echo
    # shellcheck disable=SC2086
    ros2 run mlcs_mpc calibrate --ros-args $ARGS
    ;;

mocap)
    load_ros; require_ws_pkg mlcs_mpc
    trap cleanup EXIT INT TERM
    log "mocap 위치추정만 실행 (차량 브링업 없음)"
    ros2 run mlcs_mpc mocap_bridge --ros-args \
        --params-file "$SCRIPT_DIR/src/mlcs_mpc/config/mocap.yaml" &
    PIDS="$PIDS $!"
    sleep 2
    echo
    log "차를 손으로 옮기며 값이 맞는지 확인하세요:"
    echo "    ros2 topic echo /mpc/state --field pose.pose.position"
    echo "    ros2 topic hz   /mpc/state        # 100Hz 근처"
    echo
    wait
    ;;

topics)
    load_ros
    echo
    log "토픽 점검"
    echo
    printf '  %-24s %s\n' "토픽" "상태"
    printf '  %-24s %s\n' "────────────────────────" "──────────────"
    for t in /joy /teleop /drive /mpc/state /mocap/valid /odom /sensors/imu; do
        if ros2 topic list 2>/dev/null | grep -qx "$t"; then
            HZ=$(timeout 3 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' | awk '{printf "%.0f Hz", $3}')
            [ -n "$HZ" ] && printf '  %-24s %s%s%s\n' "$t" "$GRN" "$HZ" "$RST" \
                         || printf '  %-24s %s(발행 없음)%s\n' "$t" "$YLW" "$RST"
        else
            printf '  %-24s %s없음%s\n' "$t" "$RED" "$RST"
        fi
    done
    echo
    echo "  노드:"
    ros2 node list 2>/dev/null | sed 's/^/    /'
    echo
    ;;

stop)
    load_ros
    log "${RED}비상 정지${RST}"
    stop_all
    pkill -f 'joystick_teleop' 2>/dev/null || true
    pkill -f 'mlcs_mpc'        2>/dev/null || true
    pkill -f 'bringup_launch'  2>/dev/null || true
    pkill -9 -f '[j]oy_teleop' 2>/dev/null || true
    log "종료 완료"
    ;;

help|--help|-h|*)
    sed -n '3,20p' "$0" | sed 's/^# \{0,1\}//'
    ;;
esac
