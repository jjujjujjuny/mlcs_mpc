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
#     ./drive.sh log [태그]          조이스틱 주행 + HyperPM 학습 데이터 기록
#     ./drive.sh mocap               mocap 위치추정만 (연동 확인용)
#     ./drive.sh mocapcal [spin|straight]  마커 오프셋 측정
#     ./drive.sh topics              토픽 상태 점검
#     ./drive.sh stop                비상 정지 + 전부 종료
#
#  환경변수:
#     MLCS_WS=~/f1tenth_ws          워크스페이스 경로
#     MLCS_SPEED=1.5                수동 조종 최고 속도 (m/s)
#     MLCS_NO_BRINGUP=1             브링업을 안 띄운다 (이미 떠 있을 때)
#     MLCS_DEBUG=1                  조이스틱 값 출력
#     MLCS_DATA=~/mlcs_data         데이터 로그 저장 폴더
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

# ── joy_node 를 누가 띄우는가 ────────────────────────────────────────────────
#
#   ★ 2026-09-14 젯슨 실측: bringup_launch.py 가 joy_node 를 **안 띄운다.**
#     ros2 node list →  /ackermann_mux /ackermann_to_vesc_node /vesc_driver_node
#                       /vesc_to_odom_node /static_baselink_to_laser
#     joy_node 도 joy_teleop 도 없었고, 그래서 /joy 가 아예 발행되지 않아
#     조이스틱을 움직여도 서보가 안 움직였다.
#
#     원래는 bringup 이 띄운다고 보고 joy:=false 를 넘겼는데(중복 기동 방지),
#     그 전제가 이 환경에서는 틀렸다. humble-devel 의 구성이 다르거나,
#     라이다를 뺄 때 함께 빠졌을 수 있다.
#
#   그래서 **실제로 떠 있는지 보고 정한다.** 추측하지 않는다.
#   bringup 이 joy_node 를 띄우는 환경으로 바뀌어도 그대로 동작한다.
#   ★★ 2026-09-14 추가 — `ros2 node list` 를 믿으면 안 된다.
#
#     ros2 CLI 는 **데몬(ros2-daemon)의 그래프 캐시**를 읽는다. 이 캐시는
#     노드가 죽어도 곧바로 비워지지 않는다. 실측:
#
#       19:41  joy_node 를 kill
#       19:42  ros2 node list  →  여전히 /joy 가 보임   ← 죽은 지 1분
#              그래서 decide_joyarg 가 joy:=false 를 넘김
#              → joy_node 가 아무도 안 띄움 → /joy 발행 0
#              → 조이스틱을 아무리 움직여도 서보가 안 움직인다
#
#     증상이 "조이스틱이 VESC 로 안 넘어간다" 인데 로그에는 아무 에러가
#     없다. 앞서 66번 줄의 '실측' 도 이 착시였을 가능성이 크다.
#
#     그래서 데몬이 아니라 **프로세스 실체**를 본다. pgrep 은 캐시가 없다.
joy_node_running () {
    pgrep -f '/lib/joy/joy_node' >/dev/null 2>&1
}

# ── 조이스틱이 실제로 살아 있는지 확인 ───────────────────────────────────────
#   joy_node 가 떴다고 /joy 가 나온다는 보장은 없다 (권한, 장치 분리 등).
#   조용히 실패하면 "차가 반응을 안 하는데 에러는 없는" 상태가 된다.
warn_if_no_joy () {
    ( sleep 6
      if ! timeout 4 ros2 topic echo /joy --once >/dev/null 2>&1; then
          warn "${BLD}/joy 가 발행되지 않습니다 — 조이스틱이 먹지 않습니다.${RST}"
          warn "  확인:  ls -l /dev/input/js0   그리고   ros2 topic hz /joy"
      fi
    ) &
    PIDS="$PIDS $!"
}

# bringup 뒤에 호출 — joy_node 가 없으면 "true"(우리가 띄운다) 를 준다
decide_joyarg () {
    if [ "${MLCS_NO_BRINGUP:-0}" = "1" ]; then echo "true"; return; fi
    if joy_node_running; then
        echo "false"                      # bringup 이 이미 띄웠다
    else
        echo "true"                       # 아무도 안 띄웠다 → 우리가 띄운다
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

# ── ★ pkill -f 는 자기 자신의 부모 셸까지 잡는다 ────────────────────────────
#
#   원래 stop 모드는 `pkill -f 'mlcs_mpc'` 를 썼다. 그런데 이 스크립트의
#   경로가 **/home/mlcs/mlcs_mpc/drive.sh** 다. 절대경로로 실행하면
#   (`~/mlcs_mpc/drive.sh stop`) 명령줄에 'mlcs_mpc' 가 들어가므로 패턴에
#   걸린다. pkill 은 자기 자신은 안 죽이지만 **부모 셸은 죽인다** — 그래서
#   뒤에 남은 정리 단계가 실행되지 않은 채 끝나 버린다.
#
#   패턴을 실행 파일 경로로 좁히고, 자기 PID 와 부모 PID 는 건너뛴다.
kill_matching () {
    local pat="$1" sig="${2:--TERM}" p
    for p in $(pgrep -f "$pat" 2>/dev/null); do
        [ "$p" = "$$" ] && continue
        [ "$p" = "${PPID:-0}" ] && continue
        kill "$sig" "$p" 2>/dev/null || true
    done
}

stop_all () {
    log "정지 명령 발행 중..."
    # ★ 한 번이 아니라 반복해서 보낸다. VESC 는 마지막 명령을 유지하므로
    #   "안 보내는 것" 으로는 안 선다.
    # 셋을 **병렬로** 쏜다. ros2 CLI 는 기동에만 2초쯤 걸려서, 순차로 하면
    # 종료가 10초를 넘긴다 (실측). 정지는 빠를수록 좋다.
    pub_stop /drive  ackermann_msgs/msg/AckermannDriveStamped \
        '{drive: {speed: 0.0, steering_angle: 0.0}}' & _s1=$!
    pub_stop /teleop ackermann_msgs/msg/AckermannDriveStamped \
        '{drive: {speed: 0.0, steering_angle: 0.0}}' & _s2=$!
    pub_stop /mpc/enabled std_msgs/msg/Bool '{data: false}' & _s3=$!
    # ★ 인자 없는 `wait` 는 브링업 launch 등 **모든** 백그라운드 잡을 기다린다
    #   — 그러면 종료가 영영 안 끝난다. 반드시 PID 를 지정한다.
    wait "$_s1" "$_s2" "$_s3" 2>/dev/null || true
}

# ── ★ 정지 명령은 '절대 멈추지 않는' 방식으로 보내야 한다 ────────────────────
#
#   2026-09-14 젯슨: Ctrl+C 로 종료가 안 되는 문제의 원인이 여기였다.
#
#   `ros2 topic pub --once` 는 **구독자가 나타날 때까지 무한히 기다린다.**
#   Humble 의 기본값이 그렇다:
#       -w, --wait-matching-subscriptions
#           Defaults to 1 when using "-1"/"--once"/"--times"
#
#   그리고 Ctrl+C 는 포그라운드 **프로세스 그룹 전체**에 SIGINT 를 보내므로,
#   cleanup 이 도는 시점에는 정지 명령을 받아줄 mux 가 이미 죽는 중이다.
#   구독자 0 → pub 이 "Waiting for at least 1 matching subscription(s)..."
#   에서 영영 멈춤 → cleanup 이 아래 kill 까지 **도달하지 못한다.**
#
#   실제로 이 좀비가 7분 넘게 남아 있는 걸 확인했다:
#       ros2 topic pub --once /drive ...        (19:20 기동, 19:27 까지 생존)
#       ros2 topic pub --once /mpc/enabled ...  (19:18 기동)
#
#   그래서 두 가지를 건다:
#     -w 0      구독자를 기다리지 않는다 (없으면 그냥 쏘고 끝낸다)
#     timeout   그래도 막히면 3초에 끊는다
#   -t 20 -r 20 = 1초 동안 20번. 기존의 'seq 1 10 + sleep 0.1' 과 같은 양인데
#   멈출 수가 없다.
pub_stop () {
    timeout 3 ros2 topic pub -t 20 -r 20 -w 0 "$1" "$2" "$3" >/dev/null 2>&1 || true
}

# ★ trap 이 INT 와 EXIT 양쪽에 걸려 있어 cleanup 은 두 번 불린다
#   (INT 로 한 번, 그 뒤 스크립트가 끝나며 EXIT 로 또 한 번).
#   정지 명령을 두 번 쏘는 건 무해하지만 종료가 그만큼 늦어진다.
_CLEANED=0
cleanup () {
    [ "$_CLEANED" = "1" ] && return
    _CLEANED=1
    echo
    log "종료 중 — 정지 명령을 보냅니다"
    stop_all
    for p in ${PIDS:-}; do kill "$p" 2>/dev/null || true; done
    # 브링업은 별도 프로세스 그룹이라 Ctrl+C 가 닿지 않는다 — 직접 죽인다
    [ -n "${BRINGUP_PGID:-}" ] && kill -TERM "-$BRINGUP_PGID" 2>/dev/null || true
    sleep 1
    for p in ${PIDS:-}; do kill -9 "$p" 2>/dev/null || true; done
    [ -n "${BRINGUP_PGID:-}" ] && kill -9 "-$BRINGUP_PGID" 2>/dev/null || true
}

PIDS=""
BRINGUP_PGID=""
start_bringup () {
    if [ "${MLCS_NO_BRINGUP:-0}" = "1" ]; then
        log "브링업 생략 (MLCS_NO_BRINGUP=1)"
        return
    fi
    log "차량 브링업 (VESC + mux)..."
    # ★ setsid — 브링업을 **별도 프로세스 그룹**으로 띄운다.
    #
    #   Ctrl+C 는 포그라운드 프로세스 그룹 전체에 SIGINT 를 보낸다. 그냥 '&'
    #   로 띄우면 mux/vesc_driver 가 스크립트와 **동시에** 죽기 시작한다.
    #   그러면 cleanup 이 보내는 정지 명령을 받을 노드가 남지 않는다 —
    #   즉 "종료할 때 차를 세운다" 는 이 스크립트의 핵심 보장이 깨진다.
    #   VESC 는 마지막 명령을 유지하므로, 전달 경로가 죽은 채로 종료하면
    #   차는 마지막 속도로 계속 간다.
    #
    #   그래서 브링업은 SIGINT 경로 밖에 두고, cleanup 이 ① 정지 명령을
    #   먼저 보내고 ② 그 다음에 프로세스 그룹째로 죽인다.
    setsid ros2 launch f1tenth_stack bringup_launch.py >/tmp/mlcs_bringup.log 2>&1 &
    BRINGUP_PID=$!
    PIDS="$PIDS $BRINGUP_PID"
    kill_bringup_joy &               # ★ launch 와 동시에 시작 (위 주석 참고)
    PIDS="$PIDS $!"
    sleep 3
    BRINGUP_PGID="$(ps -o pgid= -p "$BRINGUP_PID" 2>/dev/null | tr -d ' ')"
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
    JOYARG="$(decide_joyarg)"
    [ "$JOYARG" = "true" ] && log "joy_node 가 없어서 직접 띄웁니다"
    warn_if_no_joy
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
    JOYARG="$(decide_joyarg)"
    [ "$JOYARG" = "true" ] && log "joy_node 가 없어서 직접 띄웁니다"
    warn_if_no_joy
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
    JOYARG="$(decide_joyarg)"
    [ "$JOYARG" = "true" ] && log "joy_node 가 없어서 직접 띄웁니다"
    warn_if_no_joy
    ros2 launch mlcs_mpc joystick.launch.py \
        joy:="$JOYARG" max_speed:="$SPEED" allow_toggle:=true debug:="$([ "$DEBUG" = 1 ] && echo true || echo false)"
    ;;

log)
    TAG="${1:-}"
    DATA_DIR="${MLCS_DATA:-$HOME/mlcs_data}"
    load_ros; require_ws_pkg mlcs_mpc
    trap cleanup EXIT INT TERM
    start_bringup

    log "HyperPM 학습 데이터 기록 → ${GRN}${DATA_DIR}${RST}"
    ros2 run mlcs_mpc data_logger --ros-args \
        -p output_dir:="$DATA_DIR" -p tag:="$TAG" >/tmp/mlcs_datalog.log 2>&1 &
    PIDS="$PIDS $!"
    sleep 1
    echo
    warn "운용 영역 전체를 덮도록 모세요 — 직선만 왕복하면 코너 데이터가 없습니다."
    echo "    가감속 / 좌우 선회 / 한계 부근을 골고루"
    echo "    로거 상태: tail -f /tmp/mlcs_datalog.log"
    echo
    JOYARG="$(decide_joyarg)"
    [ "$JOYARG" = "true" ] && log "joy_node 가 없어서 직접 띄웁니다"
    ros2 launch mlcs_mpc joystick.launch.py \
        joy:="$JOYARG" max_speed:="$SPEED" allow_toggle:=false

    sleep 1
    # 헤더 줄을 뺀 총 행수 → 분 (100Hz 기준). 파일이 없으면 0.
    # ★ grep -c 는 매치가 없으면 exit 1 이라 || echo 0 이 같이 찍혀
    #   "0\n0" 이 되고 awk 가 깨진다. head -1 로 한 줄만 취한다.
    TOT=$( { cat "$DATA_DIR"/drive_*.csv 2>/dev/null || true; } | grep -vc '^time' || true )
    TOT=$(echo "${TOT:-0}" | head -1)
    echo
    log "누적 데이터: ${GRN}$(awk -v n="$TOT" 'BEGIN{printf "%.1f", n/6000}')분${RST} / 36분 (논문 기준)"
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
    JOYARG="$(decide_joyarg)"
    [ "$JOYARG" = "true" ] && log "joy_node 가 없어서 직접 띄웁니다"
    warn_if_no_joy
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
    # ★ cleanup(정지명령 발행) 을 걸지 않는다 — 이 모드는 차를 건드리지
    #   않고 위치추정만 본다. 진단 실패로 exit 할 때마다 4.5초씩 정지
    #   명령을 쏘는 것은 낭비이고, 브링업도 안 띄웠으므로 받을 대상도 없다.
    trap 'for p in ${PIDS:-}; do kill "$p" 2>/dev/null || true; done' EXIT INT TERM

    # 설정된 토픽이 실제로 오는지 먼저 본다 — 안 오면 브릿지를 띄워도
    # 조용히 아무 일도 안 일어난다 (에러가 안 난다).
    CFG="$SCRIPT_DIR/src/mlcs_mpc/config/mocap.yaml"
    # 따옴표(작은/큰/없음)와 줄 끝 주석을 모두 벗겨낸다
    MT=$(grep -E "^[[:space:]]*mocap_topic:" "$CFG" | head -1 \
         | sed -e 's/#.*//' -e 's/^[^:]*:[[:space:]]*//' \
               -e "s/^['\"]//" -e "s/['\"][[:space:]]*$//" \
         | tr -d '[:space:]')
    log "mocap 토픽: ${GRN}${MT}${RST}  (config/mocap.yaml)"

    if ! ros2 topic list 2>/dev/null | grep -qx "$MT"; then
        err "$MT 토픽이 없습니다."
        echo
        echo "  1) natnet 드라이버가 떠 있나요?"
        echo "       ros2 launch natnet_ros2 natnet_ros2.launch.py \\"
        echo "           serverIP:=192.168.1.3 clientIP:=\$(hostname -I | awk '{print \$1}') \\"
        echo "           serverType:=unicast pub_rigid_body:=true activate:=true"
        echo
        echo "     ★ pub_rigid_body 와 activate 는 둘 다 기본값이 false 다."
        echo "       빼먹으면 연결은 되는데 토픽이 안 나온다 (조용히 실패)."
        echo
        echo "  2) 실제 토픽 이름이 다를 수 있습니다 (RigidBody 이름 = 토픽 이름):"
        ros2 topic list 2>/dev/null | grep -i pose | sed 's/^/       /' || echo "       (pose 토픽 없음)"
        echo
        echo "  3) 자세한 절차: docs/MOCAP_SETUP.md"
        exit 1
    fi

    log "토픽 존재 확인 — 수신율 측정 (5초)..."
    HZ=$(timeout 6 ros2 topic hz "$MT" 2>/dev/null | grep -m1 'average rate' | awk '{printf "%.0f", $3}')
    if [ -z "$HZ" ] || [ "$HZ" = "0" ]; then
        err "$MT 토픽은 있는데 데이터가 오지 않습니다 (0 Hz)."
        echo
        warn "가장 흔한 원인: Motive 의 Transmission Type 이 ${BLD}Multicast${RST}"
        echo "  젯슨이 무선이면 Multicast 는 AP 에서 걸러집니다 → ${BLD}Unicast${RST} 로 바꾸세요."
        echo "  그 다음 흔한 것: Motive 의 Local Interface 가 다른 랜카드로 잡힘"
        echo "  docs/MOCAP_SETUP.md 5단계 참고"
        exit 1
    fi
    log "수신율 ${GRN}${HZ} Hz${RST}"

    log "mocap_bridge 기동 (차량 브링업 없음)"
    ros2 run mlcs_mpc mocap_bridge --ros-args --params-file "$CFG" &
    PIDS="$PIDS $!"
    sleep 2
    echo
    log "차를 손으로 옮기며 확인하세요:"
    echo "    ros2 topic echo /mpc/state --field pose.pose.position   # 1m 옮기면 1 변화"
    echo "    ros2 topic echo /mpc/state --field twist.twist.linear.x # 밀면 +, 세우면 0"
    echo "    ros2 topic hz   /mpc/state                              # 100Hz 근처"
    echo
    wait
    ;;

mocapcal)
    CM="${1:-spin}"
    load_ros; require_ws_pkg mlcs_mpc
    trap 'for p in ${PIDS:-}; do kill "$p" 2>/dev/null || true; done' EXIT INT TERM
    CFG="$SCRIPT_DIR/src/mlcs_mpc/config/mocap.yaml"
    MT=$(grep -E "^[[:space:]]*mocap_topic:" "$CFG" | head -1 \
         | sed -e 's/#.*//' -e 's/^[^:]*:[[:space:]]*//' \
               -e "s/^['\"]//" -e "s/['\"][[:space:]]*$//" \
         | tr -d '[:space:]')
    if ! ros2 topic list 2>/dev/null | grep -qx "$MT"; then
        err "$MT 토픽이 없습니다 — natnet 드라이버를 먼저 띄우세요."
        err "  docs/MOCAP_SETUP.md 참고"
        exit 1
    fi
    ros2 run mlcs_mpc mocap_calibrate --ros-args \
        -p mocap_topic:="$MT" -p mode:="$CM"
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
    # ★ 정지 명령이 실제로 전달된 뒤에 죽인다 — 순서가 중요하다.
    #   먼저 죽이면 명령을 받아줄 노드가 없어지고, VESC 는 마지막 속도를
    #   그대로 유지한다.
    kill_matching 'ros2 launch mlcs_mpc'
    kill_matching 'ros2 launch f1tenth_stack'
    kill_matching 'bringup_launch'
    kill_matching 'install/mlcs_mpc/lib/mlcs_mpc/'
    kill_matching 'joystick_teleop'
    # ★ ros2 launch 를 죽여도 자식 노드는 고아로 남는다 (실측: launch 를
    #   SIGTERM 하면 joy_node / vesc 노드가 그대로 살아남았다). 노드를
    #   직접 지목해야 한다 — 특히 joy_node 는 살아 있으면 다음 기동에서
    #   'joy_node 가 이미 떠 있다' 로 오인될 수 있다.
    kill_matching '/lib/joy/joy_node'
    kill_matching '/lib/vesc_driver/'
    kill_matching '/lib/vesc_ackermann/'
    kill_matching '/lib/ackermann_mux/'
    kill_matching '[j]oy_teleop' -9
    sleep 1
    kill_matching 'install/mlcs_mpc/lib/mlcs_mpc/' -9
    kill_matching 'bringup_launch' -9
    kill_matching '/lib/joy/joy_node' -9
    log "종료 완료"
    ;;

help|--help|-h|*)
    sed -n '3,20p' "$0" | sed 's/^# \{0,1\}//'
    ;;
esac
