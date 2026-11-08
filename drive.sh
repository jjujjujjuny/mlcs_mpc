#!/usr/bin/env bash
# ============================================================================
#  drive.sh — 실차 딸깍 실행 스크립트
#
#  사용법:
#     ./drive.sh joy                 조이스틱 수동 조종 (+ 차량 브링업)
#                       --log        몰면서 HyperPM 학습 데이터도 쌓는다
#     ./drive.sh rally [태그]        원선회용 수동 조종 — 십자키로 최대 속도·조향 조절
#                                    (기록 포함, 태그 기본값 rally)
#     ./drive.sh bench               거치대 위 조향 확인 (속도 0 고정)
#     ./drive.sh servo               거치대 위 조향 기계 한계 측정 (키보드)
#     ./drive.sh mpc <경로.csv>      MPC 자율주행 (조이스틱 병행 — LB 로 전환)
#     ./drive.sh stanley [경로.csv]  Stanley 자율주행 (배관 검증용, 게인 2개)
#                                    인자 없으면 config/track.yaml 을 따른다
#                       --bag        주행 데이터를 rosbag 으로 기록
#                                    (횡오차·상태·명령 — 나중에 분석용)
#     ./drive.sh hmpc                HyperMPC OCP 노드. 기본은 거치대용 (속도 항상 0)
#                       --log [태그]  주행 데이터도 기록 (태그 기본값 hmpc)
#                       MLCS_HMPC_TRACK=<트랙>  트랙 지정 (이름 또는 prep csv)
#                       MLCS_HMPC_VMAX=1.0      속도 상한 (기본 config/hmpc.yaml 의 v_max)
#                       MLCS_HMPC_SOURCE=lidar MLCS_HMPC_MAP=<맵>  라이다 위치추정으로 (mocap 없이)
#     ./drive.sh drawtrack [이름]    손으로 레이싱 라인 클릭 → 트랙 생성 (차 불필요)
#     ./drive.sh stracks [이름]     직선+S자 트랙을 수식으로 생성 (차 불필요)
#     ./drive.sh preptrack <이름>    HyperMPC 원저자 전처리 → prep_*.csv
#     ./drive.sh preview <이름>      트랙 프리뷰 그림 (배치 + 곡률)
#     ./drive.sh trackinfo [파일]    트랙 기하 + 안전 여유 점검 (주행 없음)
#     ./drive.sh record <이름>       조이스틱으로 몰면서 웨이포인트 기록
#     ./drive.sh laptrack <기록>     그 기록 한 바퀴 → 트랙 (★ 권장)
#     ./drive.sh checkstate [초]     기록 전에 /mpc/state 가 멀쩡한지 점검
#     ./drive.sh cleanlog [로그...]  주행 로그에서 mocap 불량 구간 제거 → clean/
#     ./drive.sh hpmdata             정리된 로그 → HyperPM 학습 형식 (~/mlcs_data/hpm)
#     ./drive.sh actuator [로그...]  속도 명령 → 바퀴 응답. MPC 의 ws_acc_max / ws_dec_max 값
#     ./drive.sh cal <모드> [값]     캘리브레이션 (neutral|speed|steer)
#     ./drive.sh log [태그]          joy --log 과 같음 (예전 이름)
#     ./drive.sh mocap               mocap 위치추정만 (연동 확인용)
#     ./drive.sh mocapcal [spin|straight]  마커 오프셋 측정
#     ./drive.sh rviz                RViz 시각화 (젯슨에서 — NoMachine 으로 본다)
#     ./drive.sh lidar [--norviz]    라이다만 띄워서 확인 (RViz + 주기, 차 브링업 없음)
#     ./drive.sh mapping             라이다 맵핑 (Cartographer + RViz, 조이스틱으로 몬다)
#     ./drive.sh savemap [이름]      맵 저장 — mapping 중 다른 터미널에서 (maps/<이름>.*)
#     ./drive.sh natnet              mocap 드라이버만 (보통 자동으로 뜬다)
#
#   ── 라이다 맵 → 트랙 (대회: 라이다만, mocap 없음) ──
#     ./drive.sh mapping             ① 맵핑 (위에 있음)
#     ./drive.sh savemap <맵>        ② 저장 (위에 있음)
#     ./drive.sh racereport          라이다 주행을 mocap 정답으로 채점 (race.sh 가 자동으로)
#     ./drive.sh maptrack [맵] [트랙] ③ 맵 → 트랙 + prep + 프리뷰 한 번에 (최소 곡률선)
#                                    맵 생략 = 가장 최근 맵, 트랙 생략 = <맵>_track
#     ./drive.sh localize [맵]       저장한 맵으로 위치추정만 (맵은 안 바뀜)
#
#   ── 라이다 맵 ↔ mocap (연구실: 정답으로 검증) ──
#     ./drive.sh align [맵]          localize 중에 다른 창에서. 기록 → Ctrl+C 하면
#                                    첫 번째는 변환 맞추기, 그다음부터는 검증
#     ./drive.sh align fit|eval [기록] [맵]   다시 맞추기 / 다시 검증
#     ./drive.sh align list          맵별 변환·기록 목록
#     ./drive.sh mocaptrack <트랙> [맵]  map 프레임 트랙 → mocap 프레임 (<트랙>_mocap)
#
#     ./drive.sh go                  ★ 출발 — /mpc/enabled=true
#     ./drive.sh halt                ★ 정지 — /mpc/enabled=false (노드는 유지)
#     ./drive.sh topics              토픽 상태 점검
#     ./drive.sh stop                비상 정지 + 전부 종료
#
#  환경변수:
#     MLCS_WS=~/f1tenth_ws          워크스페이스 경로
#     MLCS_SPEED=1.5                수동 조종 최고 속도 (m/s)
#     MLCS_NO_BRINGUP=1             브링업을 안 띄운다 (이미 떠 있을 때)
#     MLCS_NO_NATNET=1              natnet 을 안 띄운다 (다른 창에서 띄웠을 때)
#     MLCS_SAFETY=1                 ★ 안전장치(경계 감시)를 **켠다** — 기본은 꺼짐
#     MLCS_MOTIVE_IP=192.168.1.3    Motive PC 의 IP
#     MLCS_DEBUG=1                  조이스틱 값 출력
#     MLCS_DATA=~/mlcs_data         데이터 로그 저장 폴더
#     MLCS_TRACK=<트랙폴더>          경계까지 그릴 원본 트랙 폴더
# ============================================================================
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${MLCS_WS:-$HOME/f1tenth_ws}"
SPEED="${MLCS_SPEED:-4.5}"
# ★ 속도는 반드시 소수점을 붙여 넘긴다.
#
#   ROS 2 launch 는 인자 문자열을 보고 타입을 추론한다. MLCS_SPEED=2 처럼
#   정수로 주면 INTEGER 로 넘어가는데, 노드는 declare_parameter('max_speed', 1.5)
#   로 DOUBLE 을 기대하므로 기동하다 죽는다:
#
#       InvalidParameterTypeException: Trying to set parameter 'max_speed'
#           to '2' of type 'INTEGER', expecting type 'DOUBLE'
#
#   조이스틱 노드가 통째로 안 뜨므로 수동 조종이 아예 안 된다.
#   2026-09-24 에 MLCS_SPEED=2 로 실제로 겪었다.
case "$SPEED" in
    *.*) ;;
    *)   SPEED="${SPEED}.0" ;;
esac
SPEED_SET=0
[ -n "${MLCS_SPEED:-}" ] && SPEED_SET=1
DEBUG="${MLCS_DEBUG:-0}"
MOTIVE_IP="${MLCS_MOTIVE_IP:-192.168.1.3}"

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

# ── HyperPM 학습 데이터 ────────────────────────────────────────────
#
#  ./drive.sh joy --log   과  ./drive.sh log   가 **같은 코드**를 쓴다.
#  따로 두면 한쪽만 고치는 일이 생긴다 (log 모드가 mocap 을 안 띄워서
#  데이터가 0분이었던 게 그런 경우였다 — 2026-11-02).
start_data_logger () {
    local tag="${1:-}"
    DATA_DIR="${MLCS_DATA:-$HOME/mlcs_data}"
    # ★ data_logger 는 /mpc/state 를 시간 기준으로 삼는다. mocap 이 없으면
    #   한 줄도 안 쌓인다. 36분을 모으기 전에 여기서 막는다.
    if [ "${STATE_SOURCE:-mocap}" != lidar ]; then
        start_natnet
        start_mocap_bridge
    fi
    ensure_single_state_publisher
    wait_for_state
    if ! python3 "$SCRIPT_DIR/tools/check_state.py" 4; then
        err "데이터 수집을 시작하지 않습니다. 위 안내대로 정리한 뒤 다시 하세요."
        exit 1
    fi
    log "HyperPM 학습 데이터 기록 → ${GRN}${DATA_DIR}${RST}"
    # ★ 빈 값을 넘기면 ROS 2 가 거부한다. 태그 없이 joy --log 를 쓰면
    #   '-p tag:=' 가 되어 rcl 이 기동 중에 죽는다:
    #
    #     RCLError: Couldn't parse parameter override rule: '-p tag:='
    #
    #   로거가 떠 있는 줄 알고 한참 몰았는데 한 줄도 안 쌓인다
    #   (2026-11-02 에 실제로 겪었다). 레포에서 track_dir / waypoints /
    #   MLCS_SPEED 로 이미 세 번 겪은 함정이다 — 인자 자체를 빼야 한다.
    local DL_ARGS=(-p output_dir:="$DATA_DIR")
    [ -n "$tag" ] && DL_ARGS+=(-p tag:="$tag")
    ros2 run mlcs_mpc data_logger --ros-args "${DL_ARGS[@]}" \
        >/tmp/mlcs_datalog.log 2>&1 &
    PIDS="$PIDS $!"
    # 로그는 파일로만 가니, ✗ ✓ 경고는 터미널에도 띄운다. 안 보이면 다 몰고 나서야 안다.
    ( tail -n 0 -F /tmp/mlcs_datalog.log 2>/dev/null | grep --line-buffered -E '✗|✓' ) &
    PIDS="$PIDS $!"
    sleep 1
    echo
    warn "운용 영역 전체를 덮도록 모으세요 — 한 바퀴만 돌면 좁은 영역만 쌓입니다."
    echo "    가감속 / 좌우 선회 / 8자 / 한계 부근을 골고루"
    echo "    로거 상태: tail -f /tmp/mlcs_datalog.log"
    echo
}

report_data_total () {
    [ -n "${DATA_DIR:-}" ] || return 0
    # 헤더 줄을 뺀 총 행수 → 분 (100Hz 기준). 파일이 없으면 0.
    # ★ grep -c 는 매치가 없으면 exit 1 이라 || echo 0 이 같이 찍혀
    #   "0\n0" 이 되고 awk 가 깨진다. head -1 로 한 줄만 취한다.
    local tot
    tot=$( { cat "$DATA_DIR"/drive_*.csv 2>/dev/null || true; } | grep -vc '^time' || true )
    tot=$(echo "${tot:-0}" | head -1)
    echo
    log "누적 데이터: ${GRN}$(awk -v n="$tot" 'BEGIN{printf "%.1f", n/6000}')분${RST} / 36분 (논문 기준)"
}

# 조이스틱 수동 조종 세션. LOGDATA=1 이면 학습 데이터를 같이 쌓는다.
joy_session () {
    local tag="${1:-}"
    start_bringup
    [ "$LOGDATA" = 1 ] && start_data_logger "$tag"

    local note=""
    [ "$LOGDATA" = 1 ] && note="  ${YLW}+ 학습 데이터 수집${RST}"
    if [ "$RALLY" = 1 ]; then
        # 상한은 config/joystick.yaml 의 rally_* 에서 시작한다. MLCS_SPEED 는 안 쓴다
        log "랠리 모드 수동 조종${note}"
    else
        log "조이스틱 수동 조종  ${GRN}max_speed=${SPEED} m/s${RST}${note}"
    fi
    echo
    echo "    RT 당김        전진 (놓으면 즉시 정지)"
    echo "    RB + RT        후진"
    echo "    L스틱          조향"
    if [ "$RALLY" = 1 ]; then
        echo "    십자키 위/아래  최대 속도 +/- 0.05 m/s  (RT 끝까지 = 이 값)"
        echo "    십자키 오른/왼  최대 조향 +/- 0.01 rad  (스틱 끝까지 = 이 값)"
    fi
    echo "    START          비상 정지"
    echo
    warn "기동 직후 RT 에서 손을 떼세요 — 트리거 휴지값을 자동으로 잽니다"
    echo
    # joy:=false — bringup 이 joy_node 를 이미 띄웠다
    JOYARG="$(decide_joyarg)"
    [ "$JOYARG" = "true" ] && log "joy_node 가 없어서 직접 띄웁니다"
    warn_if_no_joy
    ros2 launch mlcs_mpc joystick.launch.py \
        joy:="$JOYARG" max_speed:="$SPEED" allow_toggle:=false \
        rally:="$([ "$RALLY" = 1 ] && echo true || echo false)" \
        debug:="$([ "$DEBUG" = 1 ] && echo true || echo false)"

    [ "$LOGDATA" = 1 ] && { sleep 1; report_data_total; }
    return 0
}

# ── 종료 ────────────────────────────────────────────────────────────
#
#  ★ Ctrl+C 한 번이면 알아서 정리하고 **끝나야** 한다.
#
#    예전에는 `trap cleanup EXIT INT TERM` 이었다. 그런데 bash 의 INT 트랩은
#    핸들러를 돌린 뒤 **스크립트를 계속 진행**한다. 그래서 Ctrl+C 를 눌러도
#    /mpc/state 대기 루프가 계속 돌아 "✗ /mpc/state 가 오지 않습니다" 를
#    찍고서야 끝났다 (2026-11-02). 신호를 받으면 정리 후 exit 해야 한다.
#
#    정리 중에 Ctrl+C 를 **한 번 더** 누르면 기다리지 않고 바로 죽인다.
_CLEANED=0
_FORCE=0

# PID 와 그 자식들에게 신호를 보낸다.
# ★ `ros2 run` 래퍼는 받은 신호를 노드에 넘기지 않는다 (실측: 래퍼에 INT 를
#   줘도 노드가 살아 있었다). 래퍼만 죽이면 노드가 고아로 남는다 —
#   mocap_bridge 가 중복으로 떠 있던 원인 중 하나다. 자식까지 찍는다.
signal_tree () {
    local pid="$1" sig="$2" c g
    for c in $(pgrep -P "$pid" 2>/dev/null); do
        for g in $(pgrep -P "$c" 2>/dev/null); do kill "$sig" "$g" 2>/dev/null || true; done
        kill "$sig" "$c" 2>/dev/null || true
    done
    kill "$sig" "$pid" 2>/dev/null || true
}

# 우리가 띄운 것이 다 내려갈 때까지 기다린다 (최대 $1 초).
# ★ 고정 sleep 대신 폴링한다. 다 내려갔으면 바로 다음으로 — 종료가 빠르다.
wait_gone () {
    local limit="$1" t=0 p alive
    while :; do
        alive=0
        for p in ${PIDS:-}; do
            kill -0 "$p" 2>/dev/null && { alive=1; break; }
            [ -n "$(pgrep -P "$p" 2>/dev/null)" ] && { alive=1; break; }
        done
        [ "$alive" = 0 ] && return 0
        [ "$_FORCE" = 1 ] && return 0
        t=$(awk -v a="$t" 'BEGIN{print a+0.1}')
        awk -v a="$t" -v b="$limit" 'BEGIN{exit !(a>=b)}' && return 0
        sleep 0.1
    done
}

cleanup () {
    [ "$_CLEANED" = "1" ] && return
    _CLEANED=1
    echo
    # 정리 중 두 번째 Ctrl+C → 기다림을 건너뛰고 바로 죽인다
    trap '_FORCE=1; echo; warn "강제 종료합니다"' INT
    # ★ 기록을 **먼저** 닫는다. 노드를 죽인 뒤에 닫으면 마지막 구간이
    #   비거나 메타데이터가 깨진다.
    stop_bag
    # ★ VESC 가 떠 있을 때만 정지 명령을 쏜다. 없으면 ros2 CLI 기동만으로
    #   3초를 버린다 (mocap / natnet / rviz 모드).
    if bringup_running; then
        log "종료 중 — 정지 명령을 보냅니다"
        [ "$_FORCE" = 1 ] || stop_all
    else
        log "종료 중..."
    fi
    # ① INT — 노드가 저장할 틈을 준다 (data_logger / waypoint_logger)
    for p in ${PIDS:-}; do signal_tree "$p" -INT; done
    kill_matching 'install/mlcs_mpc/lib/mlcs_mpc/' -INT
    wait_gone 2.0
    # ② TERM — 브링업·natnet 은 setsid 로 띄워 Ctrl+C 가 안 닿는다. 직접.
    for p in ${PIDS:-}; do signal_tree "$p" -TERM; done
    [ -n "${BRINGUP_PGID:-}" ] && kill -TERM "-$BRINGUP_PGID" 2>/dev/null || true
    [ -n "${NATNET_PGID:-}" ] && kill -TERM "-$NATNET_PGID" 2>/dev/null || true
    wait_gone 1.5
    # ③ KILL — 남은 것
    for p in ${PIDS:-}; do signal_tree "$p" -KILL; done
    [ -n "${BRINGUP_PGID:-}" ] && kill -KILL "-$BRINGUP_PGID" 2>/dev/null || true
    [ -n "${NATNET_PGID:-}" ] && kill -KILL "-$NATNET_PGID" 2>/dev/null || true
    log "종료 완료"
}

# 신호를 받으면 정리하고 **끝낸다**. 130 = 128+SIGINT, 143 = 128+SIGTERM.
on_signal () {
    cleanup
    trap - EXIT        # EXIT 트랩이 cleanup 을 또 부르지 않게
    exit "$1"
}

# 모든 모드가 이걸로 트랩을 건다. 따로 쓰면 반드시 갈라진다.
arm_cleanup () {
    trap cleanup EXIT
    trap 'on_signal 130' INT
    trap 'on_signal 143' TERM
}

PIDS=""
BRINGUP_PGID=""
NATNET_PGID=""
# ── mocap 드라이버 ───────────────────────────────────────────────────────────
#
#   ★ 이 두 인자는 **둘 다 기본값이 false** 다. 빼먹으면 연결은 되는데
#     토픽이 하나도 안 나온다 (라이프사이클 노드가 활성화되지 않아
#     /natnet_ros/transition_event 만 남는다). 매번 손으로 치다 빠뜨리기
#     좋아서 여기에 박아 둔다.
#
#         pub_rigid_body:=true   강체 퍼블리셔를 만든다
#         activate:=true         라이프사이클 노드를 활성화한다
#
#   ★ Motive 2.x 를 쓴다면 patches/apply.sh 를 먼저 적용해야 한다.
#     안 하면 노드가 SIGABRT 로 즉사한다 (patches/README.md 참고).
natnet_running () {
    pgrep -f '/lib/natnet_ros2/natnet_ros2_node' >/dev/null 2>&1
}

wait_for_state () {
    # ★ 한 번만 보고 포기하면 안 된다. natnet → 브링업 → mocap_bridge 가
    #   차례로 뜨고 DDS 디스커버리까지 끝나야 /mpc/state 가 잡히는데,
    #   그게 5초를 넘긴다 (2026-09-30 실측: 단발 5초 확인은 실패,
    #   조금 더 기다리면 198Hz 로 멀쩡히 나온다). 20초까지 재시도한다.
    log "/mpc/state 대기 중..."
    local ok=0 _
    for _ in $(seq 1 4); do
        if timeout 5 ros2 topic echo /mpc/state --once >/dev/null 2>&1; then
            ok=1; break
        fi
    done
    if [ "$ok" != 1 ]; then
        err "/mpc/state 가 오지 않습니다 — mocap 연동을 먼저 확인하세요."
        err "  ./drive.sh mocap   으로 점검하고 docs/SETUP_JETSON.md 5절 참고"
        exit 1
    fi
    log "  ✓ /mpc/state 확인"
}

start_natnet () {
    if [ "${MLCS_NO_NATNET:-0}" = "1" ]; then
        log "natnet 생략 (MLCS_NO_NATNET=1)"
        return
    fi
    if natnet_running; then
        log "natnet 이 이미 떠 있습니다"
        return
    fi
    local CLIENT_IP
    CLIENT_IP="$(hostname -I | awk '{print $1}')"
    log "mocap 드라이버 (Motive ${MOTIVE_IP} → ${CLIENT_IP})..."
    # 브링업과 같은 이유로 setsid — Ctrl+C 경로 밖에 둔다 (start_bringup 주석)
    setsid ros2 launch natnet_ros2 natnet_ros2.launch.py \
        serverIP:="$MOTIVE_IP" clientIP:="$CLIENT_IP" serverType:=unicast \
        pub_rigid_body:=true activate:=true \
        >/tmp/mlcs_natnet.log 2>&1 &
    NATNET_PID=$!
    PIDS="$PIDS $NATNET_PID"
    sleep 5
    NATNET_PGID="$(ps -o pgid= -p "$NATNET_PID" 2>/dev/null | tr -d ' ')"

    # ★ 한 번 더 시도한다 — 재연결이 간헐적으로 실패한다.
    #
    #   유니캐스트 NatNet 은 Motive 가 접속 클라이언트를 기억한다. drive.sh stop
    #   으로 죽인 직후 몇 초 만에 다시 붙으면, 이전 세션이 정리되기 전이라
    #   서버 설명을 못 받고 SDK 가 assert 로 죽는다:
    #
    #       ValidateHostConnection(): Assertion `HostPresent' failed  (exit -6)
    #
    #   Motive 자체는 멀쩡하다 — 같은 순간 커맨드 포트를 직접 찔러보면
    #   "Motive / 2.3.0.1 / NatNet 3.1.0.0" 이 정상 응답한다.
    #   그래서 실패를 보고하기 전에 잠시 쉬었다 한 번 더 붙어 본다.
    #   2026-09-30 에 캘리브레이션 중간에 이걸로 한 번 날렸다.
    if ! natnet_running; then
        warn "natnet 이 안 떴습니다 — 잠시 뒤 다시 시도합니다"
        [ -n "${NATNET_PGID:-}" ] && kill -9 "-$NATNET_PGID" 2>/dev/null
        sleep 5
        setsid ros2 launch natnet_ros2 natnet_ros2.launch.py \
            serverIP:="$MOTIVE_IP" clientIP:="$CLIENT_IP" serverType:=unicast \
            pub_rigid_body:=true activate:=true \
            >/tmp/mlcs_natnet.log 2>&1 &
        NATNET_PID=$!
        PIDS="$PIDS $NATNET_PID"
        sleep 6
        NATNET_PGID="$(ps -o pgid= -p "$NATNET_PID" 2>/dev/null | tr -d ' ')"
    fi

    if natnet_running; then
        log "  ✓ mocap 드라이버 확인"
    else
        warn "mocap 드라이버가 안 뜹니다 — /tmp/mlcs_natnet.log 를 보세요."
        warn "  Motive 의 Data Streaming 이 켜져 있는지, Local Interface 가"
        warn "  ${MOTIVE_IP} 인지 확인하세요."
    fi
}

# ── mocap 브릿지 ─────────────────────────────────────────────────────────────
#
#   ★ 이미 떠 있으면 절대 또 띄우지 않는다.
#
#     같은 이름(/mocap_bridge)의 노드가 둘 이상이면 DDS 매칭이 꼬인다.
#     2026-09-30 실측: 브릿지가 3개 떠 있었고 /mpc/state 가 271Hz(≈3배)로
#     나오는데도 calibrate 는 "mocap 상태 대기 중" 에서 영영 멈춰 있었다.
#     토픽은 멀쩡히 보이니 원인을 찾기가 아주 어렵다.
#     (앞서 mocap_bridge 중복으로 스탬프 dt=0 이 나던 것과 같은 문제)
# ★ ros2 node list 를 쓰면 안 된다 — 데몬 캐시라 죽은 노드가 한동안 남는다
#   (joy_node_running 주석 참고). 그걸로 판단하면 "이미 떠 있다" 며 안 띄우고,
#   실제로는 아무것도 없어서 차가 안 움직인다. 2026-09-30 에 실제로 겪었다:
#   캘리브레이션이 평균 속도 -0.000 m/s 로 나왔다.
mocap_bridge_running () {
    pgrep -f '/lib/mlcs_mpc/mocap_bridge' >/dev/null 2>&1
}

# ★ /mpc/state 에 퍼블리셔가 둘이면 기록이 통째로 못 쓰게 된다.
#
#   waypoint_logger 는 min_dist(0.1m) 이상 움직였을 때만 점을 쌓는데,
#   퍼블리셔가 둘이면 점이 두 궤적을 **번갈아** 찍는다. 그러면 점간
#   거리가 min_dist 가 아니라 두 궤적 사이 거리가 되고, 어떤 평활화로도
#   복구되지 않는다 (2026-10-01 에 두 번 겪었다 — 점간 중앙 2.4 m).
#
#   ros2 topic info 는 데몬 캐시 때문에 못 믿는다. 프로세스를 직접 센다.
ensure_single_state_publisher () {
    local n extra p
    n="$(pgrep -fc '/lib/mlcs_mpc/mocap_bridge' 2>/dev/null || echo 0)"
    if pgrep -f '/lib/mlcs_mpc/sim_bridge' >/dev/null 2>&1; then
        warn "sim_bridge 가 떠 있습니다 — /mpc/state 를 같이 냅니다. 내립니다."
        kill_matching '/lib/mlcs_mpc/sim_bridge' -9
        sleep 1
    fi
    if [ "${n:-0}" -gt 1 ]; then
        warn "mocap_bridge 가 ${n}개 떠 있습니다 — 하나만 남깁니다."
        # 가장 오래된 것 하나만 남기고 나머지를 내린다
        extra="$(pgrep -f '/lib/mlcs_mpc/mocap_bridge' | tail -n +2)"
        for p in $extra; do kill -9 "$p" 2>/dev/null || true; done
        sleep 1
        n="$(pgrep -fc '/lib/mlcs_mpc/mocap_bridge' 2>/dev/null || echo 0)"
        log "  → 이제 ${n}개"
    fi
    if [ "${n:-0}" -gt 1 ]; then
        err "mocap_bridge 를 하나로 못 줄였습니다. ./drive.sh stop 후 다시 하세요."
        exit 1
    fi
}

start_mocap_bridge () {
    if mocap_bridge_running; then
        log "mocap 브릿지가 이미 떠 있습니다 — 그대로 씁니다"
        return
    fi
    log "mocap 브릿지..."
    ros2 run mlcs_mpc mocap_bridge --ros-args \
        --params-file "$SCRIPT_DIR/src/mlcs_mpc/config/mocap.yaml" \
        >/tmp/mlcs_mocap.log 2>&1 &
    PIDS="$PIDS $!"
    sleep 2
}

# ── 주행 기록 (rosbag) ───────────────────────────────────────────────────────
#
#   ./drive.sh stanley --bag   처럼 쓴다.
#
#   ★ 왜 필요한가 — 로그만으로는 "잘 돌았는지" 를 알 수 없다.
#     stanley 로그에는 막힌 사유(정지: ...)만 찍히고, 정작 품질 지표인
#     횡오차는 /stanley/cross_track 으로 흘러갈 뿐 어디에도 안 남는다.
#     2026-09-30 주행이 그랬다: 57.7초 무사고였다는 것만 알 수 있었고
#     "직진에서 바깥으로 치우친다" 는 체감을 숫자로 확인할 수 없었다.
# ── 실행별 로그 보존 ─────────────────────────────────────────────────────────
#
#   ★ /tmp/mlcs_stanley.log 를 매번 '>' 로 덮어쓰면 **직전 실행의 증거가
#     사라진다.** 2026-09-30 에 이걸로 막혔다: --bag 을 붙인 두 번의 주행이
#     안 돌았는데, 그 사이에 --bag 없이 한 번 더 띄우는 바람에 로그가 덮여
#     차단 사유(정지: ...)를 영영 확인할 수 없었다. bag 에는 enabled/manual 이
#     정상으로 찍혀 있어서 "왜 안 돌았나" 를 데이터로 좁히지 못했다.
#
#   그래서 실행마다 타임스탬프 파일에 쓰고, 문서에 적힌 /tmp 경로는
#   그 파일을 가리키는 심링크로 유지한다. tail -f 사용법은 그대로 두면서
#   지난 실행 기록이 남는다.
run_log () {          # $1 = 이름 (stanley / mpc / bringup)
    local dir="${MLCS_DATA:-$HOME/mlcs_data}/logs"
    mkdir -p "$dir"
    local f="$dir/$(date +%Y%m%d_%H%M%S)_$1.log"
    ln -sfn "$f" "/tmp/mlcs_$1.log"
    echo "$f"
}

BAG_PID=""
BAG_DIR=""

start_bag () {
    [ "${BAG:-0}" = "1" ] || return 0
    local base="${MLCS_DATA:-$HOME/mlcs_data}/bags"
    mkdir -p "$base"
    BAG_DIR="$base/$(date +%Y%m%d_%H%M%S)_${MODE}"
    log "기록 시작 → ${GRN}${BAG_DIR}${RST}"
    # 제어 품질을 되짚는 데 필요한 것만 담는다. /car/pose 와 /sensors/core 는
    # "제어기가 본 것" 과 "차가 실제로 한 것" 을 대조하려고 넣는다.
    setsid ros2 bag record -o "$BAG_DIR" \
        /stanley/cross_track /mpc/state /mpc/reference_path \
        /drive /ackermann_cmd /teleop \
        /commands/motor/speed /commands/servo/position \
        /car/pose /sensors/core /odom \
        /mpc/enabled /mpc/manual /mocap/valid \
        >/tmp/mlcs_bag.log 2>&1 &
    BAG_PID=$!
    sleep 2
}

stop_bag () {
    [ -n "${BAG_PID:-}" ] || return 0
    # ★ SIGINT 로 끝내야 한다. SIGKILL 이면 메타데이터가 안 써져서
    #   bag 이 열리지 않는다. 다 쓸 시간을 준다.
    kill -INT "$BAG_PID" 2>/dev/null
    for _ in $(seq 1 20); do
        kill -0 "$BAG_PID" 2>/dev/null || break
        sleep 0.25
    done
    kill -9 "$BAG_PID" 2>/dev/null
    if [ -d "${BAG_DIR:-}" ]; then
        log "기록 저장됨: ${GRN}${BAG_DIR}${RST}"
        log "  분석:  ros2 bag info $BAG_DIR"
    fi
    BAG_PID=""
}

bringup_running () {
    pgrep -f '/lib/vesc_driver/vesc_driver_node' >/dev/null 2>&1
}

start_bringup () {
    if [ "${MLCS_NO_BRINGUP:-0}" = "1" ]; then
        log "브링업 생략 (MLCS_NO_BRINGUP=1)"
        return
    fi
    # ★ 이미 떠 있으면 절대 또 띄우지 않는다.
    #
    #   2026-09-23 실측: 다른 창에서 ./drive.sh joy 를 돌리는 중에
    #   ./drive.sh stanley 를 실행했더니 브링업이 두 벌 떴다.
    #
    #       /ackermann_mux         ×2
    #       /ackermann_to_vesc_node ×2
    #       /vesc_to_odom_node      ×2
    #
    #   vesc_driver 는 시리얼이 배타적이라 하나만 살아남지만, mux 와
    #   ackermann_to_vesc 는 **둘 다 VESC 에 명령을 쏜다.** 게다가 같은
    #   이름의 노드가 둘이면 DDS 매칭이 꼬인다 (mocap_bridge 로 이미 겪었다).
    #
    #   joy 를 Ctrl+C 로 내려도 ros2 launch 만 죽고 자식 노드는 고아로
    #   남아서, 겉보기엔 아무것도 안 도는데 실제로는 살아 있었다.
    # 이 실행이 방금 띄운 것이면 조용히 넘어간다 (mapping → joy_session 이 두 번 부른다)
    [ -n "${BRINGUP_PID:-}" ] && return
    if bringup_running; then
        log "브링업이 이미 떠 있습니다 — 그대로 씁니다"
        warn "  다른 창의 drive.sh 를 아직 안 내렸다면 먼저 내리세요."
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
    setsid ros2 launch f1tenth_stack bringup_launch.py >"$(run_log bringup)" 2>&1 &
    BRINGUP_PID=$!
    PIDS="$PIDS $BRINGUP_PID"
    kill_bringup_joy &               # ★ launch 와 동시에 시작 (위 주석 참고)
    PIDS="$PIDS $!"
    sleep 3
    BRINGUP_PGID="$(ps -o pgid= -p "$BRINGUP_PID" 2>/dev/null | tr -d ' ')"
}

# ── 라이다 (Hokuyo UST-10LX) ─────────────────────────────────────────────────
#   bringup_launch.py 에서는 주석 처리돼 있다 (mocap 운용 때문). 필요한 모드만 띄운다.
#   이미 떠 있으면 그대로 쓴다 — 그 경우 PIDS 에 안 넣으므로 이 창이 내려도 안 죽는다.
#   네트워크: 유선 enP8p1s0 에 nmcli 프로파일 'hokuyo' (192.168.0.15/24)
# Cartographer 위치추정 (맵 고정). $1 = pbstream. localize, hmpc(라이다) 가 쓴다.
# ★ 코어 5 를 솔버 혼자 쓰게 한다.
#   이 스크립트(와 앞으로 띄울 자식들)는 0-4 로, 이미 떠 있는 내 프로세스(VS Code 등)도 0-4 로 옮긴다.
#   솔버 워커만 스스로 5 로 간다 (ocp_worker sched_setaffinity). 되돌리려면 재부팅하거나 taskset -a -cp 0-5 <pid>.
isolate_solver_core () {
    local n=0 p
    taskset -cp 0-4 $$ >/dev/null 2>&1
    for p in $(pgrep -u "$(id -u)"); do
        taskset -a -cp 0-4 "$p" >/dev/null 2>&1 && n=$((n + 1))
    done
    log "코어 배치: 솔버 5 전용, hmpc_node 4, Cartographer·RViz 0-3, 나머지 0-4  (옮긴 프로세스 $n 개)"
}

start_localization () {
    local pb="$1" base
    base="$(basename "${pb%.pbstream}")"
    log "Cartographer 위치추정 — 맵 $base  (코어 0-3)"
    # ★ 코어 0-3 에 묶는다. 안 묶으면 2 코어 넘게 쓰면서 솔버 코어(5)를 빼앗는다 —
    #   2026-11-08 라이다 주행에서 풀이 시간 22 → 31~35 ms (p95 47~62) 로 주기를 넘겼다.
    taskset -c 0-3 ros2 run cartographer_ros cartographer_node \
        -configuration_directory "$SCRIPT_DIR/src/mlcs_mpc/config/cartographer" \
        -configuration_basename localization_2d.lua \
        -load_state_filename "$pb" -load_frozen_state=true \
        >"$(run_log cartographer)" 2>&1 &
    PIDS="$PIDS $!"
    taskset -c 0-3 ros2 run cartographer_ros cartographer_occupancy_grid_node \
        -resolution 0.05 -publish_period_sec 1.0 \
        >/tmp/mlcs_cartogrid.log 2>&1 &
    PIDS="$PIDS $!"
    # 시작 위치: RViz 2D Pose Estimate / 조이스틱 LT → 스캔 매칭 → 궤적 다시 시작
    taskset -c 0-3 ros2 run mlcs_mpc relocalize --ros-args \
        -p configuration_directory:="$SCRIPT_DIR/src/mlcs_mpc/config/cartographer" \
        -p configuration_basename:=localization_2d.lua \
        >"$(run_log relocalize)" 2>&1 &
    PIDS="$PIDS $!"
    sleep 3
    if ! pgrep -f '/lib/cartographer_ros/cartographer_node' >/dev/null 2>&1; then
        err "Cartographer 가 기동 직후 죽었습니다 — tail -50 /tmp/mlcs_cartographer.log"
        exit 1
    fi
    echo "localize $base" > /tmp/mlcs_carto_mode
}

lidar_running () {
    pgrep -f '/lib/urg_node/urg_node_driver' >/dev/null 2>&1
}

# 라이다 관련 프로세스를 **확실히** 내린다 (stop, start_lidar 가 쓴다)
#
#   ★ 2026-11-08: 이전 실행의 urg_node 가 남아서 다음 주행 때 /scan 이 안 오는
#     일이 종종 있었다. 원인 두 가지:
#       · SIGTERM 만 보냈다. TCP 읽기에서 막힌 드라이버는 그걸로 안 죽는다.
#       · 'ros2 run urg_node ...' 래퍼는 '/lib/urg_node/' 패턴에 안 걸린다.
#     UST-10LX 는 **TCP 접속을 하나만** 받는다. 남은 드라이버가 접속을 쥐고
#     있으면 새 드라이버는 연결이 안 돼서 /scan 이 0 이 된다.
#   그래서 TERM → 최대 3초 대기 → KILL, 마지막에 라이다와의 TCP 접속이
#   남았는지까지 본다. (defunct 좀비는 부모가 죽으면 init 이 거둬 간다)
kill_lidar () {
    local pats=('/lib/urg_node/' 'ros2 run urg_node' 'ros2 topic hz /scan') p _
    for p in "${pats[@]}"; do kill_matching "$p"; done
    for _ in $(seq 1 30); do
        lidar_running || break
        sleep 0.1
    done
    for p in "${pats[@]}"; do kill_matching "$p" -9; done
    sleep 0.3
    if lidar_running; then
        err "라이다 드라이버가 KILL 로도 안 죽습니다:"
        pgrep -af '/lib/urg_node/' | sed 's/^/    /' >&2
        return 1
    fi
    if ss -tn state established dst 192.168.0.10 2>/dev/null | grep -q 192.168.0.10; then
        warn "라이다와의 TCP 접속이 아직 남아 있습니다 — 몇 초 뒤 풀립니다"
    fi
    return 0
}

start_lidar () {
    # 떠 있는데 /scan 이 안 오면 멈춘 좀비다 — 재사용하지 말고 내린 뒤 새로 띄운다
    if lidar_running && ! timeout 5 ros2 topic echo /scan --once >/dev/null 2>&1; then
        warn "라이다 드라이버가 떠 있는데 /scan 이 안 옵니다 — 내리고 다시 띄웁니다"
        kill_lidar || exit 1
        sleep 1
    fi
    if lidar_running; then
        log "라이다 드라이버가 이미 떠 있습니다 — 그대로 씁니다"
    else
        log "라이다 (Hokuyo UST-10LX, 192.168.0.10)..."
        local yaml
        yaml="$(ros2 pkg prefix f1tenth_stack)/share/f1tenth_stack/config/sensors.yaml"
        ros2 run urg_node urg_node_driver --ros-args --params-file "$yaml" \
            >"$(run_log lidar)" 2>&1 &
        PIDS="$PIDS $!"
    fi
    # ★ 시간으로 기다린다. 'echo --once' 는 토픽이 아직 광고되지 않았으면 1초 안에 바로 실패한다 —
    #   예전의 '3번 시도' 는 드라이버 접속이 늦으면 접속 전에 다 써 버렸다 (2026-11-08).
    local ok=0 end=$((SECONDS + 20))
    while [ $SECONDS -lt $end ]; do
        timeout 5 ros2 topic echo /scan --once >/dev/null 2>&1 && { ok=1; break; }
        sleep 0.5
    done
    if [ "$ok" != 1 ]; then
        err "/scan 이 오지 않습니다. 확인:"
        err "  ping 192.168.0.10            (안 되면  nmcli con up hokuyo)"
        err "  tail -f /tmp/mlcs_lidar.log"
        exit 1
    fi
    log "  ✓ /scan 확인"
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

# ★ --bag 은 어디에 붙어도 되게 한다. 나머지 인자는 그대로 남긴다.
#     ./drive.sh stanley --bag
#     ./drive.sh stanley other.csv --bag
BAG=0
LOGDATA=0
RALLY=0
_ARGS=()
for a in "$@"; do
    case "$a" in
        --bag) BAG=1 ;;
        --log) LOGDATA=1 ;;
        *)     _ARGS+=("$a") ;;
    esac
done
set -- "${_ARGS[@]+"${_ARGS[@]}"}"

case "$MODE" in

joy)
    # ./drive.sh joy          그냥 수동 조종
    # ./drive.sh joy --log    몰면서 HyperPM 학습 데이터도 쌓는다
    load_ros; require_ws_pkg mlcs_mpc
    arm_cleanup
    joy_session "${1:-}"
    ;;

rally)
    # ./drive.sh rally          원선회. 십자키로 최대 속도·조향을 옮긴다
    # ./drive.sh rally <태그>    파일명에 태그를 붙인다 (기본 rally)
    load_ros; require_ws_pkg mlcs_mpc
    arm_cleanup
    RALLY=1
    LOGDATA=1
    joy_session "${1:-rally}"
    ;;

bench)
    load_ros; require_ws_pkg mlcs_mpc
    arm_cleanup
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

stracks)
    # 직선 + S자 트랙을 해석적으로 생성 (tools/make_s_track.py)
    # 손클릭과 달리 곡률이 설계로 보장된다. 창도 ROS 도 차량도 필요 없다.
    python3 "$SCRIPT_DIR/tools/make_s_track.py" "$@"
    ;;

checkstate)
    # 기록 전에 /mpc/state 에 궤적이 하나만 오는지 몇 초 본다.
    load_ros
    python3 "$SCRIPT_DIR/tools/check_state.py" "$@"
    ;;

laptrack)
    # 수동 주행 기록 한 바퀴 → 트랙 (tools/track_from_lap.py)
    # 차가 실제로 돈 라인이라 선회반경과 경계가 자동으로 지켜진다.
    python3 "$SCRIPT_DIR/tools/track_from_lap.py" "$@"
    ;;

cleanlog)
    # 학습 전에 mocap 끊김·튐 구간을 잘라낸다 (tools/clean_drive_log.py)
    # ROS 도 차량도 필요 없다. 기록 중인 로그는 지정하지 말 것.
    python3 "$SCRIPT_DIR/tools/clean_drive_log.py" "$@"
    ;;

actuator)
    # ./drive.sh actuator [로그...]   속도 명령 → 바퀴 응답 (tools/actuator_id.py)
    #   일정 속도 오차(VESC Ki), 감속·가속 때 바퀴 가속도와 Iq, MPC 에 넣을 ws_acc_max / ws_dec_max
    #   로그를 안 주면 ~/mlcs_data 의 가장 최근 드라이브 로그 3개
    if [ $# -eq 0 ]; then
        mapfile -t _L < <(ls -t "${MLCS_DATA:-$HOME/mlcs_data}"/drive_*.csv 2>/dev/null | grep -v _clean | head -3)
        [ ${#_L[@]} -gt 0 ] || { err "드라이브 로그가 없습니다 (~/mlcs_data/drive_*.csv)"; exit 1; }
        set -- "${_L[@]}"
        log "가장 최근 로그 ${#_L[@]}개: $(for f in "${_L[@]}"; do basename "$f"; done | tr '\n' ' ')"
    fi
    python3 "$SCRIPT_DIR/tools/actuator_id.py" "$@"
    ;;

hpmdata)
    # cleanlog 결과 → 저자 형식 train/val/test (tools/prep_hpm_dataset.py)
    python3 "$SCRIPT_DIR/tools/prep_hpm_dataset.py" "$@"
    ;;

preview)
    # 트랙을 그림 한 장으로 — 배치 + 곡률 프로파일 (tools/preview_track.py)
    python3 "$SCRIPT_DIR/tools/preview_track.py" "$@"
    ;;

preptrack)
    # 저자들 TrackReader 를 그대로 돌려 prep_<이름>.csv 를 만든다.
    # ROS 도 차량도 창도 필요 없다 (MPLBACKEND=Agg).
    python3 "$SCRIPT_DIR/tools/prep_track.py" "$@"
    ;;

drawtrack)
    # 손클릭 레이싱 라인 → 닫힌 스플라인 → 좌/우 벽 (tools/draw_track.py)
    # ROS 도 차량도 필요 없다. 창을 띄우므로 NoMachine 으로 젯슨 화면에서.
    [ -n "${DISPLAY:-}" ] || { err "DISPLAY 가 없습니다 — NoMachine 으로 젯슨 화면에 접속한 뒤 실행하세요"; exit 1; }
    python3 "$SCRIPT_DIR/tools/draw_track.py" "$@"
    ;;

servo)
    # 조향 기계적 한계 찾기 — 서보 값을 키보드로 직접 민다 (tools/servo_sweep.py)
    load_ros
    arm_cleanup
    start_bringup
    log "${YLW}서보 한계 측정${RST} — 속도 명령 없음. 조향만 움직입니다."
    warn "거치대 위에서. 바퀴가 더 안 돌거나 서보가 '지잉' 거리면 거기가 한계 — 바로 되돌리세요."
    python3 "$SCRIPT_DIR/tools/servo_sweep.py" \
        "$WS/src/f1tenth_system/f1tenth_stack/config/vesc.yaml"
    ;;

stanley)
    # ★ 인자 없이 실행하면 config/track.yaml 을 따른다. 트랙을 바꾸려면
    #   그 파일의 waypoint_file 한 줄만 고치면 된다.
    #   인자를 주면 그것만 이번 한 번 덮어쓴다 (파일명만 줘도 된다).
    WP="${1:-}"
    if [ -n "$WP" ] && [ -f "$WP" ]; then
        WP="$(cd "$(dirname "$WP")" && pwd)/$(basename "$WP")"
    fi
    load_ros; require_ws_pkg mlcs_mpc
    arm_cleanup
    start_natnet
    start_bringup

    if [ -n "$WP" ]; then
        log "Stanley 자율주행  웨이포인트: ${GRN}$WP${RST}  속도 ${SPEED} m/s"
    else
        log "Stanley 자율주행  ${GRN}config/track.yaml${RST} 을 따릅니다"
        log "  트랙: $(grep -m1 'waypoint_file:' "$SCRIPT_DIR/src/mlcs_mpc/config/track.yaml" | sed "s/.*'\(.*\)'.*/\1/")"
    fi
    # ★ 빈 값을 'name:=' 로 넘기면 안 된다. ros2 launch 가 거부한다:
    #     malformed launch argument 'waypoints:=', expected format '<name>:=<value>'
    #   런치는 인자를 아예 안 주면 track.yaml 을 따르도록 되어 있으므로,
    #   값이 있을 때만 넣는다. (-p 빈 값이 rcl 을 죽이는 것과 같은 함정)
    ST_ARGS=(debug:="$([ "$DEBUG" = 1 ] && echo true || echo false)")
    [ -n "$WP" ] && ST_ARGS+=(waypoints:="$WP")
    [ "$SPEED_SET" = 1 ] && ST_ARGS+=(target_speed:="$SPEED")
    # ★ 안전장치는 **기본이 꺼짐**이다 (2026-09-30).
    #
    #   safety_node 의 비상정지에 리셋 경로가 없어서(_trip 이 래치하고 20Hz 로
    #   /mpc/enabled=false 를 계속 쏜다), 한 번 걸리면 재시작 말고는 방법이
    #   없었다. 트랙이 경계에 바짝 붙어 허용 횡오차가 34cm 뿐이라 정상
    #   주행 중에도 걸렸다.
    #
    #   리셋 경로를 넣으면 기본값을 다시 켜짐으로 되돌릴 것.
    if [ "${MLCS_SAFETY:-0}" = "1" ]; then
        ST_ARGS+=(safety:=true)
        log "안전장치 ${GRN}켜짐${RST} (MLCS_SAFETY=1) — 경계 ±2.5m, 허용 횡오차 34cm"
        warn "  비상정지에 리셋 경로가 없습니다. 걸리면 Ctrl+C 후 재실행하세요."
    else
        ST_ARGS+=(safety:=false)
        echo
        warn "${RED}${BLD}════════ 안전장치 꺼짐 ════════${RST}"
        warn "  경계 감시가 없습니다. 차가 트랙을 벗어나도 아무도 안 세웁니다."
        warn "  ${BLD}킬스위치를 손에 들고, 조이스틱 START(비상정지)를 기억하세요.${RST}"
        warn "  켜려면  MLCS_SAFETY=1 ./drive.sh stanley"
        echo
    fi
    ros2 launch mlcs_mpc stanley.launch.py "${ST_ARGS[@]}" \
        >"$(run_log stanley)" 2>&1 &
    PIDS="$PIDS $!"
    sleep 2
    start_bag
    echo
    warn "차는 아직 출발하지 않습니다. 준비되면 다른 창에서:"
    echo "    ./drive.sh go            # 출발"
    echo "    ./drive.sh halt          # 정지 (노드는 유지)"
    echo
    log "로그: tail -f /tmp/mlcs_stanley.log"
    log "횡오차: ros2 topic echo /stanley/cross_track"
    echo
    JOYARG="$(decide_joyarg)"
    [ "$JOYARG" = "true" ] && log "joy_node 가 없어서 직접 띄웁니다"
    ros2 launch mlcs_mpc joystick.launch.py \
        joy:="$JOYARG" max_speed:="$SPEED" allow_toggle:=true
    ;;

hmpc)
    # OCP 는 hypermpc/ocp_worker.py 가 따로 풀고 hmpc_node 가 감시한다.
    # ★ 기본은 bench 다. 달리려면 MLCS_HMPC_DRIVE=1 을 직접 줘야 한다.
    load_ros; require_ws_pkg mlcs_mpc
    # 위치 출처: MLCS_HMPC_SOURCE=lidar 면 Cartographer 위치추정 (맵 MLCS_HMPC_MAP),
    #   아니면 mocap. 라이다면 트랙도 같은 맵의 map 프레임 트랙이어야 한다 (*_mocap 아님).
    STATE_SOURCE="${MLCS_HMPC_SOURCE:-mocap}"
    if [ "$STATE_SOURCE" = lidar ]; then
        MAP_DIR="${MLCS_MAPS:-$SCRIPT_DIR/maps}"
        PB="$MAP_DIR/${MLCS_HMPC_MAP:-}.pbstream"
        [ -n "${MLCS_HMPC_MAP:-}" ] && [ -f "$PB" ] \
            || { err "라이다로 달리려면 맵이 필요합니다: MLCS_HMPC_MAP=<맵> (maps/<맵>.pbstream)"; exit 1; }
        if mocap_bridge_running; then
            err "mocap_bridge 가 떠 있습니다 — map 프레임 TF 가 Cartographer 와 겹칩니다. ./drive.sh stop"
            exit 1
        fi
        if pgrep -f '/lib/cartographer_ros/cartographer_node' >/dev/null 2>&1; then
            err "Cartographer 가 이미 떠 있습니다 — ./drive.sh stop"
            exit 1
        fi
        case "${MLCS_HMPC_TRACK:-}" in
            *_mocap*) err "라이다 주행인데 mocap 프레임 트랙입니다: $MLCS_HMPC_TRACK"; exit 1 ;;
        esac
    fi
    arm_cleanup
    isolate_solver_core
    if [ "$STATE_SOURCE" = lidar ]; then
        start_bringup
        start_lidar
        start_localization "$PB"
        if [ -n "${DISPLAY:-}" ]; then
            taskset -c 0-3 rviz2 -d "$SCRIPT_DIR/src/mlcs_mpc/config/cartographer/localization.rviz" \
                >/tmp/mlcs_rviz.log 2>&1 &
            PIDS="$PIDS $!"
        else
            warn "DISPLAY 가 없어 RViz 를 못 띄웁니다 — NoMachine 화면의 터미널에서 실행하세요"
        fi
        log "위치: ${GRN}라이다${RST} (Cartographer, 맵 ${MLCS_HMPC_MAP})"
        warn "${BLD}RViz 상단 '2D Pose Estimate' 로 차 위치·방향을 찍으세요${RST} — 찍기 전에는 B 를 눌러도 출발 안 함"
        echo "    LT 끝까지 = 지금 위치 근처에서 맵에 다시 맞춤 (차가 멈춰 있을 때)"
    else
        start_natnet
        start_bringup
        log "위치: mocap"
    fi
    HM_BENCH=true
    if [ "${MLCS_HMPC_DRIVE:-0}" = "1" ]; then
        HM_BENCH=false
        warn "${RED}${BLD}MLCS_HMPC_DRIVE=1 — 차가 실제로 움직입니다.${RST} 상한은 config/hmpc.yaml 의 v_max"
    else
        warn "bench — 속도는 항상 0 입니다. ${BLD}바퀴를 띄우고${RST} 조향만 확인하세요."
    fi
    # 트랙: MLCS_HMPC_TRACK=<prep csv 경로 또는 트랙 이름>. 없으면 config/hmpc.yaml
    HM_ARGS=(bench:="$HM_BENCH" source:="$STATE_SOURCE")
    if [ -n "${MLCS_HMPC_TRACK:-}" ]; then
        HT="$MLCS_HMPC_TRACK"
        [ -f "$HT" ] || HT="$SCRIPT_DIR/lab_map/data/tracks/$HT/prep_$HT.csv"
        [ -f "$HT" ] || { err "트랙을 못 찾았습니다: $MLCS_HMPC_TRACK"; exit 1; }
        HT="$(cd "$(dirname "$HT")" && pwd)/$(basename "$HT")"
        log "트랙: ${GRN}$HT${RST}"
        HM_ARGS+=(track_csv:="$HT")
    fi
    if [ -n "${MLCS_HMPC_PARAMS:-}" ]; then
        HM_ARGS+=(extra_params:="$MLCS_HMPC_PARAMS")
    fi
    if [ -n "${MLCS_HMPC_VMAX:-}" ]; then
        log "속도 상한: ${GRN}${MLCS_HMPC_VMAX} m/s${RST} (MLCS_HMPC_VMAX)"
        HM_ARGS+=(v_max:="$MLCS_HMPC_VMAX")
    fi
    # hmpc_node·브릿지는 코어 4, 솔버 워커는 스스로 코어 5 (config/hmpc.yaml solver_core)
    taskset -c 4 ros2 launch mlcs_mpc hmpc.launch.py "${HM_ARGS[@]}" >"$(run_log hmpc)" 2>&1 &
    PIDS="$PIDS $!"
    sleep 2
    if [ "$LOGDATA" = 1 ]; then
        # 브릿지는 위 런치가 띄운다. 로거가 먼저 보면 하나 더 띄우므로 기다린다.
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            pgrep -f '/lib/mlcs_mpc/(mocap|lidar)_bridge' >/dev/null 2>&1 && break
            sleep 0.5
        done
        start_data_logger "${1:-hmpc}"
        if [ "$STATE_SOURCE" = lidar ]; then
            # mocap 을 정답으로 같이 기록한다 (브릿지는 안 띄운다 — TF 가 겹친다)
            start_natnet
            ALIGN_LOG="$HOME/mlcs_data/align/${MLCS_HMPC_MAP}_$(date +%Y%m%d_%H%M%S)_race.csv"
            mkdir -p "$(dirname "$ALIGN_LOG")"
            python3 "$SCRIPT_DIR/tools/map_mocap_align.py" record "$ALIGN_LOG" \
                >/tmp/mlcs_race_align.log 2>&1 &
            PIDS="$PIDS $!"
            printf 'MAP=%q\nTRACK=%q\nALIGN=%q\nSTART=%q\n' "$MLCS_HMPC_MAP" "$HT" \
                "$ALIGN_LOG" "$(date +%s)" > /tmp/mlcs_race_last
            log "정답 기록 (mocap + 라이다): $ALIGN_LOG"
            echo "    끝나면  ./drive.sh racereport  (./race.sh 는 자동으로 돈다)"
        fi
    fi
    echo
    warn "${BLD}자율 모드로 시작합니다 — 조이스틱 B 를 누르면 출발${RST} (그 전에는 정지)"
    echo "    LB     → 수동 (비상정지, 사람이 조종)   LB 다시 → 바로 자율 재개 (B 불필요)"
    echo "    START  → 비상정지 (수동)                B      → mocap 끊김·솔버 고장 뒤 재출발"
    log "로그: tail -f /tmp/mlcs_hmpc.log    워커: /tmp/mlcs_hmpc_worker.log"
    log "상태: ros2 topic echo /hmpc/status   풀이 시간: ros2 topic echo /mpc/solve_time"
    echo
    JOYARG="$(decide_joyarg)"
    [ "$JOYARG" = "true" ] && log "joy_node 가 없어서 직접 띄웁니다"
    ros2 launch mlcs_mpc joystick.launch.py \
        joy:="$JOYARG" max_speed:=1.0 allow_toggle:=true start_auto:=true
    [ "$LOGDATA" = 1 ] && { sleep 1; report_data_total; }
    ;;

mpc)
    WP="${1:-}"
    [ -n "$WP" ] || { err "웨이포인트 파일이 필요합니다:  ./drive.sh mpc <경로.csv>"; exit 1; }
    [ -f "$WP" ] || { err "파일이 없습니다: $WP"; exit 1; }
    WP="$(cd "$(dirname "$WP")" && pwd)/$(basename "$WP")"   # 절대경로
    load_ros; require_ws_pkg mlcs_mpc
    arm_cleanup
    start_natnet
    start_bringup

    log "MPC 자율주행  웨이포인트: ${GRN}$WP${RST}"
    ros2 launch mlcs_mpc car_mpc.launch.py \
        waypoints:="$WP" target_speed:="$SPEED" >"$(run_log mpc)" 2>&1 &
    PIDS="$PIDS $!"
    sleep 2
    start_bag

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
    # ./drive.sh joy --log 과 같다. 예전부터 쓰던 이름이라 남겨둔다.
    load_ros; require_ws_pkg mlcs_mpc
    arm_cleanup
    LOGDATA=1
    joy_session "${1:-}"
    ;;

record)
    NAME="${1:-track_$(date +%Y%m%d_%H%M%S)}"
    OUT="$SCRIPT_DIR/src/mlcs_mpc/waypoints/${NAME}.csv"
    load_ros; require_ws_pkg mlcs_mpc
    arm_cleanup
    # ★ mocap 이 있어야 한다. waypoint_logger 는 /mpc/state 를 받아 적는데
    #   그 토픽을 내는 건 mocap_bridge 다. 예전에는 start_bringup 만 불러서
    #   **점이 0개로 끝나고 파일이 안 생겼다** (2026-10-01).
    start_natnet
    start_bringup
    start_mocap_bridge
    ensure_single_state_publisher
    wait_for_state
    # ★ 증상을 직접 잰다 — 원인(브릿지 2개/강체 2개/sim_bridge)을 짐작하지 않는다
    if ! python3 "$SCRIPT_DIR/tools/check_state.py" 4; then
        err "기록을 시작하지 않습니다. 위 안내대로 정리한 뒤 다시 하세요."
        exit 1
    fi

    log "웨이포인트 기록 → ${GRN}$OUT${RST}"
    ros2 run mlcs_mpc waypoint_logger --ros-args \
        -p output:="$OUT" >/tmp/mlcs_wp.log 2>&1 &
    PIDS="$PIDS $!"
    sleep 1
    echo
    warn "조이스틱으로 트랙을 한 바퀴 돈 뒤 Ctrl+C 로 저장하세요."
    log "  진행 상황은 다른 터미널에서:  tail -f /tmp/mlcs_wp.log"
    log "  두세 바퀴 돌면 laptrack 이 가장 매끄러운 바퀴를 골라 씁니다."
    echo
    JOYARG="$(decide_joyarg)"
    [ "$JOYARG" = "true" ] && log "joy_node 가 없어서 직접 띄웁니다"
    warn_if_no_joy
    ros2 launch mlcs_mpc joystick.launch.py \
        joy:="$JOYARG" max_speed:="$SPEED" allow_toggle:=false

    # 조이스틱이 끝나면 로거를 정리하고 평활화를 안내한다
    sleep 2
    if [ -f "$OUT" ]; then
        echo
        log "저장됨: $OUT"
        tail -4 /tmp/mlcs_wp.log 2>/dev/null | sed 's/^/    /'
        log "다음 — ${BLD}그대로 쓰지 마세요${RST} (사람 주행은 곡률이 튑니다):"
        echo "    ./drive.sh laptrack $NAME"
        echo "        한 바퀴만 잘라내고 평활화·벽·경계검사·HyperMPC 형식까지"
        echo "        한 번에 합니다. (중심선만 다듬으려면 smooth_path)"
    else
        echo
        err "파일이 생기지 않았습니다: $OUT"
        err "로거가 남긴 말:"
        tail -8 /tmp/mlcs_wp.log 2>/dev/null | sed 's/^/    /'
        echo
        err "흔한 원인:"
        err "  · 점이 3개 미만 — mocap 이 차를 못 봤다. ./drive.sh mocap 으로 확인"
        err "  · 차를 안 움직였다 — 0.1m 이상 움직여야 한 점이 쌓인다"
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
    arm_cleanup
    start_natnet
    start_bringup

    start_mocap_bridge

    # ★ 한 번만 보고 포기하면 안 된다. natnet → 브링업 → mocap_bridge 가
    #   차례로 뜨고 DDS 디스커버리까지 끝나야 /mpc/state 가 잡히는데,
    #   그게 5초를 넘긴다 (2026-09-30 실측: 단발 5초 확인은 실패,
    #   조금 더 기다리면 198Hz 로 멀쩡히 나온다). 20초까지 재시도한다.
    wait_for_state

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
    # ★ 공용 정리를 쓴다. 예전의 직접 쓴 트랩은 ① Ctrl+C 뒤에도 스크립트를
    #   계속 진행했고 ② 래퍼 PID 에만 신호를 줘서 노드(mocap_bridge 등)가
    #   고아로 남았다 — 다음 실행에서 브릿지가 중복되는 원인이 된다.
    arm_cleanup
    start_natnet

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
    # ★ 공용 정리를 쓴다. 예전의 직접 쓴 트랩은 ① Ctrl+C 뒤에도 스크립트를
    #   계속 진행했고 ② 래퍼 PID 에만 신호를 줘서 노드(mocap_bridge 등)가
    #   고아로 남았다 — 다음 실행에서 브릿지가 중복되는 원인이 된다.
    arm_cleanup
    start_natnet
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

rviz)
    # ★ **젯슨에서** 실행하고 NoMachine 으로 본다.
    #
    #   메인 PC 는 Ubuntu 20.04 + ROS 1 Noetic 이라 rviz2 가 없고,
    #   ROS 1 과 ROS 2 는 애초에 통신하지 않는다(DDS vs TCPROS).
    #   ros1_bridge 를 쓰려 해도 20.04 에 ROS 2 가 있어야 하는데 없다.
    #
    #   그래서 젯슨에서 띄우고 원격 데스크톱으로 화면만 가져온다.
    #   RViz 는 GPU 를 쓰므로 NoMachine 이 느리면 아래를 줄인다:
    #     · Trail 디스플레이 끄기 (점이 계속 쌓인다)
    #     · track_viz 의 trail_len 을 500 정도로
    load_ros
    if [ -z "${DISPLAY:-}" ]; then
        err "DISPLAY 가 없습니다 — SSH 로 접속하셨나요?"
        err "  NoMachine 같은 원격 데스크톱으로 접속해서 실행하세요."
        err "  (SSH 에서 쓰려면 ssh -X 로 접속)"
        exit 1
    fi
    command -v rviz2 >/dev/null || {
        err "rviz2 가 없습니다. ros-humble-desktop 을 설치하지 않은 경우입니다:"
        err "  sudo apt install -y ros-humble-rviz2"
        exit 1
    }
    # 인자를 주면 그 트랙, 없으면 config/track.yaml 을 따른다
    WP="${1:-}"
    RVIZ_CFG="$SCRIPT_DIR/src/mlcs_mpc/launch/track.rviz"
    # ★ 공용 정리를 쓴다. 예전의 직접 쓴 트랩은 ① Ctrl+C 뒤에도 스크립트를
    #   계속 진행했고 ② 래퍼 PID 에만 신호를 줘서 노드(mocap_bridge 등)가
    #   고아로 남았다 — 다음 실행에서 브릿지가 중복되는 원인이 된다.
    arm_cleanup

    # 젯슨이 이미 track_viz 를 띄웠으면 그 토픽을 쓰고, 아니면 여기서 띄운다
    if ros2 topic list 2>/dev/null | grep -qx /track/centerline; then
        log "젯슨의 track_viz 토픽을 사용합니다"
    else
        log "track_viz 를 로컬에서 띄웁니다 (트랙 표시용)"
        # ★ 빈 값을 -p 로 넘기면 안 된다. rcl 이 파싱에 실패해 노드가 죽는다:
        #     Couldn't parse parameter override rule: '-p track_dir:='
        #   그러면 RViz 는 뜨는데 트랙만 안 보인다 (원인 찾기 어렵다).
        #   기본값은 config/track.yaml 에서 온다.
        TV_ARGS=(--params-file "$SCRIPT_DIR/src/mlcs_mpc/config/track.yaml")
        [ -n "$WP" ] && TV_ARGS+=(-p "waypoint_file:=$WP")
        [ -n "${MLCS_TRACK:-}" ] && TV_ARGS+=(-p "track_dir:=$MLCS_TRACK")
        ros2 run mlcs_mpc track_viz --ros-args "${TV_ARGS[@]}" \
            >/tmp/mlcs_viz.log 2>&1 &
        PIDS="$PIDS $!"
        sleep 1
    fi
    log "RViz — Fixed Frame=map  (NoMachine 화면으로 보세요)"
    exec rviz2 -d "$RVIZ_CFG"
    ;;

natnet)
    # mocap 드라이버만 띄운다. 보통은 주행 모드가 알아서 띄우므로
    # 따로 쓸 일은 "mocap 만 확인하고 싶을 때" 정도다.
    load_ros
    # ★ 공용 정리를 쓴다. 예전의 직접 쓴 트랩은 ① Ctrl+C 뒤에도 스크립트를
    #   계속 진행했고 ② 래퍼 PID 에만 신호를 줘서 노드(mocap_bridge 등)가
    #   고아로 남았다 — 다음 실행에서 브릿지가 중복되는 원인이 된다.
    arm_cleanup
    start_natnet
    echo
    log "토픽 확인:"
    echo "    ros2 topic list | grep pose"
    echo "    ros2 topic hz /car/pose        # 100Hz 근처"
    echo
    log "로그: tail -f /tmp/mlcs_natnet.log     (Ctrl+C 로 종료)"
    wait
    ;;

go)
    # ★ 출발. 자율주행 노드는 /mpc/enabled=true 를 받아야 움직인다.
    #   래치(TRANSIENT_LOCAL) 라 한 번만 쏘면 늦게 뜬 노드도 받는다.
    load_ros
    if ! ros2 topic list 2>/dev/null | grep -qx /mpc/enabled; then
        err "/mpc/enabled 토픽이 없습니다 — 주행 노드가 안 떠 있습니다."
        err "  먼저 다른 창에서:  ./drive.sh stanley"
        exit 1
    fi
    warn "${BLD}차가 출발합니다. 킬스위치를 손에 드세요.${RST}"
    # ★ -w 1 — 구독자와 **매칭될 때까지 기다렸다가** 쏜다.
    #   -w 0 으로 두면 DDS 디스커버리가 끝나기 전에 발행할 수 있고, 그러면
    #   메시지가 조용히 사라진다. 실제로 기동 직후에 한 번 겪었다
    #   (드라이브 노드는 "출발 게이트 대기" 인데 go 는 "출발" 이라고 찍힘).
    #   그 뒤 13회 재시도에서 재현되지 않았지만, 여기서 놓치면 사람이
    #   "왜 안 나가지" 하며 다시 누르게 되므로 확실한 쪽을 쓴다.
    #   timeout 이 상한이라 구독자가 없어도 매달리지 않는다.
    #   -t 10 -r 20 = 0.5초간 10번. 한 번 놓쳐도 다음 것이 간다.
    if timeout 12 ros2 topic pub -t 10 -r 20 -w 1 /mpc/enabled \
            std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1; then
        log "${GRN}출발 신호 발행${RST} — 세우려면  ./drive.sh halt"

        # ★ enabled 만으로는 안 나간다. 컨트롤러는 manual 을 **먼저** 본다
        #   (stanley_node._blocked): 사람이 조종권을 쥐고 있으면 enabled 여도
        #   막힌다. joystick_teleop 은 /mpc/manual=true 로 시작하므로,
        #   조이스틱 LB 로 조종권을 넘기기 전까지 차는 가만히 있는다.
        #
        #   여기서 알려주지 않으면 "go 했는데 왜 안 가지" 로 한참 헤맨다.
        #   실제로 그랬다 (2026-09-23).
        # ★ 래치 토픽은 --qos-durability 만으로는 안 잡힌다.
        #   --qos-reliability reliable 까지 줘야 퍼블리셔와 매칭된다.
        MANUAL=$(timeout 8 ros2 topic echo /mpc/manual --once \
                     --qos-durability transient_local \
                     --qos-reliability reliable 2>/dev/null \
                 | grep -m1 'data:' | awk '{print $2}')
        if [ "${MANUAL:-}" = "true" ]; then
            echo
            warn "${BLD}아직 수동 조종 상태입니다 — 차는 안 나갑니다.${RST}"
            warn "  조이스틱 ${BLD}LB${RST} 를 눌러 조종권을 넘기세요."
            warn "  (컨트롤러는 /mpc/manual 을 /mpc/enabled 보다 먼저 봅니다)"
        elif [ -z "${MANUAL:-}" ]; then
            warn "/mpc/manual 을 못 읽었습니다 — 조이스틱이 안 떠 있을 수 있습니다."
            warn "  안 나가면 조이스틱 LB 를 눌러 보세요."
        fi

        # ★ safety_node 의 비상정지는 **영구 래치**다 (safety_node._trip).
        #   한 번 걸리면 tripped 를 되돌리는 경로가 없고, 20Hz 로
        #   /mpc/enabled=false 를 계속 쏜다. go 가 true 를 보내도
        #   즉시 덮어써져서 "출발/정지" 가 번갈아 찍히고 차는 안 나간다.
        #
        #   ⚠ /mpc/enabled 를 되읽어 판정하면 안 된다. 그 토픽을 래치로
        #     발행하는 건 safety_node 뿐이라, 안전장치를 끄면 퍼블리셔가
        #     0 이 되고 echo 가 그냥 타임아웃한다 — "값 없음" 과 "false" 를
        #     구분하지 못해 **오탐이 났다** (2026-09-24, 안전장치를 껐는데
        #     "트립됐다" 고 알림).
        #
        #   그래서 노드가 떠 있는지 + 로그에 트립이 찍혔는지로 본다.
        #   로그는 실행할 때마다 '>' 로 덮어쓰므로 이전 주행 기록은 안 섞인다.
        if ros2 node list 2>/dev/null | grep -qx /safety_node \
           && grep -q '비상 정지' /tmp/mlcs_stanley.log 2>/dev/null; then
            echo
            err "${BLD}출발 신호가 즉시 취소됐습니다 — safety_node 가 트립된 상태입니다.${RST}"
            err "  비상정지는 한 번 걸리면 재시작 전까지 안 풀립니다."
            err "  사유 확인:"
            err "      grep '비상 정지' /tmp/mlcs_stanley.log | tail -3"
            err "  복구: 주행 창에서 Ctrl+C 후 ./drive.sh stanley 다시 실행"
        fi
    else
        err "발행이 확인되지 않았습니다 — 주행 노드가 받았는지 로그를 보세요."
        err "  노드 로그에 '▶ 출발 허가' 가 찍혀야 합니다."
        exit 1
    fi
    ;;

halt)
    # 자율주행만 끈다. 노드는 살아 있으므로 ./drive.sh go 로 다시 출발한다.
    # 완전히 내리려면 ./drive.sh stop.
    load_ros
    log "자율주행 해제"
    pub_stop /mpc/enabled std_msgs/msg/Bool '{data: false}'
    pub_stop /drive ackermann_msgs/msg/AckermannDriveStamped \
        '{drive: {speed: 0.0, steering_angle: 0.0}}'
    log "정지 — 다시 출발하려면  ./drive.sh go"
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

trackinfo)
    # 주행 없이 트랙만 본다. 새 트랙을 넣었을 때 제일 먼저 돌릴 것.
    load_ros; require_ws_pkg mlcs_mpc
    # ★ 인자를 안 주면 -p 자체를 빼야 한다. 빈 값으로 넘기면
    #   "Couldn't parse parameter override rule: '-p waypoint_file:='" 로 죽는다.
    # vehicle.yaml — wheelbase / max_steer 를 실차 값으로 (안 주면 코드 기본 0.36)
    TI_ARGS=(--params-file "$SCRIPT_DIR/src/mlcs_mpc/config/vehicle.yaml"
             --params-file "$SCRIPT_DIR/src/mlcs_mpc/config/track.yaml")
    [ -n "${1:-}" ] && TI_ARGS+=(-p "waypoint_file:=$1")
    ros2 run mlcs_mpc track_info --ros-args "${TI_ARGS[@]}"
    ;;

lidar)
    # ./drive.sh lidar          라이다만 띄워서 확인 — 차 브링업 없음(모터 안 돈다)
    # ./drive.sh lidar --norviz  RViz 없이 정보·주기만
    #   RViz 는 Fixed Frame=laser 라 TF 없이도 보인다. Ctrl+C 로 종료.
    load_ros
    ros2 pkg prefix urg_node >/dev/null 2>&1 || {
        err "urg_node 가 없습니다:  sudo apt install -y ros-humble-urg-node"
        exit 1
    }
    arm_cleanup
    start_lidar
    echo
    log "스캔 정보 (/scan, 배열 생략)"
    timeout 5 ros2 topic echo --once --no-arr /scan 2>/dev/null \
        | grep -E 'frame_id|angle_min|angle_max|angle_increment|scan_time|range_min|range_max' \
        | sed 's/^/    /'
    if [ "${1:-}" != "--norviz" ] && [ -n "${DISPLAY:-}" ]; then
        rviz2 -d "$SCRIPT_DIR/src/mlcs_mpc/config/lidar.rviz" >/tmp/mlcs_rviz.log 2>&1 &
        PIDS="$PIDS $!"
        log "RViz — Fixed Frame=laser, 격자 1 m"
    elif [ -z "${DISPLAY:-}" ]; then
        warn "DISPLAY 가 없어 RViz 는 생략합니다 (NoMachine 화면에서 실행하세요)"
    fi
    echo
    log "주기 측정 — 정상은 약 40 Hz  (Ctrl+C 로 종료)"
    ros2 topic hz /scan
    ;;

mapping)
    # ./drive.sh mapping      라이다 + Cartographer 맵핑. 조이스틱으로 트랙을 돈다
    #   저장은 다른 터미널에서  ./drive.sh savemap [이름]
    #
    #   설정: src/mlcs_mpc/config/cartographer/mapping_2d.lua (UNICORN 기반 — 파일 주석 참고)
    #   ★ 경로를 소스에서 바로 읽는다 — lua/rviz 를 고쳐도 colcon build 가 필요 없다.
    load_ros; require_ws_pkg mlcs_mpc
    CARTO_DIR="$SCRIPT_DIR/src/mlcs_mpc/config/cartographer"
    for p in cartographer_ros urg_node; do
        ros2 pkg prefix "$p" >/dev/null 2>&1 || {
            err "패키지 '$p' 가 없습니다:"
            err "  sudo apt install -y ros-humble-cartographer-ros ros-humble-urg-node"
            exit 1
        }
    done
    # ★ mocap_bridge 는 map→base_link TF 를 낸다. Cartographer 의 map→odom 과
    #   같이 있으면 base_link 의 부모가 둘이 되어 TF 가 뒤죽박죽이 된다.
    if mocap_bridge_running; then
        err "mocap_bridge 가 떠 있습니다 — map 프레임 TF 가 Cartographer 와 겹칩니다."
        err "  ./drive.sh stop 으로 내린 뒤 다시 실행하세요."
        exit 1
    fi
    if pgrep -f '/lib/cartographer_ros/cartographer_node' >/dev/null 2>&1; then
        err "Cartographer 가 이미 떠 있습니다 (다른 창의 mapping?) — ./drive.sh stop"
        exit 1
    fi
    arm_cleanup
    # 라이다만으로 따라가므로 천천히 — 명시하지 않았으면 1.5 m/s
    [ "$SPEED_SET" = 0 ] && SPEED="1.5"

    start_bringup              # base_link→laser static TF, odom→base_link 가 여기서 나온다

    start_lidar

    log "Cartographer 맵핑 (mapping_2d.lua)..."
    ros2 run cartographer_ros cartographer_node \
        -configuration_directory "$CARTO_DIR" \
        -configuration_basename mapping_2d.lua \
        >"$(run_log cartographer)" 2>&1 &
    PIDS="$PIDS $!"
    ros2 run cartographer_ros cartographer_occupancy_grid_node \
        -resolution 0.05 -publish_period_sec 1.0 \
        >/tmp/mlcs_cartogrid.log 2>&1 &
    PIDS="$PIDS $!"
    sleep 2
    if ! pgrep -f '/lib/cartographer_ros/cartographer_node' >/dev/null 2>&1; then
        err "Cartographer 가 기동 직후 죽었습니다 — tail -50 /tmp/mlcs_cartographer.log"
        exit 1
    fi
    echo "mapping" > /tmp/mlcs_carto_mode     # ./drive.sh align 이 본다

    if [ -n "${DISPLAY:-}" ]; then
        rviz2 -d "$CARTO_DIR/mapping.rviz" >/tmp/mlcs_rviz.log 2>&1 &
        PIDS="$PIDS $!"
    else
        warn "DISPLAY 가 없어 RViz 는 생략합니다 (NoMachine 화면에서 실행하세요)"
    fi

    echo
    log "${GRN}맵핑 중${RST} — 천천히 트랙을 1~2바퀴 도세요"
    echo "    · 출발점으로 돌아와 같은 구간을 한 번 더 지나가야 루프가 닫힌다"
    echo "    · 제자리 급회전 금지 — IMU·오도메트리 없이 스캔만으로 따라간다"
    echo "    · 다 돌았으면 ${BLD}다른 터미널에서${RST}:  ./drive.sh savemap <이름>"
    echo "    · 로그: tail -f /tmp/mlcs_cartographer.log"
    echo
    joy_session
    ;;

savemap)
    # ./drive.sh savemap [이름]    mapping 이 도는 중에 다른 터미널에서
    #   → maps/<이름>.pbstream   Cartographer 원본 (pure localization 에 쓴다)
    #     maps/<이름>.pgm/.yaml  2D 점유격자 (파티클필터·RViz·경로 그리기용)
    #
    #   ★ 궤적을 끝낸(finish) 뒤 저장한다. 공식 절차가 finish → write_state 다.
    #     끝낸 궤적은 더 자라지 않으므로 저장 후에는 mapping 창을 Ctrl+C 로 닫는다.
    load_ros
    if ! pgrep -f '/lib/cartographer_ros/cartographer_node' >/dev/null 2>&1; then
        err "Cartographer 가 안 떠 있습니다 — 다른 터미널에서 ./drive.sh mapping 중이어야 합니다."
        exit 1
    fi
    MAP_DIR="${MLCS_MAPS:-$SCRIPT_DIR/maps}"
    NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"
    OUT="$MAP_DIR/$NAME"
    mkdir -p "$MAP_DIR"
    if [ -e "$OUT.pbstream" ]; then
        err "이미 있습니다: $OUT.pbstream — 다른 이름을 주세요"
        exit 1
    fi

    log "궤적 마무리 (finish_trajectory)..."
    # 이미 끝낸 궤적이면 실패 응답이 오는데, 저장은 그대로 하면 된다
    timeout 60 ros2 service call /finish_trajectory \
        cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" \
        | grep -o 'message=[^)]*' || warn "finish 응답 없음 — 저장은 계속합니다"

    log "pbstream 저장..."
    timeout 120 ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
        "{filename: '$OUT.pbstream', include_unfinished_submaps: true}" \
        | grep -o 'message=[^)]*' || true
    if [ ! -s "$OUT.pbstream" ]; then
        err "pbstream 이 안 써졌습니다: $OUT.pbstream  (tail /tmp/mlcs_cartographer.log)"
        exit 1
    fi

    log "pgm/yaml 변환..."
    ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
        -pbstream_filename "$OUT.pbstream" -map_filestem "$OUT" -resolution 0.05 \
        >/tmp/mlcs_savemap.log 2>&1 \
        || warn "pgm 변환 실패 — tail /tmp/mlcs_savemap.log  (pbstream 은 저장됨)"
    # ★ yaml 의 image: 가 절대경로로 써진다. 폴더째 옮기거나 다른 PC 로 복사하면
    #   map_server 가 pgm 을 못 찾는다 — 같은 폴더 기준 파일명으로 바꾼다.
    [ -f "$OUT.yaml" ] && sed -i "s|^image: .*|image: $NAME.pgm|" "$OUT.yaml"

    echo
    log "${GRN}저장 완료${RST}"
    ls -l "$OUT".* 2>/dev/null | awk '{print "    " $5 "\t" $NF}'
    echo
    echo "    이제 mapping 창을 Ctrl+C 로 닫으세요."
    echo "    다음:  ./drive.sh maptrack $NAME          (맵 → 트랙)"
    ;;

racereport)
    # ./drive.sh racereport     가장 최근 라이다 주행을 mocap 정답으로 채점 (tools/race_report.py)
    [ -f /tmp/mlcs_race_last ] || { err "라이다 주행 기록이 없습니다 — ./race.sh (source: lidar, log 켬)"; exit 1; }
    # shellcheck disable=SC1091
    . /tmp/mlcs_race_last
    DRV="$(find "${MLCS_DATA:-$HOME/mlcs_data}" -maxdepth 1 -name 'drive_*.csv' -newermt "@$START" 2>/dev/null \
           | sort | tail -1)"
    [ -n "$DRV" ] || { err "그 주행의 data_logger 기록을 못 찾았습니다"; exit 1; }
    [ -s "$ALIGN" ] || { err "정답 기록이 비었습니다: $ALIGN  (Motive 가 켜져 있었나요?)"; exit 1; }
    log "주행 $DRV"
    log "정답 $ALIGN"
    python3 "$SCRIPT_DIR/tools/race_report.py" --drive "$DRV" --align "$ALIGN" \
        --map "$MAP" --track "$TRACK"
    ;;

maptrack)
    # ./drive.sh maptrack [맵] [트랙이름] [옵션...]
    #   라이다 맵 → 트랙 → prep_*.csv → 프리뷰를 한 번에
    #   (tools/track_from_map.py → preview_track.py)
    #   맵을 안 주면 maps/ 에서 가장 최근 맵. 트랙 이름 기본 <맵>_track.
    #   옵션은 이름 뒤에 — 예:  ./drive.sh maptrack comp1 comp1_track --reverse
    #
    #   ★ 기본은 최소 곡률선 + 저자 전처리 없음 (2026-11-08). 가운데 선은 벽이 들쭉날쭉한
    #     연구실 맵에서 꺾였고, 저자 전처리는 코너 반경을 0.72~0.85 배로 줄여 세 맵이 다
    #     떨어졌다. 예전 방식(가운데 선 + 저자 전처리)은 --center
    MAP_DIR="${MLCS_MAPS:-$SCRIPT_DIR/maps}"
    MAPARG=""; TNAME=""
    if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then MAPARG="$1"; shift; fi
    if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then TNAME="$1"; shift; fi
    if [ -z "$MAPARG" ]; then
        MAPARG="$(ls -t "$MAP_DIR"/*.yaml 2>/dev/null | grep -v '\.mocap\.yaml$' | head -1)"
        [ -n "$MAPARG" ] || { err "maps/ 에 맵이 없습니다 — ./drive.sh mapping → savemap 먼저"; exit 1; }
        log "맵을 안 줘서 가장 최근 맵을 씁니다: $(basename "$MAPARG")"
    fi
    MBASE="$(basename "${MAPARG%.yaml}")"
    TNAME="${TNAME:-${MBASE}_track}"
    DRY=0; CENTER=0; MT_ARGS=()
    for a in "$@"; do
        case "$a" in
            --dry-run) DRY=1; MT_ARGS+=("$a") ;;
            --center)  CENTER=1 ;;
            *)         MT_ARGS+=("$a") ;;
        esac
    done
    # ★ 기본은 MPCC 용 --walls: 매끄러운 가운데 선(좌표계) + 지점별 벽 폭. MPC 가 벽 안에서 선을 고른다.
    #   --mincurv 를 주면 그 선을 쓰고, --center 는 예전 방식(저자 전처리)이다.
    if [ "$CENTER" != 1 ]; then
        case " ${MT_ARGS[*]+"${MT_ARGS[*]}"} " in
            *" --mincurv "*) MT_ARGS=(--no-author-prep "${MT_ARGS[@]}") ;;
            *) MT_ARGS=(--walls "${MT_ARGS[@]+"${MT_ARGS[@]}"}") ;;
        esac
    fi

    log "① 맵 → 트랙  ($MBASE → $TNAME)"
    python3 "$SCRIPT_DIR/tools/track_from_map.py" "$MAPARG" "$TNAME" "${MT_ARGS[@]+"${MT_ARGS[@]}"}" || {
        echo
        err "트랙을 못 만들었습니다 — 위 메시지를 보세요. 자주 쓰는 옵션:"
        echo "    --reverse            반대 방향으로 돈다"
        echo "    --start X Y          출발점 (기본 0 0 = 맵핑 시작 자리)"
        echo "    --allow-infeasible   차 한계보다 급한 코너가 있어도 저장 (폭이 넉넉할 때만)"
        echo "    --seal 0.15          벽 틈이 커서 트랙이 새면 키운다"
        echo "    --line-margin 0.30   선에서 벽까지 여유 (기본 0.40) — 좁은 트랙"
        echo "    --center             예전 방식 (가운데 선 + 저자 전처리)"
        echo "    -h                   전체 옵션"
        exit 1
    }
    [ "$DRY" = 1 ] && exit 0
    echo
    if [ "$CENTER" = 1 ]; then
        log "② 저자 전처리 → prep_$TNAME.csv"
        python3 "$SCRIPT_DIR/tools/prep_track.py" "$TNAME" \
            || warn "전처리 실패 — 트랙 파일은 저장됐습니다. ./drive.sh preptrack $TNAME 으로 다시"
    else
        log "② 저자 전처리 건너뜀 — prep_$TNAME.csv 는 ①에서 이 선으로 썼습니다"
    fi
    echo
    log "③ 프리뷰 그림"
    python3 "$SCRIPT_DIR/tools/preview_track.py" "$TNAME" || true
    echo
    log "${GRN}완료${RST} — 트랙: lab_map/data/tracks/$TNAME/   (map 프레임)"
    echo "    맵 위 그림:  lab_map/data/tracks/$TNAME/map_track.png"
    echo "    ★ map 프레임 트랙이라 라이다 위치추정(./drive.sh localize $MBASE)으로 달려야 맞습니다"
    echo "    연구실에서 mocap 으로 달리려면:  ./drive.sh mocaptrack $TNAME $MBASE"
    ;;

localize)
    # ./drive.sh localize [맵]
    #   저장한 맵(maps/<맵>.pbstream)으로 Cartographer 위치추정만 (맵은 안 바뀜)
    #   /tracked_pose, map→odom TF 가 나온다. 조이스틱 병행.
    #   맵을 안 주면 가장 최근 맵.
    load_ros; require_ws_pkg mlcs_mpc
    CARTO_DIR="$SCRIPT_DIR/src/mlcs_mpc/config/cartographer"
    MAP_DIR="${MLCS_MAPS:-$SCRIPT_DIR/maps}"
    if [ -n "${1:-}" ]; then
        PB="$MAP_DIR/$(basename "${1%.*}").pbstream"
        [ -f "$1" ] && [ "${1##*.}" = pbstream ] && PB="$1"
    else
        PB="$(ls -t "$MAP_DIR"/*.pbstream 2>/dev/null | head -1)"
    fi
    [ -n "$PB" ] && [ -f "$PB" ] || { err "pbstream 이 없습니다: ${PB:-maps/*.pbstream}  (./drive.sh savemap 으로 만든다)"; exit 1; }
    MBASE="$(basename "${PB%.pbstream}")"
    if mocap_bridge_running; then
        err "mocap_bridge 가 떠 있습니다 — map 프레임 TF 가 Cartographer 와 겹칩니다. ./drive.sh stop"
        exit 1
    fi
    if pgrep -f '/lib/cartographer_ros/cartographer_node' >/dev/null 2>&1; then
        err "Cartographer 가 이미 떠 있습니다 — ./drive.sh stop"
        exit 1
    fi
    arm_cleanup
    [ "$SPEED_SET" = 0 ] && SPEED="1.5"
    start_bringup
    start_lidar
    start_localization "$PB"
    if [ -n "${DISPLAY:-}" ]; then
        taskset -c 0-3 rviz2 -d "$CARTO_DIR/localization.rviz" >/tmp/mlcs_rviz.log 2>&1 &
        PIDS="$PIDS $!"
    fi
    echo
    log "${GRN}위치추정 중${RST} — 맵 $MBASE"
    echo "    · 맵핑을 시작한 자리·방향 근처에서 출발하세요 (새 궤적이 거기서 시작한다)"
    echo "    · 조금 움직여 RViz 에서 스캔이 맵 벽에 겹치면 붙은 것"
    echo "    · mocap 과 비교:  다른 터미널에서  ./drive.sh align"
    echo
    joy_session
    ;;

align)
    # 연구실용 — 라이다 맵 좌표계 ↔ mocap 좌표계 (tools/map_mocap_align.py)
    #
    #   ./drive.sh align [맵]              기록 → Ctrl+C 하면 자동으로
    #                                       · 맵의 변환이 아직 없으면 → 맞춘다 (fit)
    #                                       · 이미 있으면            → 검증한다 (eval)
    #   ./drive.sh align fit  [기록] [맵]   다시 맞춘다 (기록 기본: 그 맵의 가장 최근 기록)
    #   ./drive.sh align eval [기록] [맵]   다시 검증한다
    #   ./drive.sh align list              맵별 변환·기록 목록
    #
    #   맵 기본값: localize 로 띄운 맵 → 없으면 가장 최근 맵
    #   기록: ~/mlcs_data/align/<맵>_<시각>.csv   변환: maps/<맵>.mocap.yaml
    MAP_DIR="${MLCS_MAPS:-$SCRIPT_DIR/maps}"
    ALIGN_DIR="${MLCS_DATA:-$HOME/mlcs_data}/align"
    AL="$SCRIPT_DIR/tools/map_mocap_align.py"
    SUB="rec"
    case "${1:-}" in fit|eval|list|rec) SUB="$1"; shift ;; esac

    align_map () {      # $1 = 사용자가 준 맵 (없으면 추정)
        if [ -n "${1:-}" ]; then basename "${1%.*}"; return; fi
        if [ -f /tmp/mlcs_carto_mode ] && pgrep -f '/lib/cartographer_ros/cartographer_node' >/dev/null 2>&1; then
            read -r M N < /tmp/mlcs_carto_mode
            [ "$M" = localize ] && { echo "$N"; return; }
        fi
        basename "$(ls -t "$MAP_DIR"/*.pbstream 2>/dev/null | head -1)" .pbstream
    }
    align_fit () {      # $1 = 기록, $2 = 맵
        log "변환 맞추기 → maps/$2.mocap.yaml"
        python3 "$AL" fit "$1" "$MAP_DIR/$2"
    }
    align_eval () {
        log "검증 — maps/$2.mocap.yaml 로 이 기록의 라이다 위치 오차"
        python3 "$AL" eval "$1" "$MAP_DIR/$2.mocap.yaml"
    }

    case "$SUB" in
    list)
        for y in "$MAP_DIR"/*.yaml; do
            case "$y" in *.mocap.yaml) continue ;; esac
            [ -f "$y" ] || continue
            B="$(basename "$y" .yaml)"
            if [ -f "$MAP_DIR/$B.mocap.yaml" ]; then S="${GRN}변환 있음${RST}"; else S="변환 없음"; fi
            N=$(ls "$ALIGN_DIR/${B}"_*.csv 2>/dev/null | wc -l)
            echo "  $B   $S   기록 $N개"
        done
        ;;
    fit|eval)
        LOGF=""; MAPARG=""
        for a in "$@"; do
            case "$a" in *.csv) LOGF="$a" ;; *) MAPARG="$a" ;; esac
        done
        MB="$(align_map "$MAPARG")"
        [ -n "$MB" ] || { err "맵을 모르겠습니다 — ./drive.sh align $SUB [기록.csv] <맵>"; exit 1; }
        [ -n "$LOGF" ] || LOGF="$(ls -t "$ALIGN_DIR/${MB}"_*.csv 2>/dev/null | head -1)"
        [ -n "$LOGF" ] && [ -f "$LOGF" ] || { err "$MB 의 기록이 없습니다 ($ALIGN_DIR/${MB}_*.csv)"; exit 1; }
        if [ "$SUB" = eval ] && [ ! -f "$MAP_DIR/$MB.mocap.yaml" ]; then
            err "maps/$MB.mocap.yaml 이 없습니다 — 먼저 ./drive.sh align fit"
            exit 1
        fi
        "align_$SUB" "$LOGF" "$MB"
        ;;
    rec)
        load_ros
        MB="$(align_map "${1:-}")"
        [ -n "$MB" ] || { err "맵이 없습니다 — ./drive.sh mapping → savemap 먼저"; exit 1; }
        if mocap_bridge_running; then
            err "mocap_bridge 가 떠 있습니다 — TF 가 Cartographer 와 겹칩니다. ./drive.sh stop 후 localize 부터"
            exit 1
        fi
        if ! pgrep -f '/lib/cartographer_ros/cartographer_node' >/dev/null 2>&1; then
            err "Cartographer 가 안 떠 있습니다 — 다른 터미널에서 ./drive.sh localize $MB"
            exit 1
        fi
        if [ "$(cut -d' ' -f1 /tmp/mlcs_carto_mode 2>/dev/null)" != localize ]; then
            warn "Cartographer 가 맵핑 중입니다. 맵핑 중 자세는 나중에 루프가 닫히며 저장된 맵과"
            warn "어긋날 수 있습니다 — 정확한 검증은 savemap 뒤 ./drive.sh localize $MB 에서."
        fi
        arm_cleanup
        start_natnet           # mocap 드라이버만 (브릿지 없음). 이미 떠 있으면 그대로
        mkdir -p "$ALIGN_DIR"
        LOGF="$ALIGN_DIR/${MB}_$(date +%Y%m%d_%H%M%S).csv"
        HAS_T=0; [ -f "$MAP_DIR/$MB.mocap.yaml" ] && HAS_T=1
        echo
        if [ "$HAS_T" = 0 ]; then
            log "맵 ${BLD}$MB${RST} — 변환이 아직 없습니다. 이 기록으로 ${BLD}맞춥니다${RST}"
        else
            log "맵 ${BLD}$MB${RST} — 변환이 있습니다. 이 기록으로 ${BLD}검증합니다${RST}"
            echo "    (다시 맞추려면 끝난 뒤:  ./drive.sh align fit)"
        fi
        echo "    · 조이스틱은 localize 창에서. 2~3바퀴, 좌우 회전이 다 들어가게"
        echo "    · 다 돌았으면 이 창에서 Ctrl+C"
        echo
        # 기록기만 Ctrl+C 를 받게 한다 — 끝난 뒤 이 스크립트가 fit/eval 을 이어서 돈다
        trap ':' INT
        python3 "$AL" record "$LOGF"
        trap 'on_signal 130' INT        # arm_cleanup 의 것으로 되돌린다
        [ -s "$LOGF" ] || { err "기록이 비었습니다: $LOGF"; exit 1; }
        echo
        if [ "$HAS_T" = 0 ]; then align_fit "$LOGF" "$MB"; else align_eval "$LOGF" "$MB"; fi
        echo
        log "기록: $LOGF"
        [ "$HAS_T" = 0 ] && echo "    다음: 한 번 더  ./drive.sh align   → 다른 주행으로 검증"
        ;;
    esac
    ;;

mocaptrack)
    # ./drive.sh mocaptrack <트랙> [맵] [--to map] [--as 새이름]
    #   map 프레임 트랙(maptrack 결과) → mocap 프레임 (<트랙>_mocap). 연구실에서
    #   mocap 위치로 같은 트랙을 달릴 때. --to map 이면 반대 방향.
    #   맵 기본값: 트랙을 만든 맵 (track.json 에 적혀 있다)
    MAP_DIR="${MLCS_MAPS:-$SCRIPT_DIR/maps}"
    [ -n "${1:-}" ] && [ "${1#-}" = "$1" ] || { err "사용법: ./drive.sh mocaptrack <트랙> [맵]"; exit 1; }
    TR="$1"; shift
    MB=""
    if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then MB="$(basename "${1%.*}")"; shift; fi
    TO="mocap"
    for a in "$@"; do [ "$a" = map ] && TO=map; done
    if [ -z "$MB" ]; then
        TJ="$SCRIPT_DIR/lab_map/data/tracks/$TR/track.json"
        MB="$(python3 -c "import json,os,sys;f=json.load(open(sys.argv[1])).get('coordinate_frame',{});print(os.path.basename(f.get('map_yaml','')).rsplit('.',1)[0])" "$TJ" 2>/dev/null)"
    fi
    [ -n "$MB" ] || { err "어느 맵의 트랙인지 모르겠습니다 — ./drive.sh mocaptrack $TR <맵>"; exit 1; }
    [ -f "$MAP_DIR/$MB.mocap.yaml" ] || { err "maps/$MB.mocap.yaml 이 없습니다 — 먼저 ./drive.sh align"; exit 1; }
    case " $* " in *" --to "*) ;; *) set -- --to "$TO" "$@" ;; esac
    log "트랙 $TR → $TO 프레임  (변환 maps/$MB.mocap.yaml)"
    python3 "$SCRIPT_DIR/tools/map_mocap_align.py" apply "$MAP_DIR/$MB.mocap.yaml" "$TR" "$@"
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
    # ★ natnet 도 우리가 띄우므로 같이 내린다. stop 이 안 죽이면 다음
    #   실행에서 start_natnet 이 "이미 떠 있습니다" 로 넘어가 버려서,
    #   설정(Motive IP 등)을 바꿔도 반영되지 않는다 (2026-09-30 누락 발견).
    kill_matching '/lib/natnet_ros2/'
    kill_matching 'natnet_ros2.launch'
    kill_matching '/lib/vesc_driver/'
    kill_matching '/lib/vesc_ackermann/'
    kill_matching '/lib/ackermann_mux/'
    # lidar / mapping 모드가 띄우는 것들 (래퍼 'ros2 run ...' 까지)
    kill_matching '/lib/cartographer_ros/'
    kill_matching 'ros2 run cartographer_ros'
    kill_matching "rviz2 -d $SCRIPT_DIR/"          # 우리가 띄운 RViz 만
    kill_matching '[j]oy_teleop' -9
    sleep 1
    kill_matching 'install/mlcs_mpc/lib/mlcs_mpc/' -9
    kill_matching 'bringup_launch' -9
    kill_matching '/lib/joy/joy_node' -9
    kill_matching '/lib/cartographer_ros/' -9
    kill_matching 'ros2 run cartographer_ros' -9
    kill_lidar && log "  ✓ 라이다 정리"
    # 남은 게 있으면 보여준다 — 조용히 남아 있다가 다음 주행을 망치는 게 제일 나쁘다
    _left="$(pgrep -af '/lib/(urg_node|cartographer_ros|vesc_driver|vesc_ackermann|ackermann_mux|joy)/|install/mlcs_mpc/lib/mlcs_mpc/' 2>/dev/null)"
    if [ -n "$_left" ]; then
        warn "아직 남은 프로세스:"
        echo "$_left" | sed 's/^/    /'
    fi
    log "종료 완료"
    ;;

help|--help|-h|*)
    # ★ 범위는 '사용법:' 부터 'drive.sh stop' 줄까지 — stop 을 목록 맨 끝에 둔다.
    #   (예전엔 '3,32p' 고정이라 모드를 늘릴 때마다 잘렸다 — 2026-10-01, 11-07)
    sed -n '3,/drive\.sh stop /p' "$0" | sed 's/^# \{0,1\}//'
    ;;
esac
