#!/usr/bin/env bash
# ============================================================================
#  car.sh — ★ 노트북에서 쓰는 것. 젯슨 차를 한 단어로 다룬다.
#  새 노트북이면 먼저 tools/laptops/README.md (설치 안내).
#
#     ./car.sh setup            새 노트북 준비 (패키지·브리지 빌드·SSH 키, 한 번만)
#     ./car.sh ip 192.168.1.7   젯슨 주소 저장 (한 번만. 대회장에서 IP 바뀌면 다시)
#     ./car.sh key              SSH 비밀번호 안 묻게 (한 번만, 젯슨 비밀번호 필요)
#     ./car.sh pubkey           비밀번호를 모르면: 공개키를 레포에 넣는다 → push
#     ./car.sh find             ★ 대회장: 같은 공유기에서 젯슨을 찾아 주소 저장 + 시계 확인
#     ./car.sh clock            노트북·젯슨 시계 차이 (인터넷 없으면 NTP 가 안 맞춘다)
#
#     ./car.sh panel            ★ 제어판 (상태·실행·정지·설정·맵 저장·로그를 한 창에서)
#
#     ./car.sh bench            젯슨에서 race.sh 를 bench 로 (속도 0, race.yaml 안 고침)
#     ./car.sh race             젯슨에서 race.sh 실제 주행 (한 번 더 묻는다)
#     ./car.sh rviz             노트북 RViz (브리지 포함)
#     ./car.sh term             젯슨 터미널 (tmux race 세션에 붙기)
#     ./car.sh check            토픽이 잘 오는지 (/scan /tracked_pose 주파수)
#     ./car.sh bat              배터리
#     ./car.sh stop             ★ 젯슨 전부 정지·종료
#
#  주행 화면(bench/race/term)은 젯슨 tmux 안에서 돈다. 창을 닫거나 와이파이가
#  끊겨도 주행은 계속된다. 다시 보려면 ./car.sh term.
#  tmux 에서 빠지기만 하려면 Ctrl+b 누르고 d.
# ============================================================================
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_FILE="$HERE/tools/zenoh/.car_host"
USER_AT="mlcs"

RED=$'\033[31m'; GRN=$'\033[32m'; YLW=$'\033[33m'; BLD=$'\033[1m'; RST=$'\033[0m'
log()  { echo "${BLD}[car]${RST} $*"; }
err()  { echo "${RED}[car] ✗ $*${RST}" >&2; }

HOST="$(cat "$HOST_FILE" 2>/dev/null || echo 192.168.1.7)"

# ★ 젯슨 호스트 키를 IP 가 아니라 이름 'mlcs-car' 로 기억한다. 대회장에서 IP 가 바뀌어도
#   제어판(BatchMode)이 '처음 보는 호스트' 로 막히지 않고, 그 IP 에 다른 기기가 있으면 거부된다.
#   (젯슨을 다시 설치해 키가 바뀌면: ssh-keygen -R mlcs-car)
SSHO=(-o HostKeyAlias=mlcs-car)
learn_hostkey () {      # 처음 한 번만 기록 (이미 있으면 아무것도 안 한다)
    ssh-keygen -F mlcs-car >/dev/null 2>&1 && return 0
    ssh "${SSHO[@]}" -o StrictHostKeyChecking=accept-new -o BatchMode=yes -o ConnectTimeout=4 \
        "$USER_AT@$1" true >/dev/null 2>&1
    ssh-keygen -F mlcs-car >/dev/null 2>&1 && log "젯슨 호스트 키 기록 (mlcs-car)"
    return 0
}

need_car () {
    ping -c1 -W2 "$HOST" >/dev/null 2>&1 && { learn_hostkey "$HOST"; return 0; }
    err "젯슨($HOST)에 닿지 않습니다 — 같은 공유기인지 보고, 주소가 바뀌었으면:"
    err "  ./car.sh find          (같은 공유기에서 젯슨을 찾아 저장)"
    err "  ./car.sh ip <젯슨 IP>"
    exit 1
}

# 젯슨 tmux 'race' 세션에서 명령을 돌리고 붙는다. 세션이 이미 있으면 그냥 붙는다.
in_tmux () {
    need_car
    if ssh "${SSHO[@]}" "$USER_AT@$HOST" 'tmux has-session -t race 2>/dev/null'; then
        echo "${YLW}[car] 젯슨에 이미 돌고 있는 세션이 있어 거기에 붙습니다 (새로 시작하려면 먼저 ./car.sh stop)${RST}"
        sleep 1
        exec ssh "${SSHO[@]}" -t "$USER_AT@$HOST" 'tmux attach -t race'
    fi
    exec ssh "${SSHO[@]}" -t "$USER_AT@$HOST" "tmux new -s race 'cd ~/mlcs_mpc && $1; echo; echo \"[끝남 — 이 창은 exit 로 닫기]\"; exec bash -l'"
}

# VSCode(snap) 터미널은 snap 쪽 GTK·GSettings 경로를 물려줘서 Qt/RViz 창이 바로 죽는다
# (GLib-GIO-ERROR ... 'antialiasing', exit 133 — 2026-11-10 jaeungubuntu). 창을 띄우기 전에 원래 값으로.
unsnap () {
    case "${SNAP_NAME:-}${GSETTINGS_SCHEMA_DIR:-}" in *code*|*/snap/*) ;; *) return 0 ;; esac
    unset GSETTINGS_SCHEMA_DIR GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR
    [ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG+x}" ]   && export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
    [ -n "${XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG+x}" ] && export XDG_CONFIG_DIRS="$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG"
    return 0
}

case "${1:-help}" in
ip)
    [ -n "${2:-}" ] || { err "사용법: ./car.sh ip 192.168.x.x"; exit 1; }
    echo "$2" > "$HOST_FILE"
    log "젯슨 주소 저장: ${GRN}$2${RST}"
    ping -c1 -W2 "$2" >/dev/null 2>&1 && log "닿습니다 ✓" || err "아직 닿지 않습니다 (주소·공유기 확인)"
    ;;
find)
    # 대회장처럼 IP 를 모를 때: mlcs-car.local → 안 되면 같은 /24 에서 SSH 열린 곳을 훑어
    # hostname 이 mlcs-car 인 곳. 인터넷 없이 된다 (nmap 필요 없음)
    log "젯슨 찾기..."
    found=""
    ip4="$(timeout 3 getent ahostsv4 mlcs-car.local 2>/dev/null | awk 'NR==1{print $1}')"
    case "$ip4" in ""|172.17.*) ;; *)
        [ "$(ssh "${SSHO[@]}" -o BatchMode=yes -o ConnectTimeout=3 "$USER_AT@$ip4" hostname 2>/dev/null)" = mlcs-car ] \
            && found="$ip4" && log "mlcs-car.local → $ip4" ;;
    esac
    if [ -z "$found" ]; then
        dev="$(ip -4 route show default 2>/dev/null | awk '{print $5; exit}')"
        me="$(ip -4 -o addr show ${dev:+dev "$dev"} scope global 2>/dev/null | awk '$2!="docker0"{print $4; exit}')"
        [ -n "$me" ] || { err "이 노트북에 IP 가 없습니다 — 와이파이 연결부터"; exit 1; }
        net="${me%.*}"; log "mlcs-car.local 안 됨 → $net.0/24 훑기 (노트북 ${me%/*}, 약 3초)"
        cands="$(for i in $(seq 1 254); do
                    ( timeout 0.6 bash -c "echo >/dev/tcp/$net.$i/22" 2>/dev/null && echo "$net.$i" ) &
                 done; wait)"
        for c in $cands; do
            [ "$c" = "${me%/*}" ] && continue
            # 남의 기기일 수 있으니 known_hosts 에 남기지 않고 이름만 본다
            h="$(ssh -o BatchMode=yes -o ConnectTimeout=2 -o StrictHostKeyChecking=no \
                    -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR "$USER_AT@$c" hostname 2>/dev/null)"
            [ "$h" = mlcs-car ] && { found="$c"; break; }
        done
        [ -n "$found" ] || { err "못 찾음 — SSH 22 열린 곳: $(echo $cands | tr '\n' ' ')"
                             err "젯슨이 켜져 이 공유기에 붙었는지, 노트북 키가 등록됐는지 (./car.sh key)"; exit 1; }
    fi
    echo "$found" > "$HOST_FILE"; HOST="$found"
    log "젯슨: ${GRN}$found${RST} 저장"
    learn_hostkey "$found"
    exec "$0" clock
    ;;
clock)
    # 대회장에는 인터넷(NTP)이 없다. 노트북·젯슨 시계가 어긋나면 RViz 에 tf 경고가 난다
    need_car
    # 접속(핸드셰이크)이 끝난 뒤 왕복 한 번만 잰다 — 접속 시간이 섞이면 0.1 s 넘게 틀린다 (실측)
    coproc CK { ssh "${SSHO[@]}" -o BatchMode=yes "$USER_AT@$HOST" 'echo r; read x; date +%s.%N' 2>/dev/null; }
    read -r -t 8 rdy <&"${CK[0]}" || { err "젯슨 시간을 못 읽음 (SSH)"; exit 1; }
    t0="$(date +%s.%N)"; echo >&"${CK[1]}"; read -r -t 5 r <&"${CK[0]}"; t1="$(date +%s.%N)"
    [ -n "$r" ] || { err "젯슨 시간을 못 읽음 (SSH)"; exit 1; }
    awk -v a="$t0" -v b="$t1" -v r="$r" -v G="$GRN" -v Y="$YLW" -v Z="$RST" 'BEGIN{
        d=r-(a+b)/2; rt=(b-a)*1000
        c=(d<0.2 && d>-0.2)?G:Y
        printf "  시계 차이 (젯슨 − 노트북): %s%+.3f s%s  (왕복 %.0f ms)\n", c, d, Z, rt
        if (d>=0.2 || d<=-0.2) print "  ⚠ 0.2 s 넘음 — 대회장_yax_5G.md 의 '시계' 절"}'
    ;;
key)
    need_car
    [ -f ~/.ssh/id_ed25519 ] || ssh-keygen -t ed25519 -N '' -f ~/.ssh/id_ed25519
    ssh-copy-id "${SSHO[@]}" "$USER_AT@$HOST" && log "이제 비밀번호를 묻지 않습니다 ✓" \
        || err "등록 실패 — 젯슨 비밀번호를 모르면 ./car.sh pubkey"
    ;;
setup)
    exec "$HERE/tools/laptops/setup.sh"
    ;;
pubkey)
    [ -f ~/.ssh/id_ed25519.pub ] || ssh-keygen -t ed25519 -N '' -f ~/.ssh/id_ed25519 -C "$(whoami)@$(hostname)"
    name="${2:-$(whoami)@$(hostname)}"
    [[ "$name" =~ ^[A-Za-z0-9@._-]+$ ]] || { err "이름 형식: $name"; exit 1; }
    cp ~/.ssh/id_ed25519.pub "$HERE/tools/laptops/keys/$name.pub"
    log "공개키 → ${GRN}tools/laptops/keys/$name.pub${RST}"
    echo "  다음: git add tools/laptops/keys/$name.pub && git commit -m '노트북 키: $name' && git push"
    echo "  그리고 젯슨에서: cd ~/mlcs_mpc && git pull && tools/laptops/add_keys.sh"
    echo "  (비밀키 ~/.ssh/id_ed25519 는 절대 올리지 않는다 — .pub 만)"
    ;;
bench)
    log "젯슨 ${GRN}bench${RST} (속도 0 — 조향만). 다른 창에서 ./car.sh rviz"
    in_tmux "MLCS_RACE_BENCH=1 ./race.sh"
    ;;
race)
    need_car
    read -r -p "${RED}${BLD}실제로 달립니다.${RST} 트랙 비었고 킬스위치 손에 있나요? [y/N] " a
    [ "$a" = y ] || [ "$a" = Y ] || { log "취소"; exit 0; }
    in_tmux "./race.sh"
    ;;
term)
    need_car
    exec ssh "${SSHO[@]}" -t "$USER_AT@$HOST" 'tmux new -A -s race'
    ;;
rviz)
    need_car
    unsnap
    exec "$HERE/tools/zenoh/laptop.sh" "$HOST"
    ;;
panel)
    need_car
    PD="$HERE/tools/panel"
    [ -n "${ROS_DISTRO:-}" ] || { set +u; source /opt/ros/humble/setup.bash; set -u; }
    # 배터리 표시용 vesc_msgs (젯슨 f1tenth_ws 에서 복사한 메시지 정의) — 처음 한 번 빌드
    if [ ! -f "$PD/install/setup.bash" ]; then
        if command -v colcon >/dev/null; then
            log "vesc_msgs 빌드 (처음 한 번, 약 30초)..."
            ( cd "$PD" && colcon build --base-paths msgs --build-base build --install-base install \
                >"${TMPDIR:-/tmp}/mlcs_panel_build.log" 2>&1 ) \
                || err "vesc_msgs 빌드 실패 — 배터리 표시만 빠집니다 (${TMPDIR:-/tmp}/mlcs_panel_build.log)"
        else
            err "colcon 이 없어 vesc_msgs 를 못 만듭니다 — 배터리 표시만 빠집니다 (sudo apt install python3-colcon-common-extensions)"
        fi
    fi
    [ -f "$PD/install/setup.bash" ] && { set +u; source "$PD/install/setup.bash"; set -u; }
    python3 -c 'import PyQt5' 2>/dev/null || { err "PyQt5 가 없습니다: sudo apt install -y python3-pyqt5"; exit 1; }
    unsnap
    exec python3 "$PD/car_panel.py" "$HOST"
    ;;
check)
    [ -n "${ROS_DISTRO:-}" ] || { set +u; source /opt/ros/humble/setup.bash; set -u; }
    export ROS_LOCALHOST_ONLY=1
    pgrep -f 'tools/zenoh/x86_64/zenoh-bridge-ros2dds' >/dev/null \
        || { err "노트북 브리지가 안 떠 있습니다 — 다른 창에서 ./car.sh rviz 먼저"; exit 1; }
    for t in /scan /tracked_pose /tf; do
        r="$(timeout 6 ros2 topic hz "$t" 2>/dev/null | awk '/average rate/ {v=$3} END {print v}')"
        if [ -n "$r" ]; then printf '  %-15s %s%6.1f Hz%s\n' "$t" "$GRN" "$r" "$RST"
        else                 printf '  %-15s %s안 옴%s\n' "$t" "$RED" "$RST"; fi
    done
    echo "  (기대값: /scan ≈10, /tracked_pose ≈20, /tf 수십~수백)"
    ;;
bat)
    need_car
    ssh "${SSHO[@]}" -t "$USER_AT@$HOST" '~/mlcs_mpc/bat.sh'
    ;;
stop)
    need_car
    log "${RED}젯슨 정지·종료${RST}"
    ssh "${SSHO[@]}" -t "$USER_AT@$HOST" '~/mlcs_mpc/drive.sh stop; tmux kill-session -t race 2>/dev/null; true'
    ;;
*)
    sed -n '3,25p' "$0" | sed 's/^# \{0,1\}//'
    echo "  지금 젯슨 주소: $HOST"
    ;;
esac
