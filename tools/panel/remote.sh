#!/usr/bin/env bash
# ============================================================================
#  remote.sh — 노트북 제어판(car_panel.py)이 SSH 로 부르는 젯슨 쪽 창구
#
#  사람이 직접 쓸 일은 없다. 노트북 제어판이 하나의 SSH 연결 위에서
#      ssh mlcs@<젯슨> ~/mlcs_mpc/tools/panel/remote.sh <명령> [인자]
#  로 부른다.
#
#  ★ 젯슨 부하 원칙 (2026-11-10)
#    · 조회 명령(status/lists/screen/logs)은 코어 0-3 + nice 19 로 돈다. 젯슨에는
#      isolcpus 가 없어서, 그냥 두면 솔버 코어 5 에 올라갈 수 있다.
#    · 조회는 bash·pgrep·grep 만 쓴다. 파이썬·ros2 CLI 는 기동만으로 CPU 를 먹는다.
#    · ★ 주행을 띄우는 명령(start)에는 nice·taskset 을 절대 물려주지 않는다.
#      tmux 서버가 nice 19 로 뜨면 그 안의 hmpc·솔버가 전부 최저 우선순위가 되고,
#      nice 는 root 없이 되돌릴 수 없다.
#
#  명령:
#     status              상태 key=value 줄 (제어판이 5초마다)
#     lists               맵·트랙 목록 (필요할 때만)
#     screen              tmux 'race' 화면 마지막 80줄 (그 탭을 볼 때만)
#     logs                로그 파일들 tail -F (연결 하나로 계속)
#     set <키> <값>       race.yaml 값 바꾸기 (map track v_max obstacles log loc_odom map_pin friction delay_comp act_delay)
#     start bench|race|mapping
#     int                 tmux 'race' 에 Ctrl+C (drive.sh 가 정리하며 내린다)
#     stop                drive.sh stop + tmux 세션 정리 (강제)
#     savemap <이름>      맵핑 중 맵 저장 (백그라운드)
#     maptrack <맵>       맵으로 트랙 만들기 (백그라운드)
#     usage               drive.sh·race.sh·bat.sh 맨 위 사용법 주석 (명령어 탭을 열 때만)
#     tunnel              노트북 SSH 터널(tools/zenoh/link.sh)의 젯슨 쪽. 그 터널 sshd 만 코어 0-3·nice 19
# ============================================================================
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
YAML="$ROOT/race.yaml"
JOB_LOG=/tmp/mlcs_panel_job.log
MODE_FILE=/tmp/mlcs_panel_mode

light () {      # 조회 명령: 코어 0-3, 최저 우선순위
    taskset -cp 0-3 $$ >/dev/null 2>&1
    renice -n 19 -p $$ >/dev/null 2>&1
}

yaml_get () {   # 최상위 키 값 (따옴표·주석 제거) — set 의 확인 출력용
    sed -n "s/^$1:[[:space:]]*//p" "$YAML" | head -1 | sed 's/[[:space:]]*#.*//; s/^"//; s/"$//'
}

# ★ status 는 5초마다 불린다. pgrep 7번 = /proc 7번 훑기 (0.28 s CPU), 줄마다 $( ) 서브셸도
#   fork 라 비싸다 (0.13 s). 프로세스 목록 한 번, yaml 은 awk 한 번, 나머지는 셸 내장만.
PROCS=""
has () { case "$PROCS" in *"$1"*) return 0 ;; esac; return 1; }
flag () { if has "$2"; then echo "$1=1"; else echo "$1=0"; fi; }

case "${1:-}" in
status)
    light
    PROCS="$(ps -eo args=)"
    if tmux has-session -t race 2>/dev/null; then
        echo "session=1"
        m=term; [ -r "$MODE_FILE" ] && read -r m < "$MODE_FILE"; echo "mode=$m"
    else
        echo "session=0"; echo "mode="
    fi
    flag hmpc   'lib/mlcs_mpc/hmpc_node'
    flag carto  '/lib/cartographer_ros/cartographer_node'
    flag bringup '/lib/vesc_driver/vesc_driver_node'
    flag lidar  '/lib/urg_node/'
    flag bridge 'tools/zenoh/aarch64/zenoh-bridge-ros2dds'
    if has 'mlcs_mpc/drive.sh savemap' || has 'mlcs_mpc/drive.sh maptrack'; then echo job=1; else echo job=0; fi
    c=""; [ -r /tmp/mlcs_carto_mode ] && read -r c < /tmp/mlcs_carto_mode; echo "carto_mode=$c"
    read -r l _ < /proc/loadavg; echo "load=$l"
    awk '
        /^(map|track|obstacles|log|drive|loc_odom|map_pin):/ {
            k=$1; sub(":","",k); v=$0; sub(/^[^:]*:[ \t]*/,"",v); sub(/[ \t]*#.*/,"",v); gsub(/"/,"",v)
            if (!(k in seen)) { print k "=" v; seen[k]=1 } }
        /^  v_max:/ && !vm { v=$2; print "v_max=" v; vm=1 }
        /^  delay_comp:/ && !dc { v=$2; print "delay_comp=" v; dc=1 }
        /^  act_delay:/ && !ad { v=$2; print "act_delay=" v; ad=1 }
        /^  friction:/ && !fr { v=$2; print "friction=" v; fr=1 }' "$YAML"
    ;;
lists)
    light
    echo "maps=$(ls -t "$ROOT"/maps/*.pbstream 2>/dev/null | xargs -r -n1 basename | sed 's/\.pbstream$//' | paste -sd,)"
    echo "tracks=$(cd "$ROOT/lab_map/data/tracks" 2>/dev/null && ls -td -- */ | tr -d / | paste -sd,)"
    ;;
usage)
    light
    # 각 스크립트 첫 줄(#!) 다음부터 주석이 끝날 때까지. 탭에 옮겨 적지 않고 여기서 읽는다
    for f in drive.sh race.sh bat.sh; do
        echo "@@@ $f"
        awk 'NR==1{next} /^#/{print; next} {exit}' "$ROOT/$f"
    done
    ;;
tunnel)
    # ★ link.sh 의 'ssh -L ... remote.sh tunnel' 로만 불린다. $PPID = 바로 이 터널 연결의 sshd.
    #   (sshd 는 소켓 정보를 사용자에게 안 보여 줘서 ss 로는 어느 sshd 가 터널인지 모른다 — 실측.
    #    이름으로 고르면 제어판 명령용 sshd 를 잘못 집을 수 있고, 그러면 start 가 nice 19 를 물려준다)
    taskset -acp 0-3 "$PPID" >/dev/null 2>&1
    renice -n 19 -p "$PPID" >/dev/null 2>&1
    light
    # 연결이 끊기면 echo 가 SIGPIPE 로 끝나 남지 않는다 (sleep 만 두면 고아로 남는다)
    while sleep 5; do echo || exit 0; done
    ;;
screen)
    light
    tmux capture-pane -p -J -S -80 -t race 2>/dev/null || echo "(젯슨에 실행 중인 화면 없음)"
    ;;
logs)
    light
    # tail -F: 파일이 나중에 생기거나 drive.sh 가 새 실행마다 링크를 바꿔도 따라간다
    exec tail -n 60 -F /tmp/mlcs_hmpc.log /tmp/mlcs_hmpc_worker.log /tmp/mlcs_cartographer.log \
        /tmp/mlcs_lidar.log /tmp/mlcs_bringup.log /tmp/mlcs_bridge.log /tmp/mlcs_relocalize.log \
        "$JOB_LOG" 2>/dev/null
    ;;
set)
    k="${2:-}"; v="${3:-}"
    # 값은 제어판이 고른 것이지만, SSH 로 들어오는 문자열이라 형식을 다시 묶는다
    case "$k" in
        map|track) [[ "$v" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "✗ 이름 형식: $v"; exit 1; }
                   sed -i -E "s/^($k:[[:space:]]*)\"[^\"]*\"/\1\"$v\"/" "$YAML" ;;
        obstacles) [[ "$v" =~ ^(off|watch|lidar|learn)$ ]] || { echo "✗ obstacles: $v"; exit 1; }
                   sed -i -E "s/^(obstacles:[[:space:]]*)[a-z]+/\1$v/" "$YAML" ;;
        loc_odom)  [[ "$v" =~ ^(off|ekf)$ ]] || { echo "✗ loc_odom: $v"; exit 1; }
                   if grep -q '^loc_odom:' "$YAML"; then sed -i -E "s/^(loc_odom:[[:space:]]*)[a-z]+/\1$v/" "$YAML"
                   else printf '\nloc_odom: %s\n' "$v" >> "$YAML"; fi ;;
        map_pin)   [[ "$v" =~ ^(true|false)$ ]] || { echo "✗ map_pin: $v"; exit 1; }
                   if grep -q '^map_pin:' "$YAML"; then sed -i -E "s/^(map_pin:[[:space:]]*)[a-z]+/\1$v/" "$YAML"
                   else printf '\nmap_pin: %s\n' "$v" >> "$YAML"; fi ;;
        log)       [[ "$v" =~ ^[A-Za-z0-9_-]*$ ]] || { echo "✗ log 형식: $v"; exit 1; }
                   [ -n "$v" ] || v="''"
                   sed -i -E "s/^(log:[[:space:]]*)[^[:space:]#]+/\1$v/" "$YAML" ;;
        v_max)     [[ "$v" =~ ^[0-9]+(\.[0-9]+)?$ ]] || { echo "✗ v_max: $v"; exit 1; }
                   sed -i -E "s/^(  v_max:[[:space:]]*)[0-9.]+/\1$v/" "$YAML" ;;
        friction)  [[ "$v" =~ ^0\.[0-9]+$ ]] || { echo "✗ friction (0.2~0.6): $v"; exit 1; }
                   awk -v x="$v" 'BEGIN{exit !(x >= 0.2 && x <= 0.6)}' || { echo "✗ friction (0.2~0.6): $v"; exit 1; }
                   sed -i -E "s/^(  friction:[[:space:]]*)[0-9.]+/\1$v/" "$YAML" ;;
        delay_comp|act_delay)
                   if [ "$k" = delay_comp ]; then [[ "$v" =~ ^(true|false)$ ]] || { echo "✗ delay_comp: $v"; exit 1; }
                   else [[ "$v" =~ ^0(\.[0-9]+)?$ ]] || { echo "✗ act_delay (0~0.2 s): $v"; exit 1; }; fi
                   # hmpc: 아래 두 칸 들여쓴 키. 없으면 hmpc: 바로 아래에 넣는다
                   if grep -qE "^  $k:" "$YAML"; then sed -i -E "s/^(  $k:[[:space:]]*)[^[:space:]#]+/\1$v/" "$YAML"
                   else sed -i -E "0,/^hmpc:/s//hmpc:\n  $k: $v/" "$YAML"; fi ;;
        *) echo "✗ 바꿀 수 없는 키: $k"; exit 1 ;;
    esac
    echo "✓ $k = $(case "$k" in v_max|friction|delay_comp|act_delay) sed -n "s/^  $k:[[:space:]]*\([^[:space:]#]*\).*/\1/p" "$YAML" | head -1 ;; *) yaml_get "$k" ;; esac)"
    ;;
start)
    # ★ light 를 부르지 않는다 (위 원칙)
    # 안전장치: 이 셸이 nice 상태면 tmux 서버·hmpc·솔버가 그걸 물려받는다 — 띄우지 않는다
    if [ "$(nice)" != 0 ]; then
        echo "✗ 명령 연결이 nice $(nice) 상태입니다 — 주행을 띄우지 않습니다. 제어판을 닫고 다시 여세요"; exit 1
    fi
    what="${2:-}"
    case "$what" in
        bench)   cmd="MLCS_RACE_BENCH=1 ./race.sh" ;;
        race)    cmd="./race.sh" ;;
        mapping) cmd="./drive.sh mapping" ;;
        *) echo "✗ start bench|race|mapping"; exit 1 ;;
    esac
    if tmux has-session -t race 2>/dev/null; then
        # 끝나고 화면만 남은 세션(mode=ended)은 치우고 새로 시작한다
        m=""; [ -r "$MODE_FILE" ] && read -r m < "$MODE_FILE"
        if [ "$m" = ended ]; then
            tmux kill-session -t race
        else
            echo "✗ 이미 실행 중입니다 (${m:-터미널}) — 먼저 정지하세요"; exit 1
        fi
    fi
    echo "$what" > "$MODE_FILE"
    # ★ 끝나면 MODE_FILE 에 ended — tmux 로는 '돌고 있음'과 '프롬프트만 남음'이 구분되지 않는다 (실측)
    tmux new -d -s race "cd '$ROOT' && $cmd; echo ended > '$MODE_FILE'; echo; echo '[끝남 — 노트북 제어판에서 새로 시작할 수 있습니다]'; exec bash -l"
    echo "✓ 시작: $what"
    ;;
int)
    tmux send-keys -t race C-c 2>/dev/null && echo "✓ Ctrl+C 보냄 — 정리하며 내려갑니다" \
        || echo "✗ 실행 중인 화면이 없습니다"
    ;;
stop)
    "$ROOT/drive.sh" stop
    tmux kill-session -t race 2>/dev/null
    rm -f "$MODE_FILE"
    echo "✓ 정지·종료"
    ;;
savemap|maptrack)
    n="${2:-}"
    [[ "$n" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "✗ 이름 형식: $n (영문·숫자·_-)"; exit 1; }
    if pgrep -f 'mlcs_mpc/drive.sh (savemap|maptrack)' >/dev/null 2>&1; then
        echo "✗ 이전 작업이 아직 돕니다"; exit 1
    fi
    { echo; echo "=== $(date +%H:%M:%S) $1 $n ==="; } >> "$JOB_LOG"
    # 맵 저장·트랙 만들기는 주행 중이 아닐 때 쓴다. 그래도 코어 0-3 에서 돌린다
    nohup taskset -c 0-3 "$ROOT/drive.sh" "$1" "$n" >> "$JOB_LOG" 2>&1 < /dev/null &
    echo "✓ $1 $n 시작 — '작업' 로그 탭에서 진행 확인"
    ;;
*)
    sed -n '3,30p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
