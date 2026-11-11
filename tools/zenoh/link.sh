#!/usr/bin/env bash
# ============================================================================
#  link.sh — 노트북 브리지를 SSH 터널 안으로 젯슨에 붙인다 (제어판·car.sh rviz 가 부른다)
#
#     tools/zenoh/link.sh <젯슨 IP>       (앞에서 계속 돈다. Ctrl+C·TERM 이면 둘 다 내린다)
#
#  ★ 2026-11-11: 대회 공유기는 다른 팀·같은 팀 다른 차도 쓴다. 젯슨 브리지가 와이파이에
#    열려 있으면 누구든 붙어 /initialpose(위치 덮어쓰기)·/mpc/go(출발)를 보낼 수 있었다.
#    이제 젯슨 브리지는 127.0.0.1:7447 에서만 듣고, 노트북은 SSH 터널로만 들어간다.
#    → SSH 키가 등록된 노트북만, 그리고 호스트 키가 'mlcs-car' 인 우리 젯슨에만 붙는다.
#
#    노트북 127.0.0.1:17447 ──ssh -L──▶ 젯슨 127.0.0.1:7447 (젯슨 브리지)
#
#  · 와이파이가 끊기면 ssh 가 약 9 초 뒤 끝나고, 2 초 뒤 다시 연다. 노트북 브리지는
#    exit_on_failure=false 라 터널이 돌아오면 알아서 다시 붙는다.
#  · 젯슨 쪽 터널 프로세스(sshd)는 remote.sh tunnel 이 코어 0-3·nice 19 (솔버 코어 5 피함).
# ============================================================================
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST="${1:?젯슨 주소}"
ZB="$HERE/x86_64/zenoh-bridge-ros2dds"
LPORT=17447
LOGD="${TMPDIR:-/tmp}"
O=(-o HostKeyAlias=mlcs-car -o BatchMode=yes -o ConnectTimeout=4
   -o ServerAliveInterval=3 -o ServerAliveCountMax=3)

[ -x "$ZB" ] || { echo "브리지 실행 파일이 없습니다: $ZB (./car.sh setup)" >&2; exit 1; }
export ROS_LOCALHOST_ONLY=1

TP=""; BP=""; LOOP=""
cleanup () {
    [ -n "$LOOP" ] && kill "$LOOP" 2>/dev/null
    [ -n "$BP" ] && kill "$BP" 2>/dev/null
    pkill -f "ssh .*-L 127.0.0.1:$LPORT:127.0.0.1:7447" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

# 터널: 끊기면 다시
(
    trap 'kill $SP 2>/dev/null; exit 0' TERM
    while :; do
        # 젯슨 쪽 remote.sh tunnel 이 이 연결의 sshd 를 코어 0-3·nice 19 로 묶고 연결 동안 기다린다
        ssh "${O[@]}" -n -o ExitOnForwardFailure=yes \
            -L "127.0.0.1:$LPORT:127.0.0.1:7447" "mlcs@$HOST" '~/mlcs_mpc/tools/panel/remote.sh tunnel' >/dev/null &
        SP=$!
        sleep 2
        kill -0 "$SP" 2>/dev/null && echo "$(date +%T) 터널 열림 → $HOST"
        wait "$SP"
        echo "$(date +%T) 터널 끊김 (rc=$?) — 2 초 뒤 다시"
        sleep 2
    done
) >>"$LOGD/mlcs_tunnel.log" 2>&1 &
LOOP=$!

"$ZB" -c "$HERE/laptop.json5" -e "tcp/127.0.0.1:$LPORT" >"$LOGD/mlcs_laptop_bridge.log" 2>&1 &
BP=$!
wait "$BP"
