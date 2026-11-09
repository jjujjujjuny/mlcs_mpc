#!/usr/bin/env bash
# ============================================================================
#  sim.sh — 차 없이 mpcc 를 돌린다. 차는 env/simulator.py (ODE) — MPC 와 같은 env/vehicle_model.py 식·같은 mpcc.yaml 값.
#
#     ./sim.sh build               시뮬 워크스페이스 빌드 (처음 한 번, 패키지가 바뀌면 다시)
#     ./sim.sh main [옵션]          ★ 튜닝용 — 한 프로세스 실시간 + RViz (src/mlcs_mpc/main_sim.py --help)
#     ./sim.sh fast [옵션]          튜닝용 — 실시간 아님, 결과·그림만 (src/mlcs_mpc/main.py --help)
#     ./sim.sh                     race.yaml 의 track·mpcc 설정으로 시뮬 주행
#     ./sim.sh other.yaml          다른 설정 파일
#     ./sim.sh track <트랙> [v_max]  race.yaml 없이 트랙만 (이름 또는 prep csv)
#     ./sim.sh go | halt           출발 / 정지 (/mpc/enabled) — 다른 터미널에서
#     ./sim.sh rviz                RViz2 만 띄운다 (sim.rviz 설정). ★ rosrun rviz 는 ROS 1 이라 안 보인다
#     ./sim.sh status              /sim/status 한 번 (s, n, 랩 시간, 벽)
#
#  환경변수:
#     MLCS_SIM_WS=~/mlcs_sim_ws    시뮬 워크스페이스
#     MLCS_SIM_PARAMS=<yaml>       sim_node 에만 줄 파라미터 (차만 다른 값 등 — src/mlcs_mpc/mlcs_mpc/env/README.md)
#     MLCS_SIM_ONWALL=warn         벽에 닿으면 warn | stop | reset
#     MLCS_SIM_AUTOSTART=3.0       이만큼 뒤 자동 출발. 음수면 ./sim.sh go 를 기다린다
#     MLCS_SIM_BENCH=1             mpcc bench (속도 0, 조향만)
#     MLCS_SIM_NORVIZ=1            RViz 를 안 띄운다
#     MLCS_SIM_LOGS=<저장소>/logs/sim  차 상태 기록 → sim_<시각>.csv (튜닝 그림은 logs/tune/)
#
#  ★ race.yaml 의 source·map·drive·log 는 시뮬에서 쓰지 않는다. 트랙 좌표계 그대로 달린다.
# ============================================================================
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${MLCS_SIM_WS:-$HOME/mlcs_sim_ws}"
OUT=/tmp/mlcs_sim_params.yaml

source_ros () {
    set +u
    if [ "${ROS_VERSION:-}" = 1 ]; then
        # 메인 PC 는 .bashrc 가 ROS 1 (noetic) 을 띄운다. 섞이면 colcon 이 ROS 2 패키지를 못 찾는다.
        local old="/opt/ros/$ROS_DISTRO" v
        for v in PATH LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH PKG_CONFIG_PATH; do
            export "$v=$(printf '%s' "${!v:-}" | tr ':' '\n' | grep -v "^$old" | paste -sd: -)"
        done
        unset ROS_DISTRO ROS_VERSION ROS_PACKAGE_PATH ROS_ROOT ROS_MASTER_URI ROS_ETC_DIR ROSLISP_PACKAGE_DIRECTORIES
    fi
    if [ -z "${ROS_DISTRO:-}" ]; then
        for d in humble foxy; do
            [ -f "/opt/ros/$d/setup.bash" ] && { source "/opt/ros/$d/setup.bash"; break; }
        done
    fi
    [ -n "${ROS_DISTRO:-}" ] || { echo "ROS 2 가 없습니다 (/opt/ros/humble 또는 foxy)"; exit 1; }
    # 젯슨이면 차 워크스페이스 (vesc_msgs 가 있다) 위에 얹는다
    [ -f "$HOME/f1tenth_ws/install/setup.bash" ] && source "$HOME/f1tenth_ws/install/setup.bash"
    [ "${1:-}" = ws ] && [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
    set -u
}

case "${1:-}" in
build)
    source_ros
    mkdir -p "$WS/src"
    rm -f "$WS/src/mlcs_sim"                    # 예전 구조 (sim/mlcs_sim)
    ln -sfn "$HERE/src/mlcs_mpc" "$WS/src/mlcs_mpc"
    if ros2 pkg prefix vesc_msgs >/dev/null 2>&1 && [ ! -L "$WS/src/vesc_msgs" ]; then
        echo "vesc_msgs 는 이미 있습니다 ($(ros2 pkg prefix vesc_msgs)) — 그것을 씁니다"
    else
        ln -sfn "$HERE/third_party/vesc_msgs" "$WS/src/vesc_msgs"
    fi
    cd "$WS" && colcon build --symlink-install
    echo "✓ $WS — 이제 ./sim.sh"
    exit 0 ;;
main|fast)
    source_ros ws
    if [ -z "${ACADOS_SOURCE_DIR:-}" ] && [ -d "$HOME/acados" ]; then export ACADOS_SOURCE_DIR="$HOME/acados"; fi
    [ -n "${ACADOS_SOURCE_DIR:-}" ] && export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$ACADOS_SOURCE_DIR/lib"
    PY="$HOME/hmpc-venv/bin/python"; [ -x "$PY" ] || PY="$(command -v python3)"
    F=main_sim.py; [ "$1" = fast ] && F=main.py
    shift
    exec "$PY" "$HERE/src/mlcs_mpc/$F" "$@" ;;
rviz)
    source_ros ws
    exec rviz2 -d "$HERE/src/mlcs_mpc/launch/sim.rviz" ;;
go|halt)
    source_ros ws
    v=true; [ "$1" = halt ] && v=false
    ros2 topic pub --once /mpc/enabled std_msgs/msg/Bool "{data: $v}" >/dev/null
    echo "/mpc/enabled = $v"
    exit 0 ;;
status)
    source_ros ws
    # Foxy 의 ros2 topic echo 에는 --once 가 없다
    timeout 3 ros2 topic echo /sim/status 2>/dev/null | head -2 || true
    exit 0 ;;
track)
    [ -n "${2:-}" ] || { echo "사용법: ./sim.sh track <트랙> [v_max]"; exit 1; }
    CFG="$(mktemp /tmp/mlcs_sim_cfg.XXXX.yaml)"
    { echo "track: \"$2\""; [ -n "${3:-}" ] && printf 'mpcc:\n  v_max: %s\n' "$3"; } > "$CFG" ;;
*)
    CFG="${1:-$HERE/race.yaml}" ;;
esac
[ -f "$CFG" ] || { echo "설정 파일이 없습니다: $CFG"; exit 1; }

# race.sh 와 같은 규칙으로 읽는다: mpcc: 는 config/mpcc.yaml 에 있는 이름만, 트랙은 이름 또는 prep csv.
eval "$(python3 - "$CFG" "$HERE" "$OUT" <<'PY'
import os, shlex, sys
import numpy as np, yaml
cfg_path, root, out = sys.argv[1:]
c = yaml.safe_load(open(cfg_path, encoding='utf-8')) or {}
known = yaml.safe_load(open(os.path.join(root, 'src/mlcs_mpc/config/mpcc.yaml'), encoding='utf-8'))['/**']['ros__parameters']
h = dict(c.get('mpcc') or c.get('hmpc') or {})     # hmpc: 는 예전 이름
err = lambda m: (print(f'echo {shlex.quote("✗ " + m)} >&2; exit 1'), sys.exit(0))
bad = [k for k in h if k not in known]
if bad:
    err(f'mpcc 에 모르는 이름: {", ".join(bad)} (config/mpcc.yaml 에 있는 이름만)')
for k, v in h.items():
    if type(known[k]) is float and isinstance(v, int) and not isinstance(v, bool):
        h[k] = float(v)
t = str(c.get('track') or '')
if not t:
    err('track 을 적어 주세요')
path = t if os.path.isfile(t) else os.path.join(root, 'src/mlcs_mpc/mlcs_mpc/env/track/data', t, f'prep_{t}.csv')
if not os.path.isfile(path):
    err(f'트랙을 못 찾았습니다: {t}')
d = np.genfromtxt(path, delimiter=',', names=True)
yaml.safe_dump({'/**': {'ros__parameters': h}}, open(out, 'w'), allow_unicode=True)
v_max = float(h.get('v_max', known['v_max']))
lines = [f'트랙   {t}  (길이 {d["s"][-1]:.1f} m, 최소 반경 {1/max(abs(d["curvature"]).max(), 1e-9):.2f} m)',
         f'속도   v_max {v_max:.2f} m/s',
         f'모델   ' + ('학습 모델 ' + str(h['param_model']) if h.get('param_model') else 'mpcc.yaml 상수 파라미터')
         + f' — 차도 같은 식·같은 값 (마찰 {h.get("friction", known["friction"])})']
lines += [f'       {k} = {v}' for k, v in h.items() if k not in ('v_max', 'param_model')]
print(f'TRACK={shlex.quote(os.path.abspath(path))}')
print('SUMMARY=' + shlex.quote('\n'.join(lines)))
PY
)"

source_ros ws
ros2 pkg executables mlcs_mpc 2>/dev/null | grep -q sim_node || { echo "✗ 시뮬 워크스페이스가 없거나 오래됐습니다 — ./sim.sh build"; exit 1; }
PY_WORKER="$HOME/hmpc-venv/bin/python"
[ -x "$PY_WORKER" ] || PY_WORKER="$(command -v python3)"
if [ -z "${ACADOS_SOURCE_DIR:-}" ] && [ -d "$HOME/acados" ]; then
    export ACADOS_SOURCE_DIR="$HOME/acados"
fi
[ -n "${ACADOS_SOURCE_DIR:-}" ] && export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$ACADOS_SOURCE_DIR/lib"
LOG_CSV="${MLCS_SIM_LOGS:-$HERE/logs/sim}/sim_$(date +%Y%m%d_%H%M%S).csv"

echo "════════════════ 시뮬 ($(basename "$CFG")) ════════════════"
echo -e "$SUMMARY"
echo "기록   $LOG_CSV"
echo "════════════════════════════════════════════════"
echo "  출발은 ${MLCS_SIM_AUTOSTART:-3.0} 초 뒤 자동 (음수면 ./sim.sh go).  상태: ./sim.sh status"
echo "  mpcc 상태: ros2 topic echo /mpcc/status    워커 로그: /tmp/mlcs_mpcc_worker.log"
echo
BENCH=false; [ "${MLCS_SIM_BENCH:-0}" = 1 ] && BENCH=true
RVIZ=true; [ "${MLCS_SIM_NORVIZ:-0}" = 1 ] && RVIZ=false
ARGS=(track_csv:="$TRACK" extra_params:="$OUT" repo_root:="$HERE" worker_python:="$PY_WORKER"
      bench:="$BENCH" rviz:="$RVIZ" autostart_delay:="${MLCS_SIM_AUTOSTART:-3.0}"
      on_wall:="${MLCS_SIM_ONWALL:-warn}" log_csv:="$LOG_CSV")
# 빈 값은 launch 가 받지 않는다
[ -n "${MLCS_SIM_PARAMS:-}" ] && ARGS+=(sim_params:="$MLCS_SIM_PARAMS")
exec ros2 launch mlcs_mpc sim.launch.py "${ARGS[@]}"
