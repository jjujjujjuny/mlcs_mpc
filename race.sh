#!/usr/bin/env bash
# ============================================================================
#  race.sh — race.yaml 을 읽어 hmpc 를 띄운다.
#
#     ./race.sh               race.yaml
#     ./race.sh other.yaml    다른 설정 파일
#     MLCS_RACE_BENCH=1 ./race.sh   이번만 bench (속도 0) — race.yaml 의 drive 무시
#
#  자율 모드로 시작해 조이스틱 B 를 누르면 출발. LB = 수동(비상정지), 다시 LB = 바로 자율 재개.
# ============================================================================
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="${1:-$HERE/race.yaml}"
[ -f "$CFG" ] || { echo "설정 파일이 없습니다: $CFG"; exit 1; }
OUT=/tmp/mlcs_race_params.yaml

# 설정을 읽어 검사하고, hmpc 파라미터 파일과 환경 변수를 만든다.
eval "$(python3 - "$CFG" "$HERE" "$OUT" <<'PY'
import os, shlex, sys
import numpy as np, yaml
cfg_path, root, out = sys.argv[1:]
c = yaml.safe_load(open(cfg_path, encoding='utf-8')) or {}
base = yaml.safe_load(open(os.path.join(root, 'src/mlcs_mpc/config/hmpc.yaml'), encoding='utf-8'))
known = base['/**']['ros__parameters']
h = dict(c.get('hmpc') or {})
err = lambda m: (print(f'echo {shlex.quote("✗ " + m)} >&2; exit 1'), sys.exit(0))

bad = [k for k in h if k not in known]
if bad:
    err(f'race.yaml hmpc 에 모르는 이름: {", ".join(bad)} (config/hmpc.yaml 에 있는 이름만)')
for k, v in h.items():
    if type(known[k]) is float and isinstance(v, int) and not isinstance(v, bool):
        h[k] = float(v)          # 1 이라고 써도 ROS 가 float 로 받게
v_max = float(h.get('v_max', known['v_max']))
# 상한은 차가 실제로 낼 수 있는 속도: vesc_driver 가 speed_max (ERPM) 에서 자른다.
# 젯슨의 vesc.yaml 을 읽고, 없으면 (메인 PC) 패치 값 23250 / 4781.5 = 4.86 m/s.
vy_path = os.path.expanduser('~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml')
erpm_max, gain = 23250.0, 4781.5
try:
    vp = next(iter(yaml.safe_load(open(vy_path)).values()))['ros__parameters']
    erpm_max, gain = float(vp['speed_max']), float(vp['speed_to_erpm_gain'])
except Exception:
    pass
v_motor = erpm_max / gain
if not 0.0 < v_max <= v_motor:
    err(f'v_max {v_max} 는 0~{v_motor:.2f} m/s 범위여야 합니다 (vesc.yaml speed_max {erpm_max:.0f} ERPM / {gain:g})')

t = str(c.get('track') or '')
if not t:
    err('track 을 적어 주세요')
path = t if os.path.isfile(t) else os.path.join(root, 'lab_map/data/tracks', t, f'prep_{t}.csv')
if not os.path.isfile(path):
    err(f'트랙을 못 찾았습니다: {t}')
d = np.genfromtxt(path, delimiter=',', names=True)
r_min = 1.0 / max(np.abs(d['curvature']).max(), 1e-9)
# 가장 좁은 코너에서 접지력만으로 돌 수 있는 속도 (마찰 × g). MPC 가 이 아래로 알아서 감속한다
fric = float(h.get('friction', known['friction']))
v_corner = (fric * 9.81 * r_min) ** 0.5

hm = str(h.get('hyper_model') or '')
if hm and not os.path.isfile(hm if os.path.isabs(hm) else os.path.join(root, hm)):
    err(f'hyper_model 파일이 없습니다: {hm}')

drive = bool(c.get('drive', False))
# MLCS_RACE_BENCH=1 → race.yaml 을 안 고치고 이번 한 번만 bench (노트북 ./car.sh bench)
if os.environ.get('MLCS_RACE_BENCH') == '1':
    drive = False
source = str(c.get('source') or 'mocap')
if source not in ('lidar', 'mocap'):
    err(f'source 는 lidar 또는 mocap: {source}')
# 장애물: off | watch (찾기만 — RViz 로 확인, MPC 는 모름) | lidar (찾아서 피함)
#   lidar_obstacles 노드 → /obstacle/pose → hmpc 의 상대 로직 (opponent.py)
obs = str(c.get('obstacles') or 'off')
if obs not in ('off', 'watch', 'lidar'):
    err(f'obstacles 는 off, watch, lidar 중 하나: {obs}')
if obs != 'off' and source != 'lidar':
    err(f'obstacles: {obs} 는 source: lidar 에서만 (맵과 비교해 찾는다)')
if obs == 'lidar':
    if h.get('opponent_topic'):
        err('obstacles: lidar 이면 hmpc.opponent_topic 은 비워 두세요 (자동으로 /obstacle/pose)')
    h['opponent_topic'], h['opponent_msg'] = '/obstacle/pose', 'pose'
# 지연 보상의 추가 지연: 라이다 주행이면 맵 정렬에서 잰 time_lag (Cartographer 가 스탬프를 외삽해 찍어
#   스탬프로는 안 보인다). race.yaml 에 delay_extra 를 직접 쓰면 그 값
if source == 'lidar' and 'delay_extra' not in h:
    mf = os.path.join(root, 'maps', f'{c.get("map") or ""}.mocap.yaml')
    try:
        h['delay_extra'] = float(yaml.safe_load(open(mf))['time_lag_s'])
    except Exception:
        pass
yaml.safe_dump({'/**': {'ros__parameters': h}}, open(out, 'w'), allow_unicode=True)
mp = c.get('map') or ''
if isinstance(mp, int):
    err(f'map 을 따옴표로 감싸 주세요 (예: map: "1108_5"). 지금은 숫자 {mp} 로 읽혔습니다')
mp = str(mp)
if source == 'lidar':
    if not os.path.isfile(os.path.join(root, 'maps', f'{mp}.pbstream')):
        err(f'라이다 주행 맵이 없습니다: maps/{mp}.pbstream')
    if '_mocap' in t:
        err(f'라이다 주행인데 mocap 프레임 트랙입니다: {t}')
    if h.get('opponent_topic') and obs != 'lidar':
        print('echo "⚠ 라이다 주행에서 opponent_topic 을 켰습니다 — 상대 pose 가 map 좌표계가 아니면 통로가 엉뚱한 곳에 생깁니다" >&2')
elif '_mocap' not in t:
    print('echo "⚠ mocap 주행인데 트랙 이름에 _mocap 이 없습니다 — 좌표계를 확인하세요" >&2')
log = str(c.get('log') or '')
print(f'export MLCS_HMPC_TRACK={shlex.quote(os.path.abspath(path))}')
print(f'export MLCS_HMPC_PARAMS={shlex.quote(out)}')
print(f'export MLCS_HMPC_DRIVE={1 if drive else 0}')
print(f'export MLCS_HMPC_SOURCE={source}')
print(f'export MLCS_HMPC_MAP={shlex.quote(mp)}')
print(f'export MLCS_OBSTACLES={0 if obs == "off" else 1}')
print(f'RACE_LOG={shlex.quote(log)}')
lines = [f'위치   {source}' + (f'  (맵 {mp})' if source == 'lidar' else ''),
         f'트랙   {t}  (길이 {d["s"][-1]:.1f} m, 최소 반경 {r_min:.2f} m)',
         f'모드   {"★ 실제 주행" if drive else "bench (속도 0)"}' + (f'  기록 {log}' if log else '  기록 안 함'),
         f'속도   v_max {v_max:.2f} m/s (모터 상한 {v_motor:.2f}) — 최소 반경 코너는 약 {v_corner:.2f} m/s 이하로 MPC 가 감속',
         f'모델   {"★ HyperPM 시변 " + os.path.basename(hm) if hm else "상수 파라미터 (hmpc.yaml)"}',
         f'장애물 ' + {'off': '끔', 'watch': '찾기만 (RViz /obstacle/markers — MPC 는 안 피함)',
                       'lidar': '★ 라이다로 찾아 피함 (lidar_obstacles → /obstacle/pose)'}[obs]]
lines += [f'       {k} = {v}' for k, v in h.items() if k not in ('v_max', 'hyper_model', 'opponent_topic', 'opponent_msg')]
if v_max > 3.0:
    lines.append('⚠ v_max 3 m/s 초과 — 동정 데이터는 3 m/s 까지다. 모델 밖이니 offline_ocp 로 먼저 확인하고 조금씩 올릴 것')
print('RACE_SUMMARY=' + shlex.quote('\n'.join(lines)))
PY
)"

echo "════════════════ race.yaml ════════════════"
echo -e "$RACE_SUMMARY"
echo "════════════════════════════════════════════"
ARGS=()
[ -n "$RACE_LOG" ] && ARGS+=(--log "$RACE_LOG")
# Ctrl+C 는 drive.sh 가 받아 정리한다. 여기서는 끝난 뒤 채점을 이어서 돌린다.
# ★ trap '' 으로 무시하면 자식이 SIGINT 무시를 물려받아 drive.sh 정리가 안 돈다. ':' 로 둔다.
trap ':' INT
set +e
"$HERE/drive.sh" hmpc "${ARGS[@]}"
trap - INT
if [ -n "$RACE_LOG" ] && [ "$MLCS_HMPC_SOURCE" = lidar ]; then
    echo
    echo "════════════════ 채점 (mocap 정답) ════════════════"
    "$HERE/drive.sh" racereport || echo "채점을 못 했습니다 — 위 메시지를 보세요"
fi
