#!/usr/bin/env python3
"""
core — mpcc_node 의 ROS 없는 계산 (상태 변환·해 검사·명령 변환). test/test_mpcc_core.py 로 시험한다.

  OCP 상태 x = [s, n, mu, vx, vy, r, friction, wheel_speed, delta]
  모델은 무게중심 기준이고 /mpc/state 는 뒤축 기준이다.
"""
import math

import numpy as np


from mlcs_mpc.env.track import Track, wrap  # noqa: F401  (예전 import 경로)


def make_state(track, pose, twist, wheel_speed, delta, lr, v_floor, friction):
    """/mpc/state (뒤축) → OCP x (무게중심).

    v_x 는 v_floor 아래로 안 내린다. 저자 비용의 atan(vy/vx) 와 슬립 분모는 v_x = 0 에서
    발산한다. 서 있어도 조향은 계산되고, 출발 속도 명령은 ws_ref 하한이 아니라 비용이 정한다.
    """
    x, y, yaw = pose
    vx, vy, r = twist
    cx = x + lr * math.cos(yaw)
    cy = y + lr * math.sin(yaw)
    s, n, mu = track.frenet(cx, cy, yaw)
    return np.array([s, n, mu, max(vx, v_floor), vy + lr * r, r, friction,
                     max(wheel_speed, v_floor), delta])


def solution_ok(status, u, x1):
    """내보내도 되는 해인가. status 0 이고 전부 유한해야 한다."""
    return int(status) == 0 and bool(np.all(np.isfinite(u))) and bool(np.all(np.isfinite(x1)))


def steer_to_command(actual, map_cmd, map_act):
    """실제 조향 → 서보 명령. 맵은 원선회로 잰 좌우 평균이다 (0 에서 시작)."""
    return math.copysign(float(np.interp(abs(actual), map_act, map_cmd)), actual)


def command_to_steer(cmd, map_cmd, map_act, max_cmd=None):
    """서보 명령 → 실제 조향 (steer_to_command 의 반대). 시뮬레이터가 쓴다. 맵 밖은 끝 기울기로 늘린다."""
    if max_cmd is not None:
        cmd = max(-max_cmd, min(max_cmd, cmd))
    a = abs(cmd)
    if a <= map_cmd[-1]:
        act = float(np.interp(a, map_cmd, map_act))
    else:
        act = map_act[-1] + (a - map_cmd[-1]) * (map_act[-1] - map_act[-2]) / (map_cmd[-1] - map_cmd[-2])
    return math.copysign(act, cmd)


def to_command(u, v_max, max_steer, prev_steer, max_dsteer, map_cmd, map_act):
    """해 → (속도, 조향 명령). 솔버 경계와 별개로 여기서 다시 묶는다.

    u[1] 은 모델의 실제 조향이다. max_steer, 변화율, prev_steer 는 명령 단위다.
    """
    speed = float(np.clip(u[0], 0.0, v_max))
    cmd = steer_to_command(u[1], map_cmd, map_act)
    steer = float(np.clip(cmd, prev_steer - max_dsteer, prev_steer + max_dsteer))
    return speed, float(np.clip(steer, -max_steer, max_steer))
