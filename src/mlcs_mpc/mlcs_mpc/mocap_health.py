#!/usr/bin/env python3
"""
mocap_health — mocap 프레임이 믿을 만한가. ROS 없이 시험한다 (test/test_hmpc_core.py).

■ 왜 필요한가

  스트림이 끊기면 timeout 으로 잡히지만, **마커를 가리면 잡히지 않는다.** natnet 은 멈춘
  pose 를 새 스탬프로 계속 보내고, 추적이 돌아오는 순간 pose 가 튄다.
  2026-11-06 실차: 7.4 초 동안 pose 가 그대로인데 /mocap/valid 는 true 였고 차는 달렸다.
  튄 직후 칼만 필터 속도는 ±10 m/s 까지 갔다.

■ 판정

  ① 멈춤: 위치·자세가 완전히 같은 프레임이 freeze_frames 개 이어짐
     (같은 날 정상 추적 로그 3개, 약 2.3 분에서 3 프레임 이상 이어진 적이 없다.
      세워 둔 차도 값이 조금씩은 변한다)
  ② 튐: 프레임 간 이동이 max_speed, yaw 변화가 max_yaw_rate 를 넘음
     ★ 시간 간격은 min_dt 아래로 안 내리고 잡음 여유(pos_tol, yaw_tol)를 더한다.
       natnet 메시지가 몰려 오면 스탬프 간격이 1~2 ms 가 되어, 1 m/s 의 정상 이동 1 cm 가
       6 m/s 로 보였다. 2026-11-08 1.0 m/s 주행에서 6번 오탐해 차가 섰다.

  한 번 나빠지면 recover_time 동안 깨끗해야 다시 믿는다. 경계에서 들락거리면 위험하다.
"""
import math


class PoseHealth:

    def __init__(self, freeze_frames=5, max_speed=6.0, max_yaw_rate=15.0, recover_time=0.3,
                 min_dt=0.01, pos_tol=0.02, yaw_tol=math.radians(1.0)):
        self.freeze_frames = freeze_frames
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.recover_time = recover_time
        self.min_dt = min_dt
        self.pos_tol = pos_tol
        self.yaw_tol = yaw_tol
        self.prev = None
        self.same = 0
        self.bad_until = None   # 이 스탬프까지 깨끗해야 복귀
        self.reason = ''

    def update(self, stamp, x, y, z, qx, qy, qz, qw):
        """(믿을 만한가, 나빠진 이유). 이유는 새로 나빠진 프레임에서만 준다."""
        pose = (x, y, z, qx, qy, qz, qw)
        fresh = ''
        if self.prev is not None:
            pstamp, ppose = self.prev
            dt = max(stamp - pstamp, self.min_dt)
            self.same = self.same + 1 if pose == ppose else 0
            dist = math.hypot(x - ppose[0], y - ppose[1])
            dyaw = abs(_wrap(_yaw(pose) - _yaw(ppose)))
            if self.same + 1 >= self.freeze_frames:
                fresh = f'pose 가 {self.same + 1} 프레임 그대로'
            elif dist > self.max_speed * dt + self.pos_tol:
                fresh = f'위치 튐 {dist * 100:.1f} cm'
            elif dyaw > self.max_yaw_rate * dt + self.yaw_tol:
                fresh = f'자세 튐 {math.degrees(dyaw):.0f}°'
        self.prev = (stamp, pose)

        if fresh:
            newly = self.bad_until is None
            self.bad_until = stamp + self.recover_time
            return False, (fresh if newly else '')
        if self.bad_until is not None:
            if stamp < self.bad_until:
                return False, ''
            self.bad_until = None
        return True, ''


def _yaw(p):
    return math.atan2(2.0 * (p[6] * p[5] + p[3] * p[4]), 1.0 - 2.0 * (p[4] ** 2 + p[5] ** 2))


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi
