#!/usr/bin/env python3
"""
rally_limits — 랠리 모드에서 십자키로 조절하는 수동 조종 상한.

  ./drive.sh rally 로 쓴다. 원선회는 입력이 일정해야 하는데, 손으로 RT 와 스틱을
  중간에 고정하기는 어렵다. 그래서 **끝까지 당기고 끝까지 민 값이 곧 상한**이 되게
  하고, 그 상한을 십자키로 조금씩 옮긴다.

      위 / 아래       최대 속도 +/- speed_step
      오른 / 왼       최대 조향 +/- steer_step

  누를 때 한 칸만 움직인다. joy_node 의 autorepeat 가 같은 값을 계속 보내므로
  상승 엣지로 끊는다. 안 그러면 한 번 누른 것이 여러 칸이 된다.

  ROS 없이 test/test_algorithms.py 에서 돌리려고 노드와 분리했다.
"""

import math


class RallyLimits:

    PRESS = 0.5    # 십자키는 -1/0/+1 이다. 이 값을 넘으면 눌린 것으로 본다

    def __init__(self, speed, steer, speed_step, steer_step,
                 speed_limit, steer_limit):
        self.speed_step = float(speed_step)
        self.steer_step = float(steer_step)
        self.speed_limit = max(float(speed_limit), 0.0)
        self.steer_limit = max(float(steer_limit), 0.0)
        self.speed = min(max(float(speed), 0.0), self.speed_limit)
        self.steer = min(max(float(steer), 0.0), self.steer_limit)
        self._prev_x = 0
        self._prev_y = 0

    @classmethod
    def _dir(cls, raw):
        return (raw > cls.PRESS) - (raw < -cls.PRESS)

    def update(self, right, up):
        """right, up: 오른쪽·위가 + 인 십자키 값. 상한이 바뀌었으면 True."""
        x, y = self._dir(right), self._dir(up)
        before = (self.speed, self.steer)
        if y and y != self._prev_y:
            self.speed = self._move(
                self.speed, y * self.speed_step, self.speed_limit)
        if x and x != self._prev_x:
            self.steer = self._move(
                self.steer, x * self.steer_step, self.steer_limit)
        self._prev_x, self._prev_y = x, y
        return (self.speed, self.steer) != before

    @staticmethod
    def _move(value, delta, limit):
        # 0.05 를 거듭 더해도 1.1500000000000001 이 되지 않게 반올림한다
        return round(min(max(value + delta, 0.0), limit), 4)


def circle_estimate(speed, steer, wheelbase):
    """운동학 모델 기준 (선회반경 [m], 횡가속도 [m/s²]). 직진이면 반경은 inf."""
    t = math.tan(abs(steer))
    if t < 1e-6:
        return math.inf, 0.0
    return wheelbase / t, speed * speed * t / wheelbase
