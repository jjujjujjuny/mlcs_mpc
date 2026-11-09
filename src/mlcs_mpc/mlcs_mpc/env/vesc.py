"""
vesc — 차의 VESC 튜닝값 (f1tenth_stack/config/vesc.yaml). 시뮬레이터가 /drive 를 차와 똑같이 자른다.

  읽는 순서: 젯슨의 vesc.yaml (~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml) → 없으면
  third_party/patches/f1tenth_system-servo-offset.patch 의 값 (아래 PATCHED, race.sh 와 같은 규칙).

    속도   erpm = speed_to_erpm_gain · speed + offset  → [speed_min, speed_max] 에서 자른다 (모터 상한 ≈ 4.86 m/s)
    조향   servo = steering_angle_to_servo_gain · 명령 + offset → [servo_min, servo_max] 에서 자른다
    상태   /sensors/core state.speed = erpm (바퀴 속도로 되돌릴 때 같은 gain 을 쓴다)
"""
import os

DEFAULT_PATH = os.path.expanduser('~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml')

# third_party/patches/f1tenth_system-servo-offset.patch (2026-11 실측 캘리브레이션)
PATCHED = dict(speed_to_erpm_gain=4781.5, speed_to_erpm_offset=0.0,
               steering_angle_to_servo_gain=-0.8845, steering_angle_to_servo_offset=0.5077,
               speed_min=-23250.0, speed_max=23250.0, servo_min=0.08, servo_max=0.87)


class Vesc:

    def __init__(self, **p):
        v = dict(PATCHED)
        v.update({k: float(x) for k, x in p.items() if k in PATCHED})
        self.__dict__.update(v)
        self.source = 'patch'

    @classmethod
    def load(cls, path=DEFAULT_PATH):
        """vesc.yaml 이 있으면 그 값, 없으면 패치 값."""
        if path and os.path.isfile(path):
            import yaml
            d = yaml.safe_load(open(path, encoding='utf-8')) or {}
            p = next(iter(d.values()), {}).get('ros__parameters', {})
            v = cls(**p)
            v.source = path
            return v
        return cls()

    # ── 명령 → 차가 실제로 받는 값 ─────────────────────────────────────
    def speed(self, cmd):
        """속도 명령 m/s → VESC 가 자른 뒤의 속도 m/s."""
        erpm = self.speed_to_erpm_gain * cmd + self.speed_to_erpm_offset
        erpm = min(max(erpm, self.speed_min), self.speed_max)
        return (erpm - self.speed_to_erpm_offset) / self.speed_to_erpm_gain

    def steer(self, cmd):
        """조향 명령 rad → 서보 한계로 자른 뒤의 명령 rad (gain 이 음수라 왼쪽·오른쪽 한계가 다르다)."""
        servo = self.steering_angle_to_servo_gain * cmd + self.steering_angle_to_servo_offset
        servo = min(max(servo, self.servo_min), self.servo_max)
        return (servo - self.steering_angle_to_servo_offset) / self.steering_angle_to_servo_gain

    def erpm(self, wheel_speed):
        """바퀴 속도 m/s → /sensors/core state.speed."""
        return self.speed_to_erpm_gain * wheel_speed + self.speed_to_erpm_offset

    @property
    def v_motor(self):
        return (self.speed_max - self.speed_to_erpm_offset) / self.speed_to_erpm_gain

    @property
    def steer_limits(self):
        """(오른쪽, 왼쪽) 명령 한계 rad."""
        a = sorted([self.steer(-10.0), self.steer(10.0)])
        return a[0], a[1]
