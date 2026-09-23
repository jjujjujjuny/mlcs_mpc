#!/usr/bin/env python3
"""
track_info — 트랙을 주행 없이 점검한다.

  ros2 run mlcs_mpc track_info --ros-args \
      --params-file <config/track.yaml>

새 트랙을 waypoints/ 에 넣고 track.yaml 을 고쳤으면 **제일 먼저 이걸** 돌린다.
차를 꺼내기 전에 알 수 있는 건 여기서 다 알아낸다.

■ 왜 필요한가

  트랙이 잘못돼 있으면 증상이 "제어가 이상하다" 로 나타난다. 게인을 만지며
  한참 헤매다가 원인이 웨이포인트였던 경우가 흔하다. 그래서 주행 전에
  기하부터 확인한다:

    · 점 간격이 균일한가        — 들쭉날쭉하면 헤딩/곡률 추정이 튄다
    · 폐곡선인가                — closed_loop 설정과 맞는지
    · 최소 선회반경 ≥ 차량 한계 — 못 도는 코너가 있으면 무조건 이탈한다
    · 안전 경계까지 여유가 얼마인가

■ 안전 여유가 핵심이다

  safety_node 는 현재 위치가 [min+margin, max-margin] 안인지 본다.
  트랙이 경계에 가까우면, 그 여유가 곧 **허용 가능한 최대 횡오차**다.
  이 값이 작으면 차가 멀쩡히 도는데도 safety 가 세운다 — 그럴 때
  게인을 의심하기 전에 이 숫자를 먼저 보라는 뜻이다.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node

from .path_manager import PathManager


class TrackInfo(Node):

    def __init__(self):
        super().__init__('track_info')

        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('target_speed', 0.7)
        self.declare_parameter('curvature_slowdown', 1.5)
        self.declare_parameter('closed_loop', True)
        self.declare_parameter('mu', 0.35)

        # 차량 한계 — 못 도는 코너가 있는지 판정용
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steer', 0.36)

        # safety_node 와 같은 값을 받아 여유를 계산한다
        self.declare_parameter('x_min', -3.0)
        self.declare_parameter('x_max', 3.0)
        self.declare_parameter('y_min', -3.0)
        self.declare_parameter('y_max', 3.0)
        self.declare_parameter('margin', 0.5)
        self.declare_parameter('lookahead', 0.7)

    def run(self):
        def p(n):
            return self.get_parameter(n).value

        wp = str(p('waypoint_file'))
        if not wp:
            self.get_logger().error(
                'waypoint_file 이 비어 있습니다. config/track.yaml 을 주세요.')
            return 1

        path = PathManager(
            wp, target_speed=float(p('target_speed')),
            curvature_slowdown=float(p('curvature_slowdown')),
            closed_loop=bool(p('closed_loop')),
            logger=self.get_logger())
        if not path.ok:
            return 1

        xy, th, kap, v = path.xy, path.heading, path.kappa, path.v_ref
        d = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        gap = float(np.linalg.norm(xy[0] - xy[-1]))
        R = 1.0 / np.maximum(np.abs(kap), 1e-9)

        L, dmax = float(p('wheelbase')), float(p('max_steer'))
        r_car = L / max(math.tan(dmax), 1e-6)

        print()
        print('■ 기하')
        print(f'    점 {len(xy)}개   길이 {path.total_s:.2f} m')
        print(f'    간격    중앙 {np.median(d)*100:5.1f} cm   '
              f'최소 {d.min()*100:.1f}   최대 {d.max()*100:.1f}')
        print(f'    시작–끝 {gap*100:.1f} cm  '
              f'({"닫힘" if gap < 3*np.median(d) else "열림"}, '
              f'설정은 {"폐곡선" if p("closed_loop") else "개곡선"})')
        print(f'    범위    x [{xy[:,0].min():6.2f}, {xy[:,0].max():6.2f}]   '
              f'y [{xy[:,1].min():6.2f}, {xy[:,1].max():6.2f}]')
        print(f'    크기    {xy[:,0].max()-xy[:,0].min():.2f} × '
              f'{xy[:,1].max()-xy[:,1].min():.2f} m')

        print()
        print('■ 선회')
        i = int(np.argmin(R))
        print(f'    최소 반경 {R.min():.3f} m  @ 점 {i} '
              f'(x={xy[i,0]:.2f}, y={xy[i,1]:.2f})')
        print(f'    차량 한계 {r_car:.3f} m  '
              f'(wheelbase {L}, max_steer {dmax} rad = {math.degrees(dmax):.1f}°)')
        if R.min() < r_car:
            print(f'    ✗ 못 도는 코너가 있다 — 반경이 차량 한계보다 작다')
        else:
            print(f'    ✓ 여유 {(R.min()-r_car)*100:.0f} cm')

        print()
        print('■ 속도')
        print(f'    v_ref {v.min():.2f} ~ {v.max():.2f} m/s  '
              f'(target {p("target_speed")}, 곡률 감속 적용)')

        # ── 안전 여유 — safety_node.py 의 판정식 그대로 ──────────────
        xmin, xmax = float(p('x_min')), float(p('x_max'))
        ymin, ymax = float(p('y_min')), float(p('y_max'))
        m, la = float(p('margin')), float(p('lookahead'))

        slack = np.minimum.reduce([
            xy[:, 0] - (xmin + m), (xmax - m) - xy[:, 0],
            xy[:, 1] - (ymin + m), (ymax - m) - xy[:, 1]])
        j = int(np.argmin(slack))

        px = xy[:, 0] + v * np.cos(th) * la
        py = xy[:, 1] + v * np.sin(th) * la
        pslack = np.minimum.reduce([px - xmin, xmax - px,
                                    py - ymin, ymax - py])

        print()
        print('■ 안전 경계')
        print(f'    설정   x [{xmin}, {xmax}]  y [{ymin}, {ymax}]  '
              f'margin {m}  lookahead {la}s')
        print(f'    ① 현재위치 판정 — 최소 여유 {slack.min()*100:5.1f} cm  '
              f'@ 점 {j} (x={xy[j,0]:.2f}, y={xy[j,1]:.2f})')
        print(f'    ② 예측 판정     — 최소 여유 {pslack.min()*100:5.1f} cm')
        n_tight = int((slack < 0.5).sum())
        print(f'    여유 50cm 미만 구간 {n_tight}개 / {len(xy)} '
              f'({n_tight/len(xy)*100:.0f}%)')

        print()
        if slack.min() <= 0:
            print('    ✗ 트랙이 경계 밖으로 나간다 — 출발하자마자 정지한다.')
        elif slack.min() < 0.2:
            print(f'    ✗ 허용 횡오차가 {slack.min()*100:.0f} cm 뿐이다. '
                  '거의 확실히 safety 에 걸린다.')
        elif slack.min() < 0.5:
            print(f'    ⚠ 허용 횡오차 {slack.min()*100:.0f} cm. '
                  '첫 주행에서 걸릴 수 있다.')
            print('      차가 잘 도는데 갑자기 서면 게인이 아니라 이것부터 의심할 것.')
            print('      로그에 "경계 이탈 (x=..., y=...)" 이 찍힌다.')
        else:
            print(f'    ✓ 허용 횡오차 {slack.min()*100:.0f} cm — 여유 있다.')
        print()
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = TrackInfo()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    main()
