#!/usr/bin/env python3
"""
check_state.py — 기록하기 전에 /mpc/state 가 멀쩡한지 몇 초만 본다.

  ./drive.sh checkstate [초]

■ 왜 필요한가

  /mpc/state 에 퍼블리셔가 둘이면 waypoint_logger 가 두 궤적을 **번갈아**
  찍어서 기록이 통째로 못 쓰게 된다. 한 바퀴 다 돌고 나서 알면 늦다.
  2026-10-01 에 두 번 날렸다 (점간 중앙 2.4 m, 주행거리 18.7 km).

  원인은 여러 가지다 — mocap_bridge 가 둘, sim_bridge 가 같이 뜸, Motive
  에 강체가 둘, natnet 이 둘… 그래서 **원인을 짐작하지 않고 증상을 잰다.**

■ 판정

  한 궤적이면 연속한 두 점이 가깝다 (100Hz 에 1 m/s 면 1 cm).
  두 궤적이 번갈아 오면 그 간격이 '두 궤적 사이 거리' 로 벌어지고,
  **한 칸 건너뛰면** 다시 매끄러워진다. 그 차이로 가른다.
"""
import argparse
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Check(Node):
    def __init__(self, secs):
        super().__init__('check_state')
        self.pts = []
        self.t = []
        # mocap_bridge 는 기본 QoS(RELIABLE)로 낸다. waypoint_logger 와
        # 같은 설정으로 받아야 같은 걸 본다.
        self.create_subscription(Odometry, '/mpc/state', self._cb, 10)
        self.secs = secs

    def _cb(self, m):
        self.pts.append((m.pose.pose.position.x, m.pose.pose.position.y))
        self.t.append(self.get_clock().now().nanoseconds * 1e-9)


def main():
    p = argparse.ArgumentParser(description='/mpc/state 건강 점검')
    p.add_argument('secs', nargs='?', type=float, default=4.0)
    a = p.parse_args()

    rclpy.init()
    n = Check(a.secs)
    t0 = n.get_clock().now().nanoseconds * 1e-9
    while rclpy.ok() and (n.get_clock().now().nanoseconds * 1e-9 - t0) < a.secs:
        rclpy.spin_once(n, timeout_sec=0.1)

    # 퍼블리셔 수 (데몬 캐시를 안 타는 경로)
    try:
        pubs = n.get_publishers_info_by_topic('/mpc/state')
    except Exception:
        pubs = []
    xy = np.array(n.pts)
    n.destroy_node()
    rclpy.shutdown()

    print(f'■ /mpc/state  {a.secs:.0f}초 관찰')
    print(f'   퍼블리셔 {len(pubs)}개  (DDS 캐시라 죽은 노드가 남을 수 있음)'
          + ('  ← 확인 필요' if len(pubs) > 1 else ''))
    for pi in pubs:
        print(f'     · {pi.node_namespace.rstrip("/")}/{pi.node_name}')

    if len(xy) < 20:
        print(f'   ✗ 받은 메시지 {len(xy)}개 — mocap 연동부터 확인하세요.')
        print('     ./drive.sh mocap   으로 점검')
        sys.exit(1)

    dt = np.diff(n.t)
    hz = 1.0 / max(np.median(dt), 1e-9)
    d = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    d2 = np.linalg.norm(np.diff(xy[0::2], axis=0), axis=1)
    med, med2 = float(np.median(d)), float(np.median(d2))
    print(f'   {len(xy)}개 수신, {hz:.0f} Hz')
    print(f'   점간 거리 중앙 {med*100:.2f} cm   한 칸 건너뛰면 {med2*100:.2f} cm')
    print(f'   범위 x [{xy[:,0].min():+.2f}, {xy[:,0].max():+.2f}]  '
          f'y [{xy[:,1].min():+.2f}, {xy[:,1].max():+.2f}]')

    # 번갈아 오는가 — 건너뛰면 절반 이하로 줄면 의심.
    # ★ 퍼블리셔 수는 **보조 지표로만** 쓴다. DDS 디스커버리가 죽은 노드를
    #   한참 기억해서, 실제로는 하나인데 둘로 보일 수 있다 (실측).
    #   판정은 실제로 받은 데이터로 한다.
    interleaved = med > 0.03 and med2 < 0.5 * med
    if interleaved:
        print()
        print('   ✗ /mpc/state 에 궤적이 둘 섞여 있습니다.')
        print('     이대로 기록하면 못 씁니다. 먼저 정리하세요:')
        print('       ./drive.sh stop                     전부 내리고')
        print('       pgrep -af /lib/mlcs_mpc/            남은 노드 확인')
        print('       ./drive.sh mocap                    mocap 만 띄워 다시 점검')
        print('     Motive 에 강체가 둘 잡혀 있지 않은지도 보세요 '
              '(차 말고 다른 마커 뭉치).')
        sys.exit(1)

    print()
    if len(pubs) > 1:
        print(f'   ⚠ 퍼블리셔가 {len(pubs)}개로 보이지만 데이터는 한 궤적입니다.')
        print('     죽은 노드가 DDS 캐시에 남은 것일 수 있습니다 — 진행해도 됩니다.')
    print(f'   ✓ 한 궤적만 옵니다 — 기록해도 됩니다.')


if __name__ == '__main__':
    main()
