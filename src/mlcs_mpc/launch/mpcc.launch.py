#!/usr/bin/env python3
"""
mpcc.launch.py — MPCC 노드 + 위치 브릿지 (mocap_bridge | lidar_bridge | 없음 = 시뮬).

  ros2 launch mlcs_mpc mpcc.launch.py bench:=true

★ 기본이 bench 다 (속도 항상 0). 달리려면 bench:=false 를 직접 줘야 한다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mlcs_mpc')

    def _nodes(context, *_a, **_k):
        bench = LaunchConfiguration('bench').perform(context).lower() in ('1', 'true', 'yes')
        over = {'bench': bench, 'require_enable': True}
        track = LaunchConfiguration('track_csv').perform(context).strip()
        if track:       # 비우면 mpcc.yaml 을 따른다
            over['track_csv'] = track
        extra = LaunchConfiguration('extra_params').perform(context).strip()
        vmax = LaunchConfiguration('v_max').perform(context).strip()
        if vmax:
            over['v_max'] = float(vmax)
        for k in ('repo_root', 'worker_python'):
            v = LaunchConfiguration(k).perform(context).strip()
            if v:
                over[k] = v
        src = LaunchConfiguration('source').perform(context).strip()
        if src == 'sim':
            # mlcs_sim 의 sim_node 가 /mpc/state·/mocap/valid·/sensors/core 를 직접 낸다
            bridge = None
            over['sim_mode'] = True     # 시뮬용 한계 (ay_max_sim)
        elif src == 'lidar':
            # require_ready: RViz 로 시작 위치를 찍기 전(relocalize)에는 출발하지 않는다
            bridge = Node(package='mlcs_mpc', executable='lidar_bridge', name='lidar_bridge',
                          output='screen', parameters=[{'require_ready': True}])
        else:
            bridge = Node(package='mlcs_mpc', executable='mocap_bridge', name='mocap_bridge',
                          output='screen', parameters=[os.path.join(pkg, 'config', 'mocap.yaml')])
        return ([bridge] if bridge else []) + [
            Node(package='mlcs_mpc', executable='mpcc_node', name='mpcc_node',
                 output='screen',
                 parameters=[os.path.join(pkg, 'config', 'mpcc.yaml')]
                 + ([extra] if extra else []) + [over]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('bench', default_value='true',
                              description='true 면 속도 0. 거치대 확인용'),
        DeclareLaunchArgument('source', default_value='mocap',
                              description='위치 출처: mocap | lidar (Cartographer /tracked_pose) | sim (mlcs_sim)'),
        DeclareLaunchArgument('repo_root', default_value='',
                              description='저장소 경로. 비우면 mpcc.yaml (~/mlcs_mpc)'),
        DeclareLaunchArgument('worker_python', default_value='',
                              description='OCP 워커 파이썬. 비우면 mpcc.yaml (~/hmpc-venv/bin/python)'),
        DeclareLaunchArgument('extra_params', default_value='',
                              description='mpcc.yaml 뒤에 덮어쓸 파라미터 파일 (race.sh 가 쓴다)'),
        DeclareLaunchArgument('v_max', default_value='',
                              description='속도 상한 m/s. 비우면 mpcc.yaml'),
        DeclareLaunchArgument('track_csv', default_value='',
                              description='prep 형식 트랙 (절대경로). 비우면 mpcc.yaml'),
        OpaqueFunction(function=_nodes),
    ])
