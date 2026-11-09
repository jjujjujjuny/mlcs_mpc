#!/usr/bin/env python3
"""
hmpc.launch.py — HyperMPC OCP 노드 + mocap_bridge.

  ros2 launch mlcs_mpc hmpc.launch.py bench:=true

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
        if track:       # 비우면 hmpc.yaml 을 따른다
            over['track_csv'] = track
        extra = LaunchConfiguration('extra_params').perform(context).strip()
        vmax = LaunchConfiguration('v_max').perform(context).strip()
        if vmax:
            over['v_max'] = float(vmax)
        src = LaunchConfiguration('source').perform(context).strip()
        if src == 'lidar':
            # 시작 위치 찍기(require_ready)는 끔 — Cartographer 가 출발 자리에서 알아서 잡는다 (2026-11-09).
            #   위치 튐도 정지하지 않고 무시 (stop_on_jump 끔). 위치가 끊기면(timeout) 여전히 정지.
            bridge = Node(package='mlcs_mpc', executable='lidar_bridge', name='lidar_bridge',
                          output='screen', parameters=[{'require_ready': False,
                                                        'stop_on_jump': False}])
        else:
            bridge = Node(package='mlcs_mpc', executable='mocap_bridge', name='mocap_bridge',
                          output='screen', parameters=[os.path.join(pkg, 'config', 'mocap.yaml')])
        nodes = [bridge]
        if src == 'lidar' and LaunchConfiguration('obstacles').perform(context).lower() in ('1', 'true'):
            # 라이다 장애물 — 저장한 맵과 비교 (obstacle_detect.py). /obstacle/pose → opponent_topic
            nodes.append(Node(package='mlcs_mpc', executable='lidar_obstacles', name='lidar_obstacles',
                              output='screen',
                              parameters=[{'map_yaml': LaunchConfiguration('map_yaml').perform(context),
                                           'track_csv': track}]))
        return nodes + [
            Node(package='mlcs_mpc', executable='hmpc_node', name='hmpc_node',
                 output='screen',
                 parameters=[os.path.join(pkg, 'config', 'hmpc.yaml')]
                 + ([extra] if extra else []) + [over]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('bench', default_value='true',
                              description='true 면 속도 0. 거치대 확인용'),
        DeclareLaunchArgument('source', default_value='mocap',
                              description='위치 출처: mocap | lidar (Cartographer /tracked_pose)'),
        DeclareLaunchArgument('extra_params', default_value='',
                              description='hmpc.yaml 뒤에 덮어쓸 파라미터 파일 (race.sh 가 쓴다)'),
        DeclareLaunchArgument('v_max', default_value='',
                              description='속도 상한 m/s. 비우면 hmpc.yaml'),
        DeclareLaunchArgument('obstacles', default_value='false',
                              description='source lidar 에서 true 면 lidar_obstacles 를 띄운다'),
        DeclareLaunchArgument('map_yaml', default_value='', description='lidar_obstacles 가 쓸 맵 yaml'),
        DeclareLaunchArgument('track_csv', default_value='',
                              description='prep 형식 트랙 (절대경로). 비우면 hmpc.yaml'),
        OpaqueFunction(function=_nodes),
    ])
