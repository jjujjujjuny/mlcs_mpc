#!/usr/bin/env python3
"""
stanley.launch.py — Stanley 자율주행 (mocap 위치추정).

  ros2 launch mlcs_mpc stanley.launch.py waypoints:=<경로.csv>

MPC 대신 Stanley 를 쓰는 것 말고는 car_mpc.launch.py 와 같다 —
mocap_bridge, safety_node 구성이 동일하므로 둘을 바꿔 끼우며
같은 트랙에서 비교할 수 있다.

★ 기본은 정지 상태다. 준비되면:
    ros2 topic pub --once /mpc/enabled std_msgs/msg/Bool "{data: true}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mlcs_mpc')
    stanley_cfg = os.path.join(pkg, 'config', 'stanley.yaml')
    mocap_cfg = os.path.join(pkg, 'config', 'mocap.yaml')
    safety_cfg = os.path.join(pkg, 'config', 'safety.yaml')
    # ★ 지금 달릴 트랙 — 트랙마다 바뀌는 것만 들어 있다 (경로/속도/경계).
    #   safety_cfg 와 stanley_cfg 뒤에 실어서 이 파일이 덮어쓰게 한다.
    track_cfg = os.path.join(pkg, 'config', 'track.yaml')

    args = [
        # ★ 아래 셋의 기본값이 빈 문자열인 것은 의도적이다.
        #   값을 주면 track.yaml 을 덮어쓰고, 안 주면 track.yaml 이 이긴다.
        #   기본값을 '0.7' 같은 실제 값으로 두면 인자를 안 줘도 항상
        #   덮어써 버려서 track.yaml 의 값이 무시된다.
        DeclareLaunchArgument('waypoints', default_value='',
                              description='웨이포인트 CSV. 파일명만 줘도 된다. '
                                          '비우면 track.yaml 을 따른다'),
        DeclareLaunchArgument('target_speed', default_value='',
                              description='목표 속도 (m/s). 비우면 track.yaml'),
        DeclareLaunchArgument('k_e', default_value='',
                              description='Stanley 횡오차 게인. 비우면 stanley.yaml'),
        DeclareLaunchArgument('bench', default_value='false',
                              description='true 면 속도 0 — 거치대 확인용'),
        DeclareLaunchArgument('safety', default_value='true'),
        DeclareLaunchArgument('mocap', default_value='true',
                              description='mocap_bridge 를 여기서 띄운다'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='track_viz 를 띄운다 (RViz 용 토픽)'),
        DeclareLaunchArgument('track_dir', default_value='',
                              description='경계까지 그리려면 원본 트랙 폴더'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz 도 같이 띄운다 (젯슨에선 보통 false)'),
    ]

    def _nodes(context, *_a, **_k):
        def arg(name):
            return LaunchConfiguration(name).perform(context).strip()

        # 준 것만 덮어쓴다 (위 DeclareLaunchArgument 주석 참고)
        over = {
            'bench': arg('bench').lower() in ('1', 'true', 'yes'),
            'debug': arg('debug').lower() in ('1', 'true', 'yes'),
            'require_enable': True,
        }
        if arg('waypoints'):
            over['waypoint_file'] = arg('waypoints')
        if arg('target_speed'):
            over['target_speed'] = float(arg('target_speed'))
        if arg('k_e'):
            over['k_e'] = float(arg('k_e'))

        # track_viz 도 같은 규칙을 따라야 한다. 안 그러면 인자를 안 줬을 때
        # waypoint_file 이 '' 이 되어 시각화만 빈 채로 뜬다.
        viz_over = {}
        if arg('waypoints'):
            viz_over['waypoint_file'] = arg('waypoints')
        if arg('track_dir'):
            viz_over['track_dir'] = arg('track_dir')

        mocap = Node(
            package='mlcs_mpc', executable='mocap_bridge', name='mocap_bridge',
            output='screen', parameters=[mocap_cfg],
            condition=IfCondition(LaunchConfiguration('mocap')))

        safety = Node(
            package='mlcs_mpc', executable='safety_node', name='safety_node',
            output='screen', parameters=[safety_cfg, track_cfg],
            condition=IfCondition(LaunchConfiguration('safety')))

        stanley = Node(
            package='mlcs_mpc', executable='stanley_node', name='stanley_node',
            output='screen',
            parameters=[stanley_cfg, track_cfg, over])

        viz = Node(
            package='mlcs_mpc', executable='track_viz', name='track_viz',
            output='screen',
            parameters=[safety_cfg, track_cfg, viz_over],
            condition=IfCondition(LaunchConfiguration('viz')))

        rviz = Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(pkg, 'launch', 'track.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')))

        return [mocap, safety, stanley, viz, rviz]

    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
