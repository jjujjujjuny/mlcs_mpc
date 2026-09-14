#!/usr/bin/env python3
"""
car_mpc.launch.py — 실차 MPC 주행 (mocap 위치추정).

  ros2 launch mlcs_mpc car_mpc.launch.py waypoints:=<경로.csv>

■ 이 런치가 띄우는 것

    mocap_bridge  — NatNet pose → /mpc/state (+속도 추정)
    safety_node   — 경계 감시
    mpc_node      — MPC 컨트롤러 → /drive

■ 따로 띄워야 하는 것

    ① natnet_ros2        — Motive 에서 pose 를 받아오는 드라이버
    ② f1tenth_stack      — VESC 드라이버 + ackermann_mux (/drive → 모터)
       ros2 launch f1tenth_stack bringup_launch.py

    이 둘을 여기 안 넣은 이유: 둘 다 이 패키지 밖의 외부 패키지이고,
    한쪽이 안 떠 있을 때 "왜 안 도는지" 가 섞이면 디버깅이 어렵다.
    별도 터미널로 띄우고 각자의 로그를 보는 편이 낫다.

■ ★ 기본은 정지 상태다

    require_enable=true 이므로 차는 출발하지 않는다. 준비가 끝나면:
        ros2 topic pub --once /mpc/enabled std_msgs/msg/Bool "{data: true}"
    정지:
        ros2 topic pub --once /mpc/enabled std_msgs/msg/Bool "{data: false}"

    첫 주행은 bench:=true (거치대 위, 바퀴가 땅에 안 닿게) 로 조향만
    확인한 뒤 내리세요.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mlcs_mpc')
    vehicle_cfg = os.path.join(pkg, 'config', 'vehicle.yaml')
    mocap_cfg = os.path.join(pkg, 'config', 'mocap.yaml')
    safety_cfg = os.path.join(pkg, 'config', 'safety.yaml')

    args = [
        DeclareLaunchArgument('waypoints', default_value='',
                              description='추종할 웨이포인트 CSV (절대경로)'),
        DeclareLaunchArgument('target_speed', default_value='1.5',
                              description='목표 속도 (m/s). 캘리브레이션 전에는 낮게'),
        DeclareLaunchArgument('bench', default_value='false',
                              description='true 면 속도 0 — 거치대 위 조향 확인용'),
        DeclareLaunchArgument('safety', default_value='true',
                              description='경계 안전장치 사용'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]

    mocap = Node(
        package='mlcs_mpc', executable='mocap_bridge', name='mocap_bridge',
        output='screen', parameters=[mocap_cfg])

    safety = Node(
        package='mlcs_mpc', executable='safety_node', name='safety_node',
        output='screen', parameters=[safety_cfg],
        condition=IfCondition(LaunchConfiguration('safety')))

    mpc = Node(
        package='mlcs_mpc', executable='mpc_node', name='mpc_node',
        output='screen',
        parameters=[vehicle_cfg, {
            'waypoint_file': LaunchConfiguration('waypoints'),
            'target_speed': LaunchConfiguration('target_speed'),
            'bench': LaunchConfiguration('bench'),
            'require_enable': True,     # ★ 실차는 항상 출발 게이트를 건다
        }])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(pkg, 'launch', 'mpc.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription(args + [mocap, safety, mpc, rviz])
