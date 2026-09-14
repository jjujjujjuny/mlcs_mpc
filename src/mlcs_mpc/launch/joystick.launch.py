#!/usr/bin/env python3
"""
joystick.launch.py — 조이스틱 수동 조종.

  ros2 launch mlcs_mpc joystick.launch.py                 # 수동 전용
  ros2 launch mlcs_mpc joystick.launch.py max_speed:=2.0

■ 이 런치가 띄우는 것

    joy_node          — /dev/input/js0 → /joy
    joystick_teleop   — /joy → /teleop

■ 따로 띄워야 하는 것

    ros2 launch f1tenth_stack bringup_launch.py     ← VESC + ackermann_mux

  ★ bringup 을 이미 띄웠다면 `joy:=false` 로 실행하세요.
    f1tenth_stack 의 bringup_launch.py 가 joy_node 를 **이미 띄웁니다.**
    여기서 또 띄우면 /joy 퍼블리셔가 둘이 되어 deadzone 설정이 뒤섞이고,
    어느 쪽 값이 오는지에 따라 조종감이 달라집니다.

      ros2 launch mlcs_mpc joystick.launch.py joy:=false

■ 자율 주행과 같이 쓰기

  mpc_node 와 함께 띄우면 LB 로 수동↔자율을 오갈 수 있다:

      ros2 launch mlcs_mpc car_mpc.launch.py waypoints:=<경로.csv>   # 터미널 A
      ros2 launch mlcs_mpc joystick.launch.py joy:=false             # 터미널 B

  LB 를 누르면 joystick_teleop 이 /mpc/enabled 를 토글한다.
  수동일 때는 /teleop 을 발행해 mux 우선순위(joystick 100 > navigation 10)로
  MPC 명령을 덮어쓴다.
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
    joy_cfg = os.path.join(pkg, 'config', 'joystick.yaml')

    args = [
        DeclareLaunchArgument(
            'joy', default_value='true',
            description='joy_node 를 띄운다. bringup 을 이미 띄웠으면 false'),
        DeclareLaunchArgument(
            'device_id', default_value='0',
            description='/dev/input/jsN 의 N'),
        DeclareLaunchArgument(
            'max_speed', default_value='1.5',
            description='수동 조종 최고 속도 (m/s)'),
        DeclareLaunchArgument(
            'allow_toggle', default_value='true',
            description='LB 로 수동↔자율 토글. 수동 전용이면 false'),
        DeclareLaunchArgument(
            'debug', default_value='false',
            description='트리거/조향 값을 0.5초마다 출력'),
    ]

    joy_node = Node(
        package='joy', executable='joy_node', name='joy',
        output='screen',
        condition=IfCondition(LaunchConfiguration('joy')),
        parameters=[{
            'device_id': LaunchConfiguration('device_id'),
            # ★ deadzone=0 — 트리거 응답을 왜곡 없이 그대로 넘긴다.
            #   joy 의 deadzone 은 축의 '중앙' 기준으로 잡히는데, 트리거는
            #   중앙이 절반쯤 당긴 지점이라 그대로 두면 스로틀 중간에
            #   평평한 구간이 생긴다. 데드존은 joystick_teleop 이
            #   정규화된 0~1 스로틀 기준으로 처리한다.
            'deadzone': 0.0,
            # ★ autorepeat_rate — RT 를 고정한 채 있어도 /joy 가 계속 나가게
            #   한다. 안 그러면 값이 안 변할 때 발행이 멈춰 아래 워치독이
            #   오발동하고, ackermann_mux 의 타임아웃에도 걸린다.
            'autorepeat_rate': 20.0,
        }])

    teleop = Node(
        package='mlcs_mpc', executable='joystick_teleop',
        name='joystick_teleop', output='screen',
        parameters=[joy_cfg, {
            'max_speed': LaunchConfiguration('max_speed'),
            'allow_toggle': LaunchConfiguration('allow_toggle'),
            'debug': LaunchConfiguration('debug'),
        }])

    return LaunchDescription(args + [joy_node, teleop])
