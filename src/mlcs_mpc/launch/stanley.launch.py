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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mlcs_mpc')
    stanley_cfg = os.path.join(pkg, 'config', 'stanley.yaml')
    mocap_cfg = os.path.join(pkg, 'config', 'mocap.yaml')
    safety_cfg = os.path.join(pkg, 'config', 'safety.yaml')

    args = [
        DeclareLaunchArgument('waypoints', default_value='',
                              description='추종할 웨이포인트 CSV (절대경로)'),
        DeclareLaunchArgument('target_speed', default_value='0.7',
                              description='목표 속도 (m/s). 첫 주행은 낮게'),
        DeclareLaunchArgument('k_e', default_value='1.5',
                              description='Stanley 횡오차 게인'),
        DeclareLaunchArgument('bench', default_value='false',
                              description='true 면 속도 0 — 거치대 확인용'),
        DeclareLaunchArgument('safety', default_value='true'),
        DeclareLaunchArgument('mocap', default_value='true',
                              description='mocap_bridge 를 여기서 띄운다'),
        DeclareLaunchArgument('debug', default_value='false'),
    ]

    mocap = Node(
        package='mlcs_mpc', executable='mocap_bridge', name='mocap_bridge',
        output='screen', parameters=[mocap_cfg],
        condition=IfCondition(LaunchConfiguration('mocap')))

    safety = Node(
        package='mlcs_mpc', executable='safety_node', name='safety_node',
        output='screen', parameters=[safety_cfg],
        condition=IfCondition(LaunchConfiguration('safety')))

    stanley = Node(
        package='mlcs_mpc', executable='stanley_node', name='stanley_node',
        output='screen',
        parameters=[stanley_cfg, {
            'waypoint_file': LaunchConfiguration('waypoints'),
            'target_speed': LaunchConfiguration('target_speed'),
            'k_e': LaunchConfiguration('k_e'),
            'bench': LaunchConfiguration('bench'),
            'debug': LaunchConfiguration('debug'),
            'require_enable': True,
        }])

    return LaunchDescription(args + [mocap, safety, stanley])
