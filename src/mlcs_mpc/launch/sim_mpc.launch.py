#!/usr/bin/env python3
"""
sim_mpc.launch.py — f1tenth_gym_ros 시뮬에서 MPC 주행.

  ros2 launch mlcs_mpc sim_mpc.launch.py waypoints:=<경로.csv>

시뮬레이터(gym_bridge)는 **따로** 띄워야 한다:
  터미널1: ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  터미널2: ros2 launch mlcs_mpc sim_mpc.launch.py

★ 시뮬에서는 safety_node 를 기본으로 끈다. 시뮬 트랙 좌표가 config/safety.yaml
  의 실험실 경계와 전혀 다르기 때문이다 — 켜두면 출발하자마자 경계 이탈로
  멈춘다. 실차에서는 반드시 켠다.
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

    wp_arg = DeclareLaunchArgument(
        'waypoints',
        default_value=os.path.join(pkg, 'waypoints', 'levine_centerline.csv'),
        description='추종할 웨이포인트 CSV 경로')
    speed_arg = DeclareLaunchArgument(
        'target_speed', default_value='2.0',
        description='목표 속도 (m/s)')
    noise_arg = DeclareLaunchArgument(
        'add_noise', default_value='false',
        description='실차 mocap 노이즈를 흉내내 게인 민감도를 미리 본다')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false', description='RViz 를 함께 띄운다')

    sim_bridge = Node(
        package='mlcs_mpc', executable='sim_bridge', name='sim_bridge',
        output='screen',
        parameters=[{
            'sim_odom_topic': '/ego_racecar/odom',
            'auto_enable': True,
            'add_noise': LaunchConfiguration('add_noise'),
        }])

    mpc = Node(
        package='mlcs_mpc', executable='mpc_node', name='mpc_node',
        output='screen',
        parameters=[vehicle_cfg, {
            'waypoint_file': LaunchConfiguration('waypoints'),
            'target_speed': LaunchConfiguration('target_speed'),
            # 시뮬은 위치가 완벽하므로 출발 게이트가 필요 없다
            'require_enable': False,
        }])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(pkg, 'launch', 'mpc.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        wp_arg, speed_arg, noise_arg, rviz_arg,
        sim_bridge, mpc, rviz,
    ])
