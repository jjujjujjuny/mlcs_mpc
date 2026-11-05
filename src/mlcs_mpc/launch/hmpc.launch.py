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
        return [
            Node(package='mlcs_mpc', executable='mocap_bridge', name='mocap_bridge',
                 output='screen', parameters=[os.path.join(pkg, 'config', 'mocap.yaml')]),
            Node(package='mlcs_mpc', executable='hmpc_node', name='hmpc_node',
                 output='screen', parameters=[os.path.join(pkg, 'config', 'hmpc.yaml'),
                                              {'bench': bench, 'require_enable': True}]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('bench', default_value='true',
                              description='true 면 속도 0. 거치대 확인용'),
        OpaqueFunction(function=_nodes),
    ])
