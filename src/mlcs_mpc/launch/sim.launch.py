#!/usr/bin/env python3
"""
sim.launch.py — sim_node (ODE 차, env/simulator.py) + mpcc_node (컨트롤러, mpcc.launch.py source:=sim) + RViz.

  ros2 launch mlcs_mpc sim.launch.py track_csv:=<prep csv> repo_root:=<저장소>
  보통은 저장소 최상위의 ./sim.sh 로 띄운다 (race.yaml 을 읽어 같은 설정으로).

  extra_params 는 mpcc_node 와 차 모델에 똑같이 들어간다 (race.yaml 의 mpcc: 덮어쓰기).
  sim_params 는 sim_node 에만 (on_wall, car_wheel_speed_tau 같은 차만 다른 값, noise_pos ...).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mlcs_mpc')

    def _nodes(context, *_a, **_k):
        arg = lambda n: LaunchConfiguration(n).perform(context).strip()  # noqa: E731
        track, extra = arg('track_csv'), arg('extra_params')
        sim_p = {'track_csv': track,
                 'mpcc_params': [os.path.join(pkg, 'config', 'mpcc.yaml')] + ([extra] if extra else []),
                 'autostart_delay': float(arg('autostart_delay')),
                 'on_wall': arg('on_wall'),
                 'log_csv': arg('log_csv')}
        nodes = [Node(package='mlcs_mpc', executable='sim_node', name='mlcs_sim', output='screen',
                      parameters=[sim_p] + ([arg('sim_params')] if arg('sim_params') else []))]
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'mpcc.launch.py')),
            launch_arguments={'source': 'sim', 'bench': arg('bench'), 'track_csv': track,
                              'extra_params': extra, 'v_max': arg('v_max'),
                              'repo_root': arg('repo_root'), 'worker_python': arg('worker_python')}.items()))
        if arg('rviz').lower() in ('1', 'true', 'yes'):
            nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                              arguments=['-d', os.path.join(pkg, 'launch', 'sim.rviz')]))
        return nodes

    return LaunchDescription([
        DeclareLaunchArgument('track_csv', description='prep 형식 트랙 (절대경로)'),
        DeclareLaunchArgument('extra_params', default_value='', description='mpcc.yaml 뒤에 덮어쓸 파일'),
        DeclareLaunchArgument('sim_params', default_value='', description='sim_node 에만 줄 파라미터 파일'),
        DeclareLaunchArgument('bench', default_value='false', description='true 면 mpcc 속도 0'),
        DeclareLaunchArgument('v_max', default_value='', description='속도 상한. 비우면 mpcc.yaml'),
        DeclareLaunchArgument('repo_root', default_value='', description='저장소 경로 (워커·트랙 상대경로 기준)'),
        DeclareLaunchArgument('worker_python', default_value='', description='OCP 워커 파이썬'),
        DeclareLaunchArgument('autostart_delay', default_value='3.0',
                              description='이만큼 뒤 /mpc/enabled=true. 음수면 ./sim.sh go 를 기다린다'),
        DeclareLaunchArgument('on_wall', default_value='warn', description='warn | stop | reset'),
        DeclareLaunchArgument('log_csv', default_value='', description='차 상태 기록 csv. 비우면 안 남긴다'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_nodes),
    ])
