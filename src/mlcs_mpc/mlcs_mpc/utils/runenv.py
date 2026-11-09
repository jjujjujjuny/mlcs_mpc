"""
runenv — python3 main.py / main_sim.py 만으로 돌게 실행 환경을 잡는다 (없으면 잡고 자기 자신을 다시 실행).

  ensure_acados()  ~/acados 의 라이브러리 경로 (LD_LIBRARY_PATH 는 프로세스 시작 때만 읽힌다)
  ensure_ros2()    ROS 2 (humble/foxy). 이 PC 는 .bashrc 가 ROS 1 (noetic) 을 불러와 섞이므로 ROS 1 을 걷어낸다.
                   ~/mlcs_sim_ws (./sim.sh build) 가 있으면 그것도 얹는다.
"""
import os
import shlex
import sys


def ensure_acados():
    src = os.environ.get('ACADOS_SOURCE_DIR') or os.path.expanduser('~/acados')
    lib = os.path.join(src, 'lib')
    if not os.path.isdir(lib):
        return      # acados 가 다른 곳에 있으면 ACADOS_SOURCE_DIR 를 주고 돌린다
    if lib in os.environ.get('LD_LIBRARY_PATH', '').split(':') and os.environ.get('ACADOS_SOURCE_DIR'):
        return
    env = dict(os.environ, ACADOS_SOURCE_DIR=src,
               LD_LIBRARY_PATH=':'.join(p for p in (os.environ.get('LD_LIBRARY_PATH', ''), lib) if p))
    os.execve(sys.executable, [sys.executable] + sys.argv, env)


def ensure_ros2():
    try:
        import rclpy  # noqa: F401
        return
    except ImportError:
        pass
    if os.environ.get('MLCS_ROS2_REEXEC'):
        sys.exit('✗ ROS 2 의 rclpy 를 못 찾았습니다 — /opt/ros/<humble|foxy> 를 확인하세요')
    distro = next((d for d in ('humble', 'foxy') if os.path.isfile(f'/opt/ros/{d}/setup.bash')), None)
    if distro is None:
        sys.exit('✗ ROS 2 가 없습니다 (/opt/ros/humble 또는 foxy). RViz 없이는 main.py 를 쓰세요')
    ws = os.path.expanduser('~/mlcs_sim_ws/install/setup.bash')
    cmd = ('for v in ROS_DISTRO ROS_VERSION ROS_PACKAGE_PATH ROS_ROOT ROS_MASTER_URI ROS_ETC_DIR '
           'ROSLISP_PACKAGE_DIRECTORIES; do unset $v; done; '
           'for v in PATH LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH PKG_CONFIG_PATH; do '
           'export $v="$(printf %s "${!v}" | tr : "\\n" | grep -v ^/opt/ros/noetic | paste -sd: -)"; done; '
           f'source /opt/ros/{distro}/setup.bash; '
           + (f'source {shlex.quote(ws)}; ' if os.path.isfile(ws) else '')
           + 'export MLCS_ROS2_REEXEC=1; exec ' + ' '.join(shlex.quote(a) for a in [sys.executable] + sys.argv))
    os.execvp('bash', ['bash', '-c', cmd])
