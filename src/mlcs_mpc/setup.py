import os
from glob import glob
from setuptools import setup

package_name = 'mlcs_mpc'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name] + [f'{package_name}.{s}' for s in
                                ('env', 'env.mpc', 'env.track', 'env.legacy', 'control', 'control.legacy', 'io', 'ui', 'utils')],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'waypoints'),
         [f for f in glob('waypoints/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MLCS',
    maintainer_email='hjchoe0512@gmail.com',
    description='F1TENTH MPC stack with OptiTrack mocap localization',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mpc_node        = mlcs_mpc.control.legacy.ilqr_node:main',
            'stanley_node    = mlcs_mpc.control.legacy.stanley_node:main',
            'mpcc_node       = mlcs_mpc.control.mpcc_node:main',
            'sim_node        = mlcs_mpc.io.sim_node:main',
            'import_track    = mlcs_mpc.utils.import_track:main',
            'track_viz       = mlcs_mpc.ui.track_viz:main',
            'track_info      = mlcs_mpc.utils.track_info:main',
            'mocap_bridge    = mlcs_mpc.io.mocap_bridge:main',
            'lidar_bridge    = mlcs_mpc.io.lidar_bridge:main',
            'relocalize      = mlcs_mpc.io.relocalize:main',
            'mocap_calibrate = mlcs_mpc.utils.mocap_calibrate:main',
            'sim_bridge      = mlcs_mpc.io.sim_bridge:main',
            'joystick_teleop = mlcs_mpc.ui.joystick_teleop:main',
            'safety_node     = mlcs_mpc.utils.safety_node:main',
            'calibrate       = mlcs_mpc.utils.calibrate:main',
            'waypoint_logger = mlcs_mpc.utils.waypoint_logger:main',
            'data_logger     = mlcs_mpc.utils.data_logger:main',
            'smooth_path     = mlcs_mpc.utils.smooth_path:main',
        ],
    },
)
