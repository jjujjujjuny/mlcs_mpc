import os
from glob import glob
from setuptools import setup

package_name = 'mlcs_mpc'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
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
            'mpc_node        = mlcs_mpc.mpc_node:main',
            'stanley_node    = mlcs_mpc.stanley_node:main',
            'import_track    = mlcs_mpc.import_track:main',
            'mocap_bridge    = mlcs_mpc.mocap_bridge:main',
            'mocap_calibrate = mlcs_mpc.mocap_calibrate:main',
            'sim_bridge      = mlcs_mpc.sim_bridge:main',
            'joystick_teleop = mlcs_mpc.joystick_teleop:main',
            'safety_node     = mlcs_mpc.safety_node:main',
            'calibrate       = mlcs_mpc.calibrate:main',
            'waypoint_logger = mlcs_mpc.waypoint_logger:main',
            'data_logger     = mlcs_mpc.data_logger:main',
            'smooth_path     = mlcs_mpc.smooth_path:main',
        ],
    },
)
