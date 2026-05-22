from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'swarm_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource index marker
        ('share/ament_index/resource_index/packages',
            ['resource/swarm-controller']),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # config / parameter files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eason',
    maintainer_email='eeeaaason070311@gmail.com',
    description='SwarmBot upper-computer controller: mocap republisher + MPC swarm controller',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mocap_republisher    = swarm_controller.mocap_republisher:main',
            'swarm_mpc_controller = swarm_controller.swarm_mpc_controller:main',
        ],
    },
)
