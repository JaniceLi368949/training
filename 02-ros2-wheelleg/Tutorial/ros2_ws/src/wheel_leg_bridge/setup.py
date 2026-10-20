import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wheel_leg_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'robots'), glob('robots/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RM Team',
    maintainer_email='rm@example.com',
    description='Wheel_Leg MuJoCo 仿真接入 ROS2 桥接节点',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wheel_leg_bridge = wheel_leg_bridge.wheel_leg_bridge:main',
        ],
    },
)
