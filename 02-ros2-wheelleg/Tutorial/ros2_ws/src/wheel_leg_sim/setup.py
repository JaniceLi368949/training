import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wheel_leg_sim'

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
    description='轮腿机器人 MuJoCo 仿真节点（纯仿真，不含任何控制律）',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wheel_leg_sim = wheel_leg_sim.wheel_leg_sim:main',
        ],
    },
)
