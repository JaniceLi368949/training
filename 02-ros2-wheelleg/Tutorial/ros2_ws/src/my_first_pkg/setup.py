import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_first_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RM Team',
    maintainer_email='rm@example.com',
    description='第一个 ROS2 Python 包：发布者(talker) 与订阅者(listener)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'talker = my_first_pkg.talker:main',
            'listener = my_first_pkg.listener:main',
        ],
    },
)
