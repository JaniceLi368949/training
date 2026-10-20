import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rm_gimbal_demo'

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
    description='云台服务端/客户端示例：演示服务、参数与 launch 文件',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gimbal_server = rm_gimbal_demo.gimbal_server:main',
            'gimbal_client = rm_gimbal_demo.gimbal_client:main',
        ],
    },
)
