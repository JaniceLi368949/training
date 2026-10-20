from setuptools import find_packages, setup

package_name = 'rm_tf_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RM Team',
    maintainer_email='rm@example.com',
    description='TF2 坐标变换示例：底盘/云台/相机坐标系链',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tf_broadcaster = rm_tf_demo.tf_broadcaster:main',
            'tf_listener = rm_tf_demo.tf_listener:main',
        ],
    },
)
