"""launch 文件：一次拉起云台服务端 + 客户端，并覆盖参数。"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_gimbal_demo',
            executable='gimbal_server',
            name='gimbal_server',
            parameters=[{'max_yaw': 1.2, 'max_pitch': 0.6}],
        ),
        Node(
            package='rm_gimbal_demo',
            executable='gimbal_client',
            name='gimbal_client',
            arguments=['0.8', '0.4'],
        ),
    ])
