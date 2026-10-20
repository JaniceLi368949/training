"""launch 文件：同时启动 talker 和 listener。"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='my_first_pkg', executable='talker'),
        Node(package='my_first_pkg', executable='listener'),
    ])
