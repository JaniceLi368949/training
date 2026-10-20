"""发布者节点：每 0.5 秒向 /chatter 话题发布一条 String 消息。"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        # 创建一个发布者：消息类型 String，话题名 'chatter'，队列长度 10
        self.publisher = self.create_publisher(String, 'chatter', 10)
        # 创建一个定时器，每 0.5 秒触发一次 timer_callback
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS2: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'发布: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    # spin 让节点保持运行，直到收到退出信号（Ctrl+C）
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
