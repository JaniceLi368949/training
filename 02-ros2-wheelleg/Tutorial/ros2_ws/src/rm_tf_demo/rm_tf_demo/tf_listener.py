"""TF 监听：周期查询 camera_link 相对 base_link 的变换。"""
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.lookup)

    def lookup(self):
        try:
            # rclpy.time.Time() 表示"最新可用的变换"
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'camera_link', rclpy.time.Time())
            t = tf.transform.translation
            self.get_logger().info(
                f'camera 相对 base: x={t.x:.2f} y={t.y:.2f} z={t.z:.2f}')
        except Exception as e:
            self.get_logger().warn(f'查询失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
