"""TF 广播：静态 base_link→gimbal_link + 动态 gimbal_link→camera_link（pitch 摆动）。"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.static_br = StaticTransformBroadcaster(self)
        self.dynamic_br = TransformBroadcaster(self)

        # 静态变换：底盘 -> 云台（云台在底盘正上方 0.2 m）
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'gimbal_link'
        static_tf.transform.translation.z = 0.2
        static_tf.transform.rotation.w = 1.0
        self.static_br.sendTransform(static_tf)
        self.get_logger().info('已发布静态变换 base_link -> gimbal_link')

        # 动态变换：云台 -> 相机（每 0.1 s 更新，pitch 角正弦摆动）
        self.timer = self.create_timer(0.1, self.broadcast_camera)

    def broadcast_camera(self):
        now = self.get_clock().now()
        pitch = 0.6 * math.sin(now.nanoseconds * 1e-9)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'gimbal_link'
        tf.child_frame_id = 'camera_link'
        tf.transform.translation.x = 0.1  # 相机在云台前方 0.1 m
        # 绕 Y 轴旋转 pitch（四元数）
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = math.sin(pitch / 2.0)
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = math.cos(pitch / 2.0)
        self.dynamic_br.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
