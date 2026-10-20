"""云台客户端：调用 set_gimbal_angle 服务（参数：yaw pitch，单位 rad）。"""
import sys
import rclpy
from rclpy.node import Node
from rm_msgs.srv import SetGimbalAngle


class GimbalClient(Node):
    def __init__(self):
        super().__init__('gimbal_client')
        self.client = self.create_client(SetGimbalAngle, 'set_gimbal_angle')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务端上线...')

    def send(self, yaw, pitch):
        req = SetGimbalAngle.Request()
        req.yaw = yaw
        req.pitch = pitch
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalClient()
    yaw = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    pitch = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
    response = node.send(yaw, pitch)
    if response is not None:
        node.get_logger().info(
            f'服务返回: success={response.success}, {response.message}')
    else:
        node.get_logger().error('服务调用失败')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
