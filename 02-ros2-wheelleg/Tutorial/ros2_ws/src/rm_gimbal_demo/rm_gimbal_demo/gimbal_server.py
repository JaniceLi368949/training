"""云台服务端：提供 set_gimbal_angle 服务，用参数限制角度范围。"""
import rclpy
from rclpy.node import Node
from rm_msgs.srv import SetGimbalAngle


class GimbalServer(Node):
    def __init__(self):
        super().__init__('gimbal_server')
        # 声明参数（可被命令行或 launch 文件覆盖）
        self.declare_parameter('max_yaw', 3.14)
        self.declare_parameter('max_pitch', 1.57)
        self.srv = self.create_service(
            SetGimbalAngle, 'set_gimbal_angle', self.set_gimbal_angle_callback)

    def set_gimbal_angle_callback(self, request, response):
        max_yaw = self.get_parameter('max_yaw').value
        max_pitch = self.get_parameter('max_pitch').value
        # 限幅，模拟真实云台的角度约束
        yaw = max(-max_yaw, min(max_yaw, request.yaw))
        pitch = max(-max_pitch, min(max_pitch, request.pitch))
        response.success = True
        response.message = f'设置成功: yaw={yaw:.2f}, pitch={pitch:.2f}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GimbalServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
