"""Wheel_Leg MuJoCo 仿真接入 ROS2 的桥接节点。

功能：
- 加载 MuJoCo 模型（robot.xml + obstacles.xml）
- 定时步进仿真
- 发布 /imu (sensor_msgs/Imu) 与 /joint_states (sensor_msgs/JointState)
- 订阅 /cmd_vel (geometry_msgs/Twist) 驱动左右轮
"""
import os
import mujoco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory


class WheelLegBridge(Node):
    def __init__(self):
        super().__init__('wheel_leg_bridge')

        # 模型路径参数（默认取包内 robots/robot.xml）
        self.declare_parameter('robot_xml', '')
        xml = self.get_parameter('robot_xml').value
        if not xml:
            xml = os.path.join(
                get_package_share_directory('wheel_leg_bridge'),
                'robots', 'robot.xml')

        # 加载 MuJoCo 模型与数据
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep  # 0.002 s
        self.get_logger().info(f'已加载模型: {xml} (dt={self.dt}s)')

        # 发布者：IMU、关节状态
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 订阅者：速度指令
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.left_torque = 0.0
        self.right_torque = 0.0
        self.track = 0.183  # 左右轮间距 (m)

        # 控制循环：50 Hz，每次步进 10 步（10 * 0.002 = 0.02 s）
        self.timer = self.create_timer(0.02, self.step)

    def cmd_vel_cb(self, msg):
        """把 /cmd_vel 简单映射为左右轮力矩（差速）。"""
        k = 1.0
        self.left_torque = k * (msg.linear.x - msg.angular.z * self.track / 2.0)
        # 模型左右轮转轴相反：右轮正向滚动对应负的执行器力矩。
        self.right_torque = -k * (msg.linear.x + msg.angular.z * self.track / 2.0)

    def step(self):
        # 执行器顺序：0-3 为腿关节，4=右轮，5=左轮（对应 XML 中 motor 定义顺序）
        self.data.ctrl[0:4] = 0.0
        self.data.ctrl[4] = self.right_torque
        self.data.ctrl[5] = self.left_torque

        for _ in range(10):
            mujoco.mj_step(self.model, self.data)

        self.publish_imu()
        self.publish_joints()

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu'

        # MuJoCo framequat 返回 [w, x, y, z]；ROS 四元数为 [x, y, z, w]
        w, x, y, z = self.data.sensor('orientation').data
        msg.orientation.x = float(x)
        msg.orientation.y = float(y)
        msg.orientation.z = float(z)
        msg.orientation.w = float(w)

        gx, gy, gz = self.data.sensor('angular_vel').data
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        self.imu_pub.publish(msg)

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        names = ['right1', 'right2', 'left1', 'left2', 'rightwheel', 'leftwheel']
        for n in names:
            msg.name.append(n)
            msg.position.append(float(self.data.sensor(n + '_pos').data[0]))
            msg.velocity.append(float(self.data.sensor(n + '_vel').data[0]))
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelLegBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
