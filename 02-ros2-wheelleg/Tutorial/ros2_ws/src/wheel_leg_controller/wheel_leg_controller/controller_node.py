import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from rm_msgs.msg import WheelLegCmd
import math
import numpy as np

from .vmc_cal import five_links_param
from .lqr_update import lqr_controller

class PIDController:
    def __init__(self, kp, ki, kd, max_out, max_i, feedforward=0.0):
        self.kp = kp; self.ki = ki; self.kd = kd; self.max_out = max_out; self.max_i = max_i; self.feedforward = feedforward
        self.integral = 0.0; self.prev_error = 0.0
    def compute(self, current, target):
        error = target - current; self.integral += error
        self.integral = np.clip(self.integral, -self.max_i, self.max_i)
        derivative = (error - self.prev_error); self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative + self.feedforward
        return np.clip(output, -self.max_out, self.max_out)


class Stage4Node(Node):
    def __init__(self):
        super().__init__('stage4_node')
        
        # 1. 基础设置 (如果还抖，可以改成 0.005)
        self.dt = 0.01 
        self.wheel_radius = 0.06
        
        # 2. 实例化 VMC, LQR 和 PID
        self.vmc_right = five_links_param(self.dt)
        self.vmc_left = five_links_param(self.dt)
        self.lqr_right = lqr_controller(self.dt)
        self.lqr_left = lqr_controller(self.dt)
        
        # 腿部力（调柔和了一点）
        self.left_L0_pid = PIDController(1000.0, 0.0, 20.0, 300.0, 0.0)
        self.right_L0_pid = PIDController(1000.0, 0.0, 20.0, 300.0, 0.0)
        
        # 3. 目标设定
        self.target_w = 0.0      
        self.target_v_cmd = 0.0  # 外部发来的突变速度
        self.v_set = 0.0         # 实际用于控制的平滑速度
        
        # 4. 数据缓存
        self.joint_pos = {'right1': 0.0, 'right2': 0.0, 'left1': 0.0, 'left2': 0.0}
        self.wheel_pos = {'rightwheel': 0.0, 'leftwheel': 0.0}
        self.wheel_vel = {'rightwheel': 0.0, 'leftwheel': 0.0}
        self.pitch = 0.0
        self.gyro_pitch = 0.0
        self.gyro_yaw = 0.0
        self.roll = 0.0
        self.roll_rate = 0.0  
        
        # 5. 通信
        self.sub_joint = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.pub_cmd = self.create_publisher(WheelLegCmd, '/actuator_cmd', 10)
        
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("阶段4节点启动！准备站立...")

    def imu_cb(self, msg):
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        raw_pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        self.pitch = -raw_pitch 
        
        raw_roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        self.roll = -raw_roll
        self.roll_rate = -msg.angular_velocity.x
        
        self.gyro_pitch = -msg.angular_velocity.y
        self.gyro_yaw = msg.angular_velocity.z

    def joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joint_pos:
                self.joint_pos[name] = msg.position[i]
            elif name in self.wheel_pos:
                self.wheel_pos[name] = msg.position[i]
                self.wheel_vel[name] = msg.velocity[i]

    def cmd_cb(self, msg):
        self.target_v_cmd = msg.linear.x  # 只记录，不直接赋值

    def control_loop(self):
        # --- 提取共模状态 ---
        motor_speed_left = self.wheel_vel['leftwheel']
        motor_speed_right = self.wheel_vel['rightwheel']
        motor_pos_left = self.wheel_pos['leftwheel']
        motor_pos_right = self.wheel_pos['rightwheel']
        
        common_motor_speed = (motor_speed_left - motor_speed_right) / 2.0
        common_motor_pos = (motor_pos_left - motor_pos_right) / 2.0

        # =========================================================================
        # 👇👇👇 新增：速度平滑过渡（防止一给指令就抽搐） 👇👇👇
        # =========================================================================
        accel_rate = 1.0  # 每秒最多加速 1.0 m/s
        if self.v_set < self.target_v_cmd:
            self.v_set = min(self.v_set + accel_rate * self.dt, self.target_v_cmd)
        elif self.v_set > self.target_v_cmd:
            self.v_set = max(self.v_set - accel_rate * self.dt, self.target_v_cmd)
            
        self.lqr_left.v_set = self.v_set
        self.lqr_right.v_set = self.v_set
        
        # 如果速度不为0，则更新目标位移，防止倒车
        if abs(self.v_set) > 0.01:
            self.lqr_left.x_b_set = (common_motor_pos * self.wheel_radius)
            self.lqr_right.x_b_set = (common_motor_pos * self.wheel_radius)
        else:
            self.lqr_left.x_b_set = (common_motor_pos * self.wheel_radius)
            self.lqr_right.x_b_set = (common_motor_pos * self.wheel_radius)
        # =========================================================================

        # ==================== 横滚平衡补偿 ====================
        roll_kp = 0.15  # 调小了，防止动态抽搐
        roll_kd = 0.01  
        roll_correction = (self.roll - 0.0) * roll_kp + (self.roll_rate - 0.0) * roll_kd
        
        roll_correction = max(min(roll_correction, 0.04), -0.04) 
        
        base_len = 0.12  # 基础腿长
        sign = 1 
        
        target_len_l = base_len + sign * roll_correction
        target_len_r = base_len - sign * roll_correction

        # ==================== 左腿控制 ====================
        self.vmc_left.forward_kinematics_cal(1, self.joint_pos['left1'], self.joint_pos['left2'], self.pitch, self.gyro_pitch)
        self.lqr_left.states_update(1, self.vmc_left, self.pitch, self.gyro_pitch, common_motor_pos, common_motor_speed, self.wheel_radius)
        self.lqr_left.calc_k_matrix_from_poly(self.vmc_left.L_0)

        F_0_ctrl_l = self.left_L0_pid.compute(self.vmc_left.L_0, target_len_l)
        F_0_l = (15 / math.cos(self.vmc_left.theta) if math.cos(self.vmc_left.theta) != 0 else 0) + F_0_ctrl_l
        F_0_l = np.clip(F_0_l, -200.0, 200.0)

        T_p_l = (self.lqr_left.K[1] * (self.lqr_left.theta - 0.0)
               + self.lqr_left.K[3] * (self.lqr_left.d_theta - 0.0)
               + self.lqr_left.K[5] * (self.lqr_left.x_b - self.lqr_left.x_b_set)
               + self.lqr_left.K[7] * (self.lqr_left.v_b_whole - self.lqr_left.v_set)
               + self.lqr_left.K[9] * (self.lqr_left.phi - 0.0)
               + self.lqr_left.K[11] * (self.lqr_left.d_phi - 0.0))

        wheel_T_l = (self.lqr_left.K[0] * (self.lqr_left.theta - 0.0)
                   + self.lqr_left.K[2] * (self.lqr_left.d_theta - 0.0)
                   + self.lqr_left.K[4] * (self.lqr_left.x_b - self.lqr_left.x_b_set)
                   + self.lqr_left.K[6] * (self.lqr_left.v_b_whole - self.lqr_left.v_set)
                   + self.lqr_left.K[8] * (self.lqr_left.phi - 0.0)
                   + self.lqr_left.K[10] * (self.lqr_left.d_phi - 0.0))
        left_wheel_T = -wheel_T_l
        # 放开限幅，给它走的力气
        left_wheel_T = np.clip(left_wheel_T, -1.5, 1.5)

        torque_set_0_l, torque_set_1_l = self.vmc_left.VMC_torque_cal(F_0_l, T_p_l)

        # ==================== 右腿控制 ====================
        self.vmc_right.forward_kinematics_cal(0, self.joint_pos['right1'], self.joint_pos['right2'], self.pitch, self.gyro_pitch)
        self.lqr_right.states_update(0, self.vmc_right, self.pitch, self.gyro_pitch, -common_motor_pos, -common_motor_speed, self.wheel_radius)
        self.lqr_right.calc_k_matrix_from_poly(self.vmc_right.L_0)

        F_0_ctrl_r = self.right_L0_pid.compute(self.vmc_right.L_0, target_len_r)
        F_0_r = (15 / math.cos(self.vmc_right.theta) if math.cos(self.vmc_right.theta) != 0 else 0) + F_0_ctrl_r
        F_0_r = np.clip(F_0_r, -200.0, 200.0)

        T_p_r = (self.lqr_right.K[1] * (self.lqr_right.theta - 0.0)
               + self.lqr_right.K[3] * (self.lqr_right.d_theta - 0.0)
               + self.lqr_right.K[5] * (self.lqr_right.x_b - self.lqr_right.x_b_set)
               + self.lqr_right.K[7] * (self.lqr_right.v_b_whole - self.lqr_right.v_set)
               + self.lqr_right.K[9] * (self.lqr_right.phi - 0.0)
               + self.lqr_right.K[11] * (self.lqr_right.d_phi - 0.0))
               
        wheel_T_r = (self.lqr_right.K[0] * (self.lqr_right.theta - 0.0)
                   + self.lqr_right.K[2] * (self.lqr_right.d_theta - 0.0)
                   + self.lqr_right.K[4] * (self.lqr_right.x_b - self.lqr_right.x_b_set)
                   + self.lqr_right.K[6] * (self.lqr_right.v_b_whole - self.lqr_right.v_set)
                   + self.lqr_right.K[8] * (self.lqr_right.phi - 0.0)
                   + self.lqr_right.K[10] * (self.lqr_right.d_phi - 0.0))
        right_wheel_T = wheel_T_r
        # 放开限幅
        right_wheel_T = np.clip(right_wheel_T, -1.5, 1.5)

        torque_set_0_r, torque_set_1_r = self.vmc_right.VMC_torque_cal(F_0_r, T_p_r)

        # ==================== 打包发送 ====================
        cmd = WheelLegCmd()
        cmd.left_phi1 = -torque_set_0_l
        cmd.left_phi4 = -torque_set_1_l
        cmd.right_phi1 = torque_set_0_r
        cmd.right_phi4 = torque_set_1_r
        cmd.left_wheel = left_wheel_T
        cmd.right_wheel = right_wheel_T
        
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Stage4Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()