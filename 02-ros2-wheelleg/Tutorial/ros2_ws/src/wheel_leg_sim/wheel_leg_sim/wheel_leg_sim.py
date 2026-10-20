"""轮腿机器人 MuJoCo 仿真节点 —— 纯仿真，不含任何控制律。

设计要点（作业契约，请勿修改本节点）
------------------------------------
本节点只做三件事：
  1. 维护 MuJoCo 模型，按固定 dt 步进物理仿真；
  2. 把传感器数据发成话题（/imu、/joint_states）；
  3. 把收到的执行器力矩指令写进 data.ctrl。

**它不含任何 PID / LQR / VMC 之类的控制律** —— 控制解算必须由外部节点完成后，
通过 /actuator_cmd 发进来。

步进方式：指令驱动（command-driven）
------------------------------------
本节点**不自己按墙钟时间步进**，而是每收到一条 /actuator_cmd 才：

    data.ctrl <- 指令  →  mj_step()  →  发布 /imu、/joint_states

这样保证「一条指令恰好对应一个 dt」。于是外部控制器里的 dt 恒等于模型 dt
(0.002 s)，与 LQR 的 K 矩阵（按 dt=0.002 离散整定）严格一致。

代价是仿真推进速度取决于控制器的发布速度（可能慢于实时），但物理与控制在
时间上始终自洽 —— 这正是不让控制律留在仿真节点里的原因之一。

话题接口
--------
发布  /imu           sensor_msgs/msg/Imu         机体四元数 + 角速度
发布  /joint_states  sensor_msgs/msg/JointState  6 个关节的位置/速度
订阅  /actuator_cmd  rm_msgs/msg/WheelLegCmd     6 个执行器力矩 (N·m)

查看器（GUI）
------------
默认打开 MuJoCo 被动查看器，可以直接看到机器人实时平衡：

    ros2 run wheel_leg_sim wheel_leg_sim                            # 带 GUI（默认）
    ros2 run wheel_leg_sim wheel_leg_sim --ros-args -p viewer:=false # 无界面

Linux 上未设置 DISPLAY / WAYLAND_DISPLAY 时会跳过查看器，自动无界面运行。
若显示变量已设置但显示服务不可用，请显式设 viewer:=false。镜头默认跟踪机体，
可用 -p cam_distance / cam_azimuth / cam_elevation 调整。

注意：渲染要花时间，所以仿真会以"慢动作"推进（比无界面时慢很多）。
这不影响控制正确性 —— 因为"一条指令 = 一个 dt"的时序契约与墙钟速度无关。

实时模式（新增，默认关闭，不影响原有行为）
----------------------------------------
默认 realtime:=false，行为与原文完全一致：能跑多快跑多快（无界面约 6 倍实时）。
打开后每步结束会 sleep 到「墙钟起点 + 仿真时间/rtf」，使仿真时间 = 墙钟时间：

    ros2 run wheel_leg_sim wheel_leg_sim --ros-args -p realtime:=true -p rtf:=1.0
    ros2 run wheel_leg_sim wheel_leg_sim --ros-args -p realtime:=true -p rtf:=0.5   # 慢动作

节流只在两步之间插 sleep，不改变 (状态 → 力矩 → 步进) 的顺序，所以控制动力学
逐位不变，只是墙钟上等它。每 2000 步会打印实测实时率，可直接核对是否 1.000x。

仿真时钟（新增，默认关闭）
------------------------
-p publish_clock:=true 时：
  · 发布 /clock（rosgraph_msgs/msg/Clock），内容 = 仿真时间 d.time；
  · /imu、/joint_states 的 header.stamp 也改用仿真时间。
订阅方（控制器）可加 --ros-args -p use_sim_time:=true 让双方时钟一致。
rtf=1.0 时仿真时间 ≈ 墙钟时间，装不装 /clock 都不影响控制结果。
"""
import os
import sys
import time

import mujoco
import mujoco.viewer
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rm_msgs.msg import WheelLegCmd
from sensor_msgs.msg import Imu, JointState

try:                       # 只有 publish_clock:=true 时才会用到
    from rosgraph_msgs.msg import Clock
except ImportError:        # 未安装 rosgraph_msgs 时不影响原有功能
    Clock = None

# 与 robot.xml 中 <sensor> 的名字一致，顺序即 /joint_states 的关节顺序
JOINT_NAMES = ['right1', 'right2', 'left1', 'left2', 'rightwheel', 'leftwheel']


class WheelLegSim(Node):
    def __init__(self):
        super().__init__('wheel_leg_sim')

        # ---- 加载模型 ----
        self.declare_parameter('robot_xml', '')
        xml = self.get_parameter('robot_xml').value
        if not xml:
            xml = os.path.join(
                get_package_share_directory('wheel_leg_sim'), 'robots', 'robot.xml')
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        # 初始化传感器而不推进时间；不能依赖查看器替我们调用 mj_forward。
        mujoco.mj_forward(self.model, self.data)
        self.dt = self.model.opt.timestep          # 0.002 s
        self.steps = 0

        # ---- 实时节流 / 仿真时钟（默认关闭 ⇒ 行为与原文完全一致）----
        self.declare_parameter('realtime', False)
        self.declare_parameter('rtf', 1.0)            # 实时率目标：1.0 同步，0.5 慢动作，2.0 加速
        self.declare_parameter('publish_clock', False)
        self.realtime = bool(self.get_parameter('realtime').value)
        self.rtf = float(self.get_parameter('rtf').value)
        self.publish_clock = bool(self.get_parameter('publish_clock').value)
        if self.publish_clock and Clock is None:
            self.get_logger().warn('未安装 rosgraph_msgs，publish_clock 自动关闭')
            self.publish_clock = False
        self._wall_origin = None
        self.clock_pub = (self.create_publisher(Clock, 'clock', 10)
                          if self.publish_clock else None)

        # ---- 话题 ----
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(WheelLegCmd, 'actuator_cmd', self.on_cmd, 10)

        # 心跳：久未收到指令就重发一次当前状态。
        # 存在的意义：本节点是"指令驱动"的，若不发状态，控制器就无从计算；
        # 而控制器恰好在本次初始状态发布之后才启动（错过那条消息）时，
        # 双方会互相空等。心跳保证任何时刻启动的控制器都能接上。
        self._last_cmd_mono = time.monotonic()
        self.create_timer(0.5, self.heartbeat)

        # ---- MuJoCo 查看器（可选，默认开）----
        # 打开后能在 GUI 里看到机器人实时平衡；无图形环境（SSH/服务器）时自动降级为无界面。
        self.declare_parameter('viewer', True)
        self.declare_parameter('viewer_fps', 60.0)
        self.declare_parameter('cam_distance', 1.2)     # 镜头到机器人的距离 (m)
        self.declare_parameter('cam_azimuth', 120.0)    # 方位角 (度)
        self.declare_parameter('cam_elevation', -12.0)  # 俯仰角 (度)
        self.viewer = None
        viewer_requested = self.get_parameter('viewer').value
        if (viewer_requested and sys.platform.startswith('linux')
                and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))):
            # launch_passive 在后台线程初始化 GLFW；失败时外层 except 无法接住，
            # 调用线程可能一直等待句柄。因此无显示环境必须在调用前跳过。
            self.get_logger().warn('未检测到显示环境，跳过 MuJoCo 查看器，使用无界面模式')
        elif viewer_requested:
            try:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                self.aim_camera()
                fps = float(self.get_parameter('viewer_fps').value)
                self.create_timer(1.0 / fps, self.render)
            except Exception as exc:                    # 仅处理同步抛出的启动异常
                self.get_logger().warn(f'MuJoCo 查看器打开失败，改为无界面运行：{exc}')
                self.viewer = None

        self.get_logger().info(f'模型已加载: {xml} (dt={self.dt}s)')
        if self.viewer is not None:
            self.get_logger().info('MuJoCo 查看器已打开（-p viewer:=false 可关闭）')
        self.get_logger().info('等待首条 /actuator_cmd ……（收到后即开始步进）')

        # 先发一次初始状态，让控制器有输入可以开始算
        self.publish_state()

    # ------------------------------------------------------------------
    def aim_camera(self):
        """把镜头对准机器人躯干。

        默认的自由视角不一定能框住机器人，这里改成"跟踪 torso"的镜头，
        这样即使机器人平移，画面里也一直能看到它。
        """
        try:
            torso_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, 'torso')
            with self.viewer.lock():
                if torso_id >= 0:
                    self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                    self.viewer.cam.trackbodyid = torso_id
                else:
                    self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                    self.viewer.cam.lookat[:] = [0.0, 0.0, 0.2]
                self.viewer.cam.distance = float(
                    self.get_parameter('cam_distance').value)
                self.viewer.cam.azimuth = float(
                    self.get_parameter('cam_azimuth').value)
                self.viewer.cam.elevation = float(
                    self.get_parameter('cam_elevation').value)
        except Exception as exc:
            self.get_logger().warn(f'设置查看器镜头失败（不影响仿真）：{exc}')

    def render(self):
        """把当前仿真状态同步到 MuJoCo 查看器窗口。

        与 on_cmd 在同一个 rclpy 线程里执行，所以不会和 mj_step 并发改 data。
        注意：渲染会占用时间，仿真推进速度会因此变慢（慢动作），
        但"一条指令 = 一个 dt"的时序契约不变，控制仍然正确。
        """
        if self.viewer is None:
            return
        if not self.viewer.is_running():
            self.get_logger().info('查看器窗口已关闭，退出仿真节点。')
            self.viewer = None
            raise SystemExit(0)
        self.viewer.sync()

    def heartbeat(self):
        """超过 0.5 s 没有收到指令，就补发一次当前状态（正常运行时不会触发）。"""
        if time.monotonic() - self._last_cmd_mono > 0.5:
            self.publish_state()

    # ------------------------------------------------------------------
    def on_cmd(self, msg):
        """收到一条执行器指令：写 ctrl → 步进一个 dt → 回发状态。"""
        self._last_cmd_mono = time.monotonic()
        d = self.data
        d.ctrl[0] = msg.right_phi1
        d.ctrl[1] = msg.right_phi4
        d.ctrl[2] = msg.left_phi1
        d.ctrl[3] = msg.left_phi4
        d.ctrl[4] = msg.right_wheel
        d.ctrl[5] = msg.left_wheel

        mujoco.mj_step(self.model, d)
        self.steps += 1

        self.publish_state()

        # ---- 实时节流：把墙钟拉齐到「仿真时间 / rtf」----
        # 仍然是一条指令一个 dt，只在这里等；不改变闭环里的任何时序。
        if self.realtime:
            now = time.monotonic()
            if self._wall_origin is None:              # 第一条指令到达时对齐起点
                self._wall_origin = now - d.time / self.rtf
            lag = (self._wall_origin + d.time / self.rtf) - now
            if lag > 0:
                time.sleep(lag)
            elif lag < -0.2:                           # 落后太多（渲染卡顿）→ 重新对齐
                self._wall_origin = now - d.time / self.rtf

        # 每 2000 步（=4 s 仿真时间）报一次进度，避免刷屏
        if self.steps % 2000 == 0:
            extra = ''
            if self.realtime and self._wall_origin is not None:
                wall = time.monotonic() - self._wall_origin
                extra = f'，墙钟 {wall:.2f} s，实测实时率 {d.time / wall:.3f}x'
            self.get_logger().info(
                f'仿真步数 {self.steps}，仿真时间 {d.time:.2f} s{extra}')

    # ------------------------------------------------------------------
    def sim_stamp(self):
        """把仿真时间 d.time 转成 ROS 时间戳（builtin_interfaces/Time）。"""
        total_ns = int(round(self.data.time * 1e9))
        return Time(sec=total_ns // 1000000000, nanosec=total_ns % 1000000000)

    def publish_state(self):
        # publish_clock:=true 时全部时间戳统一用仿真时间；否则沿用墙钟（原版行为）
        stamp = self.sim_stamp() if self.publish_clock else self.get_clock().now().to_msg()
        if self.clock_pub is not None:
            clk = Clock()
            clk.clock = stamp
            self.clock_pub.publish(clk)

        # /imu：姿态四元数 + 机体角速度
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu'
        # MuJoCo framequat 返回 [w, x, y, z]；ROS 四元数是 [x, y, z, w]
        w, x, y, z = self.data.sensor('orientation').data
        imu.orientation.x = float(x)
        imu.orientation.y = float(y)
        imu.orientation.z = float(z)
        imu.orientation.w = float(w)
        gx, gy, gz = self.data.sensor('angular_vel').data
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)
        self.imu_pub.publish(imu)

        # /joint_states：6 个关节的位置与速度
        js = JointState()
        js.header.stamp = stamp
        for name in JOINT_NAMES:
            js.name.append(name)
            js.position.append(float(self.data.sensor(name + '_pos').data[0]))
            js.velocity.append(float(self.data.sensor(name + '_vel').data[0]))
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = WheelLegSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if getattr(node, 'viewer', None) is not None:
            try:
                node.viewer.close()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
