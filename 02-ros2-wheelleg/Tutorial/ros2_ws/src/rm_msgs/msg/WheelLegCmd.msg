# 轮腿机器人 6 个执行器的力矩指令 (N·m)
#
# 字段顺序与 mjcf/robot.xml 中 <actuator> 的定义顺序一致，
# 对应 MuJoCo 的 data.ctrl[0] ~ data.ctrl[5]。
#
# 注意：这是"控制器 → 仿真/硬件"的指令消息，不是状态消息。

float64 right_phi1    # 右腿近端关节 joint_right_phi1  -> ctrl[0]  (ctrlrange ±6.0)
float64 right_phi4    # 右腿远端关节 joint_right_phi4  -> ctrl[1]  (ctrlrange ±6.0)
float64 left_phi1     # 左腿近端关节 joint_left_phi1   -> ctrl[2]  (ctrlrange ±6.0)
float64 left_phi4     # 左腿远端关节 joint_left_phi4   -> ctrl[3]  (ctrlrange ±6.0)
float64 right_wheel   # 右轮        right_wheel_torque -> ctrl[4]  (ctrlrange ±2.0)
float64 left_wheel    # 左轮        left_wheel_torque  -> ctrl[5]  (ctrlrange ±2.0)
