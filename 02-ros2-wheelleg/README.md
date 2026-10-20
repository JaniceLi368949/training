# 第二次作业：ROS2 入门 + 轮腿平衡控制

面向 RoboMaster 算法组新队员。从零学 ROS2，最后完成一个大作业：
**把轮腿机器人的 LQR / VMC 控制解算放进独立的 ROS2 节点，通过话题驱动 MuJoCo 仿真里的执行器。**

> 这是培训仓库的第 2 个作业目录。与第 1 次（C++ 基础）不同，这次是从零学 ROS2 并做控制，
> 内容多得多——建议按下面「目录结构」里的章节顺序走。

**完整教程：** [ROS2 入门完整教程](ROS2_入门_完整教程.md)（仓库结构说明 + 00–09 全部章节 + 大作业）。

首次打开：如果自己的 Fork 还没有本目录，先按[仓库首页的同步步骤](../README.md#以后怎样获取新作业)更新。下面的路径以第一次作业使用的 `~/venom_cpp_basics/training/` 为例，克隆在其他位置时换成自己的纯英文路径。

---

## ⚠️ 先看这一条：路径不能含中文

ROS2 的接口生成工具（`rosidl`）**处理不了非 ASCII 路径**，会让 `rm_msgs` 构建失败：

```text
Target dependency '/home/xxx/RM_ROS2' does not exist      ← 路径在中文处被截断
CMake Error: list index: 1 out of range (-1, 0)
```

注意是**整条路径**都不能含中文，不只是本文件夹的名字：

| | 路径 | 结果 |
|---|---|---|
| ✅ | `/home/sysu/venom/training/02-ros2-wheelleg/...` | 正常 |
| ❌ | `~/下载/training/02-ros2-wheelleg/...` | 父目录"下载"含中文 → 失败 |
| ❌ | `~/桌面/venom_cpp_basics/...` | 失败 |

> 本机实测：`/tmp/中文路径测试/...` 构建失败，`/tmp/ascii_ok/...` 6 个包全部通过。
> 所以**克隆之前先把练习目录建成纯英文**，例如 `mkdir -p ~/venom && cd ~/venom`。

---

## 目录结构

```text
.
├── Tutorial/
│   ├── ROS2_入门/          # 教程正文（先看这里的 README）
│   │   ├── README.md       #   索引 + 学习路线 + VS Code 插件清单
│   │   ├── 00_为什么用ROS2.md
│   │   ├── 01_环境准备与工作区.md    ← 环境依赖都在这一章
│   │   ├── 02_turtlesim初体验.md
│   │   ├── 03_第一个Python节点.md
│   │   ├── 04_自定义消息.md
│   │   ├── 05_服务与参数.md
│   │   ├── 06_launch文件.md
│   │   ├── 07_TF2坐标变换.md
│   │   ├── 08_串口对接电控板.md
│   │   ├── 09_实战_MuJoCo接入ROS2.md  ← 大作业在第 5 节
│   │   └── images/
│   └── ros2_ws/            # 示例工作区（教程里每段代码都指向这里）
│       └── src/
│           ├── my_first_pkg/        # 第 3 章：talker / listener
│           ├── rm_msgs/             # 第 4 章：自定义消息（含 WheelLegCmd）
│           ├── rm_gimbal_demo/      # 第 5、6 章：服务 / 参数 / launch
│           ├── rm_tf_demo/          # 第 7 章：TF2
│           ├── wheel_leg_bridge/    # 第 9 章：教学版桥接（自己算轮力矩）
│           └── wheel_leg_sim/       # 第 9 章大作业：纯仿真节点（不含控制律）
└── Wheel_Leg/              # 轮腿机器人原工程（**参考用**，不要改）
    ├── LQR_VMC/            #   LQR + VMC 主控制代码、MuJoCo 模型
    ├── robot_vmc_test/     #   五连杆运动学验证
    └── robot_feedback_test/#   传感器可视化
```

> 工作区里**没有** `wheel_leg_controller`——那是你要自己写的包。

---

## 一、先装环境

**Ubuntu 22.04 + ROS 2 Humble**，再加一个 conda 环境放第三方依赖（MuJoCo 等）。
先按 [教程第 1 章](Tutorial/ROS2_入门/01_环境准备与工作区.md) 安装 ROS2 和 Miniforge（提供 `conda` 命令）；已经装好的不用重装。确认 `conda --version` 能运行，再创建环境：

```bash
# Python 必须是 3.10，与 Humble 的 rclpy 匹配
conda create -n rm_ros2 python=3.10 -y
conda activate rm_ros2
pip install 'setuptools<72' 'empy<4' 'lark==1.1.1' \
            mujoco numpy pyserial colcon-common-extensions
conda env config vars set PYTHONNOUSERSITE=1 -n rm_ros2
conda activate rm_ros2
```

> 四个版本约束（`setuptools<72`、`empy<4`、`lark`、`PYTHONNOUSERSITE=1`）都不是随便写的，
> 少一个就会构建失败或环境失效。原因见第 1 章第 4 节和第 6 节。

---

## 二、构建并自检

```bash
conda activate rm_ros2
source /opt/ros/humble/setup.bash
cd ~/venom_cpp_basics/training/02-ros2-wheelleg/Tutorial/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

自检：开两个终端，**每个终端都要激活环境并 source**，不能只在第一个终端做。

```bash
# 终端 A
conda activate rm_ros2
source /opt/ros/humble/setup.bash
cd ~/venom_cpp_basics/training/02-ros2-wheelleg/Tutorial/ros2_ws
source install/setup.bash
ros2 run my_first_pkg talker
```

```bash
# 终端 B
conda activate rm_ros2
source /opt/ros/humble/setup.bash
cd ~/venom_cpp_basics/training/02-ros2-wheelleg/Tutorial/ros2_ws
source install/setup.bash
ros2 run my_first_pkg listener
```

B 持续打印 `收到: "Hello ROS2: N"` 就通过；用 `Ctrl+C` 停止。

---

## 三、学习顺序

按 [教程 README](Tutorial/ROS2_入门/README.md) 的路线从 00 章顺下去。
完成环境准备后，先学 00～07 章，再做第 09 章。**第 08 章实体串口可选**，没有电控板可以先跳过；必做平衡作业不需要实体机器人、电控板或手柄。

第 09 章先运行教学版桥接节点，观察 `/imu`、`/joint_states` 话题：

```bash
ros2 run wheel_leg_bridge wheel_leg_bridge
```

这个节点没有 GUI，也没有平衡控制，车体会倒是预期现象。作业用的是另一个节点 `wheel_leg_sim`：默认可打开 GUI，但要收到你写的控制器发来的力矩才推进仿真。切换到作业前先用 `Ctrl+C` 停掉桥接节点，不要同时运行两个仿真器。

---

## 四、大作业

**题目在第 [09 章第 5 节](Tutorial/ROS2_入门/09_实战_MuJoCo接入ROS2.md)。**

一句话版本：`wheel_leg_sim` 是**纯仿真器**（不含任何控制律，不许改），
你要自己写一个 `wheel_leg_controller` 包，在里面完成 LQR + VMC 解算，
通过 `/actuator_cmd`（`rm_msgs/msg/WheelLegCmd`）把 6 路执行器力矩发回去。

硬性验收：**原地平衡 ≥ 5 s，机体 pitch 偏差 < 10°**。

作业要用到的解算代码（五连杆运动学、LQR 的 K 矩阵、腿长 PID）都在 `Wheel_Leg/LQR_VMC/` 里，
按第 9 章 5.9 节的说明复用即可——**控制算法不用你从零推导，考的是"把控制回路拆成两个 ROS2 节点"这件事**。

---

## 五、几个容易卡住的地方

| 现象 | 原因 / 解法 |
|---|---|
| `colcon build` 报 `option --uninstall not recognized` | setuptools 太新，`pip install 'setuptools<72'` |
| 构建接口包报 `em has no attribute BUFFERED_OPT` | empy 太新，`pip install 'empy<4'` |
| 构建接口包报 `No module named 'lark'` | `pip install 'lark==1.1.1'` |
| `ros2 run` 报 `No module named 'mujoco'` | 没在 conda 环境里构建，见第 1 章 4.3 |
| `import` 到的包不是环境里的 | 没设 `PYTHONNOUSERSITE=1` |
| 机器人"能站住但一直往前滑走" | 位置环失效，见第 9 章 5.9 节的 `x_b_set` 提醒 |
| 机器人明明代码没错但一直摔 | 可能同时开了 `wheel_leg_bridge`，话题冲突，见第 9 章 5.5 |

---

## 六、提交什么

见第 9 章 5.8 节：一个能 `colcon build` 通过的新包 + 说明文档 + 录屏或 bag 文件。该节提供推送命令；推送到**自己的 Fork**，提交仓库链接和录屏 / bag 链接，不要直接推送队伍公共仓库。
