# ROS2 入门完整教程：从节点通信到轮腿平衡控制
## 阅读导航
- [仓库结构与文件位置](#仓库结构与文件位置)
- [00 为什么用 ROS2](#00-为什么用-ros2)
- [01 环境准备与工作区](#01-环境准备与工作区)
- [02 turtlesim 初体验](#02-turtlesim-初体验)
- [03 第一个 Python 节点](#03-第一个-python-节点)
- [04 自定义消息](#04-自定义消息)
- [05 服务与参数](#05-服务与参数)
- [06 launch 文件](#06-launch-文件)
- [07 TF2 坐标变换](#07-tf2-坐标变换)
- [08 串口对接电控板](#08-串口对接电控板)
- [09 实战：MuJoCo 接入 ROS2](#09-实战mujoco-接入-ros2)
- [第 09 章第 5 节：大作业](#5-大作业把控制解算搬到-ros2-里)
- [附录：VS Code 插件](#附录vs-code-插件)
## 仓库结构与文件位置
这一节说明代码放在哪、每个文件是干什么的。先找到文件，再按章节动手。
### 一、总览
整套教程（文档 + 可运行代码 + 参考工程）都在同一个包根目录下：
```text
<包根>/
├── README.md                         # 入口：环境、构建自检、学习顺序、作业与排错
├── ROS2_入门_完整教程.md               # 完整教程
├── Tutorial/
│   ├── ROS2_入门/                    # 教程正文：00–09 共 10 篇 Markdown
│   │   ├── README.md                 # 索引、学习路线、VS Code 插件清单
│   │   ├── 00_为什么用ROS2.md
│   │   ├── 01_环境准备与工作区.md
│   │   ├── 02_turtlesim初体验.md
│   │   ├── 03_第一个Python节点.md
│   │   ├── 04_自定义消息.md
│   │   ├── 05_服务与参数.md
│   │   ├── 06_launch文件.md
│   │   ├── 07_TF2坐标变换.md
│   │   ├── 08_串口对接电控板.md
│   │   ├── 09_实战_MuJoCo接入ROS2.md   # 大作业在第 5 节
│   │   └── images/
│   ├── ros2_ws/                      # ROS2 工作区；所有构建都在这里做
│   │   ├── .gitignore
│   │   └── src/                      # 6 个功能包
│   └── tests/                        # 节点逻辑回归测试（不是作业参考解）
└── Wheel_Leg/                        # 轮腿机器人原工程；参考用，不要改
```
「包根」取决于你拿到代码的方式：
- 从培训仓库克隆：`training/02-ros2-wheelleg/`；
- 如果拿到的是单独分发、名为 `RM_ROS2_WheelLeg_HW` 的压缩包：解压后的 `RM_ROS2_WheelLeg_HW/`。
本节的路径都相对「包根」。后续章节中 `ros2_ws/` 是 `<包根>/Tutorial/ros2_ws/` 的简称；示例中的 `cd ros2_ws` 假定当前在 `<包根>/Tutorial/`，若已经在工作区中，不要重复进入。
### 二、`Tutorial/ros2_ws/src/` 的 6 个包
下表「关键文件」相对每个包自己的根目录。
| 包 | 作用 | 关键文件 |
|---|---|---|
| `my_first_pkg` | 第 3 章：发布者 / 订阅者；第 6 章：launch | `my_first_pkg/talker.py`、`my_first_pkg/listener.py`、`launch/talker_listener.launch.py` |
| `rm_msgs` | 第 4 章：自定义接口；用 `ament_cmake`，不是 `setup.py` | `msg/GimbalTarget.msg`、`msg/ChassisCmd.msg`、`msg/WheelLegCmd.msg`、`srv/SetGimbalAngle.srv`、`CMakeLists.txt` |
| `rm_gimbal_demo` | 第 5、6 章：服务 / 参数 / launch | `rm_gimbal_demo/gimbal_server.py`、`rm_gimbal_demo/gimbal_client.py`、`launch/gimbal.launch.py` |
| `rm_tf_demo` | 第 7 章：TF2 广播 / 监听 | `rm_tf_demo/tf_broadcaster.py`、`rm_tf_demo/tf_listener.py` |
| `wheel_leg_bridge` | 第 9 章：教学版桥接，自己把 `cmd_vel` 换算成轮力矩 | `wheel_leg_bridge/wheel_leg_bridge.py`、`robots/robot.xml`、`robots/obstacles.xml` |
| `wheel_leg_sim` | 第 9 章大作业：纯仿真节点，不含任何控制律，提供后冻结，学生不许改 | `wheel_leg_sim/wheel_leg_sim.py`、`robots/robot.xml`、`robots/obstacles.xml` |
> ⚠️ `src/` 里**没有 `wheel_leg_controller`**——那是大作业要你自己写的包。
### 三、每章对应哪些文件
| 章节 | 代码 / 文件位置（相对包根） |
|---|---|
| 00 为什么用 ROS2 | 纯概念，无代码 |
| 01 环境准备与工作区 | `Tutorial/ros2_ws/`（工作区）、`Tutorial/ros2_ws/src/`（包结构） |
| 02 turtlesim 初体验 | 无自写代码，使用官方 `turtlesim` 包 |
| 03 第一个 Python 节点 | `Tutorial/ros2_ws/src/my_first_pkg/` |
| 04 自定义消息 | `Tutorial/ros2_ws/src/rm_msgs/` |
| 05 服务与参数 | `Tutorial/ros2_ws/src/rm_gimbal_demo/rm_gimbal_demo/` 下的 `gimbal_server.py` / `gimbal_client.py` |
| 06 launch 文件 | `Tutorial/ros2_ws/src/my_first_pkg/launch/talker_listener.launch.py`、`Tutorial/ros2_ws/src/rm_gimbal_demo/launch/gimbal.launch.py` |
| 07 TF2 坐标变换 | `Tutorial/ros2_ws/src/rm_tf_demo/` |
| 08 串口对接电控板 | 骨架在正文里，没有预建包；需要自行封装并接入串口设备或虚拟串口 |
| 09 MuJoCo 接入 ROS2 | 教学版：`Tutorial/ros2_ws/src/wheel_leg_bridge/`；作业基座：`Tutorial/ros2_ws/src/wheel_leg_sim/` |
| 09 第 5 节（大作业） | 自己新建 `Tutorial/ros2_ws/src/wheel_leg_controller/` |
### 四、`Wheel_Leg/`：原工程与可复用的解算代码
| 文件（相对包根） | 用途 |
|---|---|
| `Wheel_Leg/LQR_VMC/robot_control_main.py` | 原版单进程控制主循环，移植蓝本 |
| `Wheel_Leg/LQR_VMC/vmc_cal.py` | 五连杆正运动学 + VMC 力矩映射 |
| `Wheel_Leg/LQR_VMC/lqr_update.py` | LQR 状态更新 + K 矩阵多项式 |
| `Wheel_Leg/LQR_VMC/roll_leg_control.py` | 横滚平衡：左右腿长差模分配 |
| `Wheel_Leg/LQR_VMC/ground_detection.py` | 离地检测 |
| `Wheel_Leg/LQR_VMC/util/climb_traj.py` | 速度爬坡滤波器 |
| `Wheel_Leg/LQR_VMC/remote/xbox_controller.py` | 手柄 / 键盘遥控，pygame 版 |
| `Wheel_Leg/LQR_VMC/robot_mujoco_sensor_feedback/robot_info_update.py` | 四元数→欧拉角、6 关节读取 |
| `Wheel_Leg/LQR_VMC/mjcf/robot.xml` | MuJoCo 模型：6 个执行器 + IMU / 关节传感器 |
| `Wheel_Leg/LQR_VMC/mjcf/obstacles.xml` | 场景里的 5 段斜坡，被 `robot.xml` include |
| `Wheel_Leg/robot_vmc_test/` | 五连杆运动学验证 |
| `Wheel_Leg/robot_feedback_test/` | 传感器可视化：位置 / 速度 / 欧拉角曲线 |
| `Wheel_Leg/cal_lqr_k.m` | MATLAB LQR 整定，输出多项式 K |
### 五、拿到代码后怎么跑
先完成第 01 章的 ROS2 安装和 conda 环境创建，再执行下面的命令。`<包根>` 是占位符，要换成真实路径，不要原样复制到 shell。
```bash
# 1) 环境
conda activate rm_ros2
source /opt/ros/humble/setup.bash

# 2) 构建：下面以克隆到 ~/venom/training 为例，按实际位置调整
cd ~/venom/training/02-ros2-wheelleg/Tutorial/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
两个终端都激活相同环境、source ROS2 和工作区后，再分别运行：
```bash
ros2 run my_first_pkg talker      # 终端 A
ros2 run my_first_pkg listener    # 终端 B：能收到消息就说明基础通信正常
```
### 六、两个必须知道的约束
1. **`ros2_ws` 所在的整条构建路径使用纯英文。** 本教程的接口生成工具链在中文路径下出现过截断和构建失败，即使 locale 为 UTF-8 也不能解决。建议先建 `mkdir -p ~/rm_hw`，不要放进 `~/下载/` 或 `~/桌面/`。文档目录 `ROS2_入门/` 不在构建路径上，含中文没有问题。
2. **大作业复用 `Wheel_Leg/` 的解算模块，不需要从零推导 LQR / VMC。** 考点是「把控制回路拆成两个 ROS2 节点」，不是重新实现控制算法。冻结的仿真节点不写控制律，你的新包只通过话题交换状态和力矩。
## 00 为什么用 ROS2
### 先看一个真实场景
RoboMaster 的机器人大致分这么几块，每块由不同的同学开发：
| 模块 | 干什么 |
|---|---|
| 底盘 | 麦轮/舵轮运动、里程计 |
| 云台 | yaw/pitch 控制、自瞄 |
| 视觉 | 相机目标检测、测距 |
| 决策 | 综合信息、发指令 |
| 电控/裁判系统 | 读裁判系统、控电机 |
如果所有人写**一个大脚本**，会出现什么情况？
- 视觉组改了相机驱动，底盘组要重新拉代码、重新编译、可能崩。
- 一个人的 bug 拖垮整个程序，还找不到是谁的锅。
- 无法单独调试某一个模块，也无法快速替换（比如换个相机型号）。
**ROS2 的核心思路就是：把每个模块拆成独立的"节点"（node），节点之间通过"话题"（topic）通信。**
### ROS2 是什么
ROS2（Robot Operating System 2）不是操作系统，而是一套**机器人中间件框架**：
- 提供**通信机制**（话题/服务/动作），让不同进程、甚至不同机器之间互相传数据；
- 提供**工具链**（`ros2` 命令、rqt、rviz2、TF 等），方便调试和可视化；
- 提供**包管理**（colcon 构建、package 组织代码）。
对我们最直接的三个好处：
1. **解耦**：视觉节点只负责发 `/camera/target`，云台节点只负责订阅它。两边可以独立开发、独立测试、独立替换。
2. **可视化调试**：用 `rqt_graph` 直接看节点和话题的拓扑，用 `ros2 topic echo` 实时看数据流，不用打日志猜。
3. **生态**：相机、激光雷达、仿真器（Gazebo/MuJoCo 接口）大多有现成的 ROS2 驱动，直接复用。
### 核心概念速览（后面章节会逐一展开）
| 概念 | 一句话解释 | 类比 |
|---|---|---|
| **节点 Node** | 一个独立运行的程序 | 一个工人 |
| **话题 Topic** | 节点间单向数据流（发布/订阅） | 广播电台（一个台，多个听众） |
| **消息 Message** | 话题里传的数据结构 | 广播的内容格式 |
| **服务 Service** | 一次请求 → 一次响应 | 打电话问个问题 |
| **动作 Action** | 长时间任务，可取消、可反馈 | 派活 + 汇报进度 |
| **参数 Parameter** | 节点运行时可调的配置 | 工人的工作参数 |
| **TF** | 坐标系之间的变换 | 地图 + 各部件的位置 |
### ROS2 vs ROS1（听过 ROS1 的朋友看这里）
- ROS2 用 **DDS** 通信，不再依赖 ROS1 的 master（不再有单点故障）。
- 命令从 `roscore`/`rosrun` 变成 `ros2 run`、`ros2 launch`。
- 生命周期、QoS（服务质量）、实时性支持更好。
我们直接学 ROS2 即可，不用先学 ROS1。
### 我们队里会用 ROS2 做什么
结合库里现有的 [Wheel_Leg](Wheel_Leg/README.md)（MuJoCo 轮足仿真）和队里的电控框架（STM32，需向电控组另行获取，本仓库不包含），一个典型架构长这样：
```text
┌─────────────┐   /imu + /joint_states   ┌──────────────┐
│ 模型策略      │ ───────────────────────▶ │  决策/控制节点 │
│ (wheel_leg) │                          │  (订阅状态)    │
└─────────────┘   ◀───────────────────── └──────────────┘
                  /cmd_vel（速度指令）

┌─────────────┐   /serial/tx / /serial/rx   ┌──────────────┐
│ STM32 电控板 │ ◀────────────────────────▶ │  串口桥接节点  │
└─────────────┘                             └──────────────┘
```
第 9 章会带你亲手实现上面第一行（仿真 ↔ 控制节点）。
## 01 环境准备与工作区
### 1. 安装 ROS 2
本教程统一以 **Ubuntu 22.04 + ROS 2 Humble + Python 3.10** 为构建和运行验收环境，后续命令也按此组合编写。
> Ubuntu 24.04 对应 **ROS 2 Jazzy + Python 3.12**，不能只把包名中的 `humble` 换成 `jazzy` 后继续照抄 Python 3.10 的 conda 配置。本教程尚未验证 Jazzy；为避免混用 ABI，新队员建议使用 Ubuntu 22.04 完成本教程。
#### 方式一：鱼香ROS 一键安装（推荐）
国内网络下最省事的方式，社区维护的脚本，会自动帮你换源，装完还能顺手把 `rosdep` 配好：
```bash
source <(wget -qO- https://fishros.com/install)
```
运行后按菜单走：
1. 选 **安装 ROS2** → 在 Ubuntu 22.04 上选 `humble` → 选 `desktop` 桌面版。
2. 装完再回主菜单选一次 **配置 rosdep**——官方 `rosdep init/update` 在国内经常拉不下来，这一步能省很多事。
> 提醒：这是**社区脚本**（非官方）。`source <(...)` 会把下载到的内容直接执行，介意的话用下面的方式二或方式三。
#### 方式二：官方文档
官方完整安装文档：<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>
#### 方式三：手动精简步骤（桌面版，含 rviz/rqt/turtlesim 演示）
```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-turtlesim python3-colcon-common-extensions ros-dev-tools
```
> 上述命令针对 Ubuntu 22.04 / Humble，请勿在 Ubuntu 24.04 上直接执行。
### 2. source 环境
#### 先理解 `source` 在做什么
ROS2 装到了 `/opt/ros/humble`，但装完它的命令（`ros2`、`colcon`…）**默认不在 `PATH` 里**，所以每开一个终端都要先把环境"加载"进来：
```bash
source /opt/ros/humble/setup.bash
ros2 --help        # 能打印命令帮助就成功了
```
`source` 的作用是**在"当前这个 shell"里执行脚本**，而不是新开一个进程。`setup.bash` 里设置的一堆环境变量（`PATH`、`PYTHONPATH`、`AMENT_PREFIX_PATH` 等）因此会留在当前终端里，这就是"加载环境"。
反过来，`./setup.bash` 或 `bash setup.bash` **都不行**——它们会在一个子进程里执行，子进程一结束，设置好的变量就跟着消失了。
#### `~/.bashrc` 是什么
每次开新终端都手动 source 太麻烦，所以把它写进 `~/.bashrc`：
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc          # 让当前这个终端立即生效
```
`~/.bashrc` 可以理解为**你的终端启动脚本**，要点如下：
| | 说明 |
|---|---|
| 位置 | `~/.bashrc`；`~` 是家目录，所以完整路径是 `/home/你的用户名/.bashrc` |
| 为什么有点 | 文件名开头的 `.` 表示**隐藏文件**，`ls` 默认看不到，要用 `ls -a`，或者直接 `cat ~/.bashrc` 看内容 |
| 什么时候执行 | **每开一个新终端就自动跑一遍** |
| 放什么 | 每个终端都需要的东西：环境变量、`alias`、`source xxx` |
| 改完何时生效 | 只对**之后新开的**终端生效；当前终端要手动 `source ~/.bashrc` |
SSH 登录的情况：登录时 bash 先读 `~/.bash_profile`，没有才读 `~/.profile`。Ubuntu 默认不创建 `.bash_profile`，而系统自带的 `~/.profile` 里已经有一句会去 source `~/.bashrc`，所以正常不用额外处理。
#### ⚠️ 两个新手必踩的坑
1. **`>>` 和 `>` 完全不同**：`>>` 是**追加**，`>` 是**覆盖**（会清空原文件）。写成 `echo "..." > ~/.bashrc` 就把你的 `.bashrc` 清空了，之前的配置全丢。这类操作**永远用 `>>`**。
2. **别把 `.bashrc` 改坏**：它在每个终端启动时都会执行，一旦有语法错误，之后**每个**终端都会报错。改之前先备份，心里有底：
```bash
cp ~/.bashrc ~/.bashrc.bak      # 出错就 cp ~/.bashrc.bak ~/.bashrc 还原
```
#### 验证
关掉终端，**重新开一个**，不要手动 source，直接运行：
```bash
ros2 --help
```
能打印帮助，说明 `.bashrc` 里那行已经生效了。
### 3. 工作区（workspace）是什么
写自己的代码需要一个工作区，约定放在 `ros2_ws/`：
```text
ros2_ws/
├── src/        # 源码：每个子目录是一个"包"（package）
├── build/      # 构建中间产物（colcon 自动生成）
├── install/    # 构建结果 + 环境脚本（colcon 自动生成）
└── log/        # 构建日志
```
一个"包"是最小编译单元，同时带一个 `package.xml`（描述元信息）。本教程的 `src/` 下有 6 个包：
```text
src/
├── my_first_pkg/      # Python 包（发布/订阅示例）
├── rm_msgs/           # 接口包（自定义消息，ament_cmake）
├── rm_gimbal_demo/    # 服务/参数/launch 示例
├── rm_tf_demo/        # TF 示例
├── wheel_leg_bridge/  # 第9章：MuJoCo 桥接（教学版）
└── wheel_leg_sim/     # 第9章作业：纯仿真节点（不含任何控制律）
```
### 4. Python 环境：conda（第 8、9 章需要）
前 7 章只用 ROS2 自带的包，不用额外装东西。但从第 8 章（串口）和第 9 章（MuJoCo 仿真）起需要**第三方 Python 包**，而 ROS2 Humble 是 apt 装在**系统 Python 3.10** 上的——直接往系统 Python 里 `pip install` 会污染整机环境（队里别的项目也在用它）。
所以约定：**单独建一个 conda 环境放这些依赖**，并且**在这个环境里构建我们的工作区**。
#### 4.1 创建环境
若还没有 `conda` 命令，先按 [Miniforge 安装说明](https://github.com/conda-forge/miniforge#install) 安装对应 CPU 架构的版本（普通 PC 选 x86_64，Jetson 选 aarch64），初始化 shell 后重开终端。
```bash
conda create -n rm_ros2 python=3.10 -y
conda activate rm_ros2
```
> **Python 必须 3.10**：ROS2 Humble 的 `rclpy` 是给 CPython 3.10 编译的（ABI 标签 `cpython-310`），3.11/3.12 直接 import 不了。
#### 4.2 依赖清单
| 包 | 版本 | 为什么需要 |
|---|---|---|
| `mujoco` | 3.16.0 | 第 9 章的物理仿真引擎 |
| `numpy` | 2.2.6 | MuJoCo 依赖，解算代码也要用 |
| `pyserial` | 3.5 | 第 8 章读写串口 |
| `colcon-common-extensions` | 0.3.0 | **在这个环境里构建工作区**（原因见 4.3） |
| `setuptools` | `<72` | colcon 依赖的 `setup.py develop` 在 72 版被移除 |
| `empy` | `<4` | `rosidl_adapter` 用了 `em.BUFFERED_OPT`，4.x 删掉了 |
| `lark` | `1.1.1` | `rosidl_parser` 用它解析 `.idl`；系统上由 apt 提供，隔离环境要自己装 |
一条命令装齐：
```bash
pip install 'setuptools<72' 'empy<4' 'lark==1.1.1' \
            mujoco numpy pyserial colcon-common-extensions
```
再关掉 `user site`，免得 `~/.local` 里的包把环境里的覆盖掉：
```bash
conda env config vars set PYTHONNOUSERSITE=1 -n rm_ros2
conda activate rm_ros2          # 重新激活，让这个变量生效
```
> 为什么要关：CPython 把 `~/.local/lib/python3.10/site-packages` 排在 conda 环境的 `site-packages` **前面**。不关掉的话，即使你装进了环境，`import` 到的仍是用户目录里的版本，环境等于没隔离。
#### 4.3 为什么必须在环境里构建
`colcon build` 会给每个 Python 节点生成一个可执行脚本，**脚本第一行的 shebang 记录的是"构建时用的那个 python"**。如果用系统 python 构建，之后 `ros2 run` 就会用系统 python 跑节点，而系统 python 里没有 `mujoco`，于是报 `ModuleNotFoundError: No module named 'mujoco'`。
所以在环境里构建，让 shebang 指向环境：
```bash
conda activate rm_ros2
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
```
确认 shebang 对不对：
```bash
head -1 install/wheel_leg_sim/lib/wheel_leg_sim/wheel_leg_sim
# 期望：#!/home/<你>/miniconda3/envs/rm_ros2/bin/python3.10
```
#### 4.4 日常使用姿势
每个新终端都要**先激活环境、再 source ROS2**：
```bash
conda activate rm_ros2
source /opt/ros/humble/setup.bash
cd ros2_ws && source install/setup.bash
ros2 run wheel_leg_sim wheel_leg_sim
```
嫌啰嗦可以写进 `~/.bashrc`（注意第 2 节讲的 `>>` 别写成 `>`）：
```bash
echo 'conda activate rm_ros2' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```
### 5. 构建：colcon
```bash
cd ~/.../ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash     # 构建后要 source 自己工作区的 setup
```
- `--symlink-install`：Python 文件以软链接方式装进 `install/`，改代码后**不需要重新 build**（C++ 包仍需重新编译）。强烈推荐开发时加上。
- 构建成功后：`build/` `install/` `log/` 三个目录被生成，`install/setup.bash` 让你能 `ros2 run` 自己的包。
**只构建某个包**（省时间）：
```bash
colcon build --symlink-install --packages-select my_first_pkg
```
### 6. 环境踩坑速查（重要）
下面几条都是踩过并已修复的，重装或换机器时按顺序对照。
#### 坑 A：setuptools ≥ 72 让 colcon 报错
```text
error: option --uninstall not recognized
error: option --editable not recognized
```
原因：setuptools 从 72 版起移除了 colcon 依赖的 `setup.py develop`。修复（在环境里）：
```bash
conda activate rm_ros2
pip install 'setuptools<72'
python3 -c "import setuptools; print(setuptools.__version__)"   # 应 < 72
```
#### 坑 B：构建接口包报 `em` 相关错误
```text
AttributeError: module 'em' has no attribute 'BUFFERED_OPT'
```
原因：`rosidl_adapter` 需要 EmPy 3.x，而 pip 默认装的是 4.x。修复：
```bash
pip install 'empy<4'
```
#### 坑 C：构建接口包报 `No module named 'lark'`
`rosidl_parser` 用 lark 解析 `.idl`。系统上它由 apt 包提供，隔离的 conda 环境里没有：
```bash
pip install 'lark==1.1.1'
```
#### 坑 D：`import` 到的包不是环境里的
症状：明明在环境里装了某个包，`import` 却指向 `~/.local/...`。因为 CPython 把 user site 排在环境 site-packages 前面。按 4.2 设好 `PYTHONNOUSERSITE=1`，然后确认：
```bash
python3 -c "import sys; print('\n'.join(sys.path))"   # 输出里不应出现 ~/.local
```
#### 坑 E：`ros2 run` 报 `No module named 'mujoco'`
说明工作区是用系统 python 构建的（shebang 指向 `/usr/bin/python3`）。按 4.3 在环境里重新构建：
```bash
conda activate rm_ros2 && source /opt/ros/humble/setup.bash
rm -rf build install log && colcon build --symlink-install
```
#### 坑 F：工作区路径不能含中文
症状：构建接口包（`rm_msgs`）时报
```text
Target dependency '/home/xxx/RM_ROS2' does not exist      ← 路径在中文处被截断
CMake Error: list index: 1 out of range (-1, 0)
```
原因：ROS2 的接口生成工具 `rosidl` **处理不了非 ASCII 路径**。注意**整条路径**都不能含中文，不只是工作区那一层：
| | 路径 | 结果 |
|---|---|---|
| ✅ | `/home/sysu/rm_hw/ros2_ws` | 正常 |
| ❌ | `~/下载/rm_hw/ros2_ws` | 父目录含中文 → 失败 |
| ❌ | `~/桌面/ros2_ws` | 失败 |
所以工作区要放在纯英文路径下，例如：
```bash
mkdir -p ~/rm_hw && cd ~/rm_hw     # 之后把 ros2_ws 放在这里
```
> 本机实测：`/tmp/中文路径测试/...` 构建失败，`/tmp/ascii_ok/...` 正常。这条与 locale 无关（UTF-8 locale 下同样失败）。
> 文档目录（如 `ROS2_入门/`）含中文没问题——只要它不在构建路径上。
### 7. VS Code 插件（可选但推荐）
安装清单见[附录：VS Code 插件](#附录vs-code-插件)。装好后打开 `ros2_ws` 目录，`Ctrl+Shift+B` 可以直接跑 colcon 构建任务（由 ROS 扩展提供）。
## 02 turtlesim 初体验
先用官方自带的小海龟 demo，**不写一行代码**，直观感受节点、话题、服务长什么样。
### 1. 启动海龟
开两个终端，都要先 `source /opt/ros/humble/setup.bash`：
```bash
# 终端 A：启动仿真（会弹出一个窗口，中间有只海龟）
ros2 run turtlesim turtlesim_node
```
```bash
# 终端 B：键盘控制（保持此终端焦点，用方向键移动海龟）
ros2 run turtlesim turtle_teleop_key
```
按方向键，海龟会移动。现在是两个节点在控制同一只海龟，但我们还没写代码——这就是 ROS2：**两个独立的程序，通过"话题"通信**。
### 2. 看节点（node）
新开终端 C：
```bash
ros2 node list
# 输出：
# /turtlesim
# /teleop_turtle
```
`/turtlesim` 是仿真节点，`/teleop_turtle` 是键盘节点。看某个节点的详细信息：
```bash
ros2 node info /turtlesim
```
会列出它订阅了哪些话题、发布了哪些话题、提供了哪些服务。
### 3. 看话题（topic）
```bash
ros2 topic list          # 列出所有话题
ros2 topic list -t       # 同时显示每个话题的消息类型
```
重点看 `/turtle1/cmd_vel`（键盘节点发速度指令，海龟订阅它）。
**实时看数据流**：
```bash
ros2 topic echo /turtle1/pose     # 终端 D：持续打印海龟位姿
```
回到终端 B 按方向键，D 里 `x/y/theta` 会变。
**手动发一条速度指令**（不用键盘，直接发消息让海龟转圈）：
```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
`--once` 发一条就退出；换成 `-r 10` 表示以 10 Hz 持续发。
### 4. 看服务（service）
话题是"广播"（一个发、多个听），服务是"一问一答"（request → response）。
```bash
ros2 service list                       # 列出服务
ros2 service type /spawn                # 查服务类型
ros2 service list -t                    # 带类型
```
用 `spawn` 服务再生成一只海龟：
```bash
ros2 service call /spawn turtlesim/srv/Spawn \
  "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"
```
窗口里会出现第二只海龟。用 `kill` 服务删掉它：
```bash
ros2 service call /kill turtlesim/srv/Kill "{name: 'turtle2'}"
```
### 5. 可视化拓扑：rqt_graph
```bash
rqt_graph
```
会弹出一个图，椭圆是**节点**，方框是**话题**，箭头表示数据流向。这就是我们调试复杂系统时最常用的"地图"。
### 6. 心智模型总结
```text
节点(Node) ──发布(publish)──▶ 话题(Topic) ──订阅(subscribe)──▶ 节点(Node)
   │                                                            ▲
   └────────────── 请求/响应 服务(Service) ──────────────────────┘
```
记住三句话：
1. **节点** = 独立程序，`ros2 node list` 看。
2. **话题** = 数据流，`ros2 topic echo/pub` 读写。
3. **服务** = 一问一答，`ros2 service call` 调用。
下一章开始自己写节点。
## 03 第一个 Python 节点
从这一章开始写代码。目标：写一个**发布者**（每 0.5 秒发一条消息）和一个**订阅者**（收到就打印），看它们互相通信。
对应代码：`ros2_ws/src/my_first_pkg/`。
### 1. 包的结构
先看一个 Python 包长什么样：
```text
my_first_pkg/
├── package.xml                 # 包元信息（名字、依赖、构建类型）
├── setup.py                    # 安装配置，声明"可执行节点"
├── setup.cfg                   # 固定写法，一般不用改
├── resource/my_first_pkg       # 空标记文件（约定）
└── my_first_pkg/               # 实际 Python 代码目录（包名重复一层）
    ├── __init__.py
    ├── talker.py               # 发布者节点
    └── listener.py             # 订阅者节点
```
#### package.xml（关键字段）
```xml
<package format="3">
  <name>my_first_pkg</name>
  <version>0.0.0</version>
  <description>...</description>
  <maintainer email="rm@example.com">RM Team</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>      <!-- Python 客户端库 -->
  <exec_depend>std_msgs</exec_depend>   <!-- 标准消息 -->

  <export>
    <build_type>ament_python</build_type>  <!-- 纯 Python 包 -->
  </export>
</package>
```
#### setup.py（决定哪些是可执行节点）
```python
entry_points={
    'console_scripts': [
        'talker = my_first_pkg.talker:main',     # 命令名 = 模块:入口函数
        'listener = my_first_pkg.listener:main',
    ],
},
```
这一行把 `ros2 run my_first_pkg talker` 映射到 `my_first_pkg/talker.py` 里的 `main()`。
### 2. 发布者 talker.py
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):                          # 继承 Node
    def __init__(self):
        super().__init__('talker')           # 节点名字叫 'talker'
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS2: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'发布: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)                    # 初始化客户端库
    node = Talker()
    rclpy.spin(node)                         # 让节点保持运行
    node.destroy_node()
    rclpy.shutdown()
```
逐行理解：
- `create_publisher(String, 'chatter', 10)`：向话题 `chatter` 发 `String` 消息，`10` 是发送队列长度。
- `create_timer(0.5, cb)`：每 0.5 秒调用一次 `timer_callback`（周期性任务靠定时器，不是死循环）。
- `rclpy.spin(node)`：阻塞，等待回调/定时器触发。**节点自己不会"转"，靠 spin 驱动。**
### 3. 订阅者 listener.py
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String, 'chatter', self.callback, 10)

    def callback(self, msg):                 # 收到消息自动回调
        self.get_logger().info(f'收到: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
- `create_subscription(String, 'chatter', cb, 10)`：订阅 `chatter`，收到 `String` 就调 `callback`。
- **发布者和订阅者只通过"话题名 + 消息类型"匹配**，彼此不知道对方存在。
### 4. 构建并运行
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
开两个终端（都 source 过 `install/setup.bash`）：
```bash
# 终端 A：发布
ros2 run my_first_pkg talker
# 终端 B：订阅
ros2 run my_first_pkg listener
```
B 里应该持续打印 `收到: "Hello ROS2: N"`。按下 `Ctrl+C` 停止。
### 5. 用命令行验证
不启动 listener，直接用命令行订阅：
```bash
ros2 run my_first_pkg talker      # 终端 A
ros2 topic echo /chatter          # 终端 B，直接看话题数据
```
还能看话题信息：
```bash
ros2 topic list -t               # 找 /chatter 的类型
ros2 topic info /chatter         # 看发布者/订阅者数量
ros2 topic hz /chatter           # 看发布频率（应为 2.0 Hz）
```
### 6. 本节要点
1. 一个节点 = 继承 `Node` 的类 + `main()` 里 `init/spin/shutdown`。
2. 发布用 `create_publisher` + `publish()`；订阅用 `create_subscription` + 回调。
3. `package.xml` 声明依赖，`setup.py` 的 `entry_points` 声明可执行命令。
4. 加了新包/改了 setup.py 后要重新 `colcon build`（`--symlink-install` 下改 `.py` 代码本身不用重建）。
## 04 自定义消息
第 3 章用的 `String` 是别人定义好的标准消息。实际开发里，我们需要**自定义消息类型**来表达队里自己的数据。
对应代码：`ros2_ws/src/rm_msgs/`（接口包）。
### 1. 为什么需要自定义消息
想象云台控制：视觉节点要告诉云台节点"目标在 yaw=0.3 rad、pitch=0.5 rad"。用标准消息很别扭，不如定义一个 `GimbalTarget`：
```text
float64 yaw      # 目标 yaw 角 (rad)
float64 pitch    # 目标 pitch 角 (rad)
```
这样两个节点只要都 import 这个类型，就能"说同一门语言"。
本包定义了 4 个接口（其中 `WheelLegCmd` 在第 9 章大作业中使用）：
| 接口 | 类型 | 用途 |
|---|---|---|
| `rm_msgs/msg/GimbalTarget` | 消息 | 云台目标角 |
| `rm_msgs/msg/ChassisCmd` | 消息 | 底盘速度指令 |
| `rm_msgs/msg/WheelLegCmd` | 消息 | 轮腿机器人 6 路执行器力矩 |
| `rm_msgs/srv/SetGimbalAngle` | 服务 | 设置云台角（请求+响应） |
### 2. .msg 和 .srv 语法
**消息**（`.msg`）就是字段列表，每行 `类型 字段名`：
```text
# rm_msgs/msg/ChassisCmd.msg
float64 vx       # 前进速度 (m/s)
float64 vy       # 横移速度 (m/s)
float64 wz       # 自转角速度 (rad/s)
```
**服务**（`.srv`）分两段，用 `---` 分隔，上面是请求、下面是响应：
```text
# rm_msgs/srv/SetGimbalAngle.srv
float64 yaw
float64 pitch
---
bool success
string message
```
### 3. 接口包的结构（重要：用 ament_cmake）
接口包和普通 Python 包**不一样**——它必须用 **`ament_cmake`** 构建（即使业务代码全是 Python），因为要生成多语言的绑定代码。
```text
rm_msgs/
├── package.xml
├── CMakeLists.txt          # 接口包用 CMake，不是 setup.py！
├── msg/
│   ├── GimbalTarget.msg
│   ├── ChassisCmd.msg
│   └── WheelLegCmd.msg
└── srv/
    └── SetGimbalAngle.srv
```
#### package.xml
```xml
<package format="3">
  <name>rm_msgs</name>
  ...
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
关键点：
- `rosidl_default_generators`：生成 C++/Python 等语言的代码。
- `<member_of_group>rosidl_interface_packages</member_of_group>`：**必须加**，标记"这是个接口包"。
#### CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(rm_msgs)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalTarget.msg"
  "msg/ChassisCmd.msg"
  "msg/WheelLegCmd.msg"
  "srv/SetGimbalAngle.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```
`rosidl_generate_interfaces` 一行列出所有 `.msg/.srv`，build 时会自动生成 Python 模块 `rm_msgs.msg` 和 `rm_msgs.srv`。
### 4. 构建并验证
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rm_msgs
source install/setup.bash
```
验证接口已生成：
```bash
ros2 interface list | grep rm_msgs
# 输出：
#   rm_msgs/msg/ChassisCmd
#   rm_msgs/msg/GimbalTarget
#   rm_msgs/msg/WheelLegCmd
#   rm_msgs/srv/SetGimbalAngle

ros2 interface show rm_msgs/msg/GimbalTarget
# 打印字段定义
```
### 5. 在其他 Python 包里使用
```python
from rm_msgs.msg import GimbalTarget, ChassisCmd
from rm_msgs.srv import SetGimbalAngle
```
使用时在 `package.xml` 里加 `<exec_depend>rm_msgs</exec_depend>`（第 5 章会演示）。
### 6. 本节要点
1. 自定义接口 = 写 `.msg` / `.srv` 文件。
2. **接口包用 ament_cmake**，靠 `CMakeLists.txt` 里的 `rosidl_generate_interfaces` 生成代码。
3. 一定要在 package.xml 加 `<member_of_group>rosidl_interface_packages</member_of_group>`。
4. 命名建议：字段名小写下划线（`target_yaw`），注释用 `#`。
## 05 服务与参数
第 3 章的话题是"广播"，单向、谁都能听。这一章学**服务**（一问一答）和**参数**（可调配置）。
对应代码：`ros2_ws/src/rm_gimbal_demo/`。
### 1. 服务（Service）是什么
| | 话题 Topic | 服务 Service |
|---|---|---|
| 方向 | 单向（发布 → 订阅） | 请求 → 响应 |
| 语义 | 持续的数据流 | 一次调用、一次回答 |
| 例子 | 视觉持续发目标位置 | 决策节点请求"设置云台角" |
典型用法：`client` 发出请求，`server` 处理并返回结果。
### 2. 服务端 gimbal_server.py
```python
from rm_msgs.srv import SetGimbalAngle   # 第 4 章定义的服务


class GimbalServer(Node):
    def __init__(self):
        super().__init__('gimbal_server')
        # 声明参数：云台角度限幅（可被 launch 覆盖）
        self.declare_parameter('max_yaw', 3.14)
        self.declare_parameter('max_pitch', 1.57)
        # 提供服务：类型 + 服务名 + 回调
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
        return response
```
要点：
- `create_service(类型, 名字, 回调)`：注册服务。
- 回调签名是 `(request, response) -> response`：改 `response` 的字段并返回。
- `declare_parameter(name, default)`：声明参数并给默认值。
### 3. 客户端 gimbal_client.py
```python
class GimbalClient(Node):
    def __init__(self):
        super().__init__('gimbal_client')
        self.client = self.create_client(SetGimbalAngle, 'set_gimbal_angle')
        # 等服务端上线（最多等 1 秒再重试）
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务端上线...')

    def send(self, yaw, pitch):
        req = SetGimbalAngle.Request()   # 构造请求
        req.yaw = yaw
        req.pitch = pitch
        future = self.client.call_async(req)      # 异步调用
        rclpy.spin_until_future_complete(self, future)  # 等结果
        return future.result()                    # 拿到 response
```
要点：
- `create_client(类型, 名字)` 创建客户端。
- `wait_for_service` 阻塞等 server（避免请求发空）。
- `call_async` + `spin_until_future_complete` 是标准异步调用写法。
### 4. 运行验证
```bash
cd ros2_ws && source install/setup.bash

# 终端 A：启动服务端
ros2 run rm_gimbal_demo gimbal_server

# 终端 B：调用（传 yaw=0.8, pitch=0.4）
ros2 run rm_gimbal_demo gimbal_client 0.8 0.4
# 输出：服务返回: success=True, 设置成功: yaw=0.80, pitch=0.40
```
**不写客户端，用命令行直接调服务**：
```bash
ros2 service call /set_gimbal_angle rm_msgs/srv/SetGimbalAngle \
  "{yaw: 5.0, pitch: -2.0}"
```
yaw=5.0 会被限幅到 3.14（默认 `max_yaw`）。
### 5. 参数（Parameter）
参数是"运行时可调、无需改代码"的配置。命令行操作：
```bash
ros2 param list                          # 列出所有节点参数
ros2 param get /gimbal_server max_yaw    # 读值
ros2 param set /gimbal_server max_yaw 1.0   # 运行时改值
```
也可以**启动时用参数覆盖默认值**（`-p` 或 `--ros-args`）：
```bash
ros2 run rm_gimbal_demo gimbal_server --ros-args -p max_yaw:=1.2 -p max_pitch:=0.6
```
### 6. 一个必踩的大坑：别把方法命名成 `handle`
看下面这段**会崩**的代码：
```python
class GimbalServer(Node):
    def __init__(self):
        super().__init__('gimbal_server')
        self.srv = self.create_service(SetGimbalAngle, 'set_gimbal_angle', self.handle)  # 错！

    def handle(self, request, response):   # 方法名叫 handle
        ...
```
运行会报：
```text
AttributeError: __enter__
```
原因：`Node` 基类本身就有一个 **`handle` 属性**（property，返回底层节点句柄）。子类定义同名方法会**遮蔽**基类属性，导致 `Node.__init__` 内部 `with self.handle:` 拿到的是你的方法（而非句柄），于是崩溃。
**结论**：给回调/方法起名时避开 `handle`、`name` 等 `Node` 内置属性名。用 `set_gimbal_angle_callback` 这种带语义的名字就安全。
> 其它容易踩的保留名：`handle`（句柄）、`name`（节点名相关）。拿不准就用"动词 + 宾语"的命名，如 `on_cmd_vel`、`spawn_turtle`。
### 7. 本节要点
1. 服务 = 请求/响应；`create_service` / `create_client`。
2. 参数 = 可调配置；`declare_parameter` / `get_parameter`。
3. 命令行：`ros2 service call`、`ros2 param get/set`。
4. **方法名别用 `handle`**。
## 06 launch 文件
前面都是一个一个 `ros2 run` 启动节点。真实系统里动辄十几个节点，手动起不现实。**launch 文件**让你一次启动多个节点、顺便传参。
对应代码：各包的 `launch/*.launch.py`。
### 1. 最简单的 launch
`ros2_ws/src/my_first_pkg/launch/talker_listener.launch.py`：
```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='my_first_pkg', executable='talker'),
        Node(package='my_first_pkg', executable='listener'),
    ])
```
- 一个 `Node(...)` 就对应一次 `ros2 run <package> <executable>`。
- `generate_launch_description()` 返回一个 `LaunchDescription`，列出要启动的东西。
### 2. 运行
```bash
cd ros2_ws && source install/setup.bash
ros2 launch my_first_pkg talker_listener.launch.py
```
一个命令同时起两个节点，`Ctrl+C` 一次全停。
### 3. 带参数的 launch
`ros2_ws/src/rm_gimbal_demo/launch/gimbal.launch.py`：
```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_gimbal_demo',
            executable='gimbal_server',
            name='gimbal_server',
            parameters=[{'max_yaw': 1.2, 'max_pitch': 0.6}],   # 覆盖参数
        ),
        Node(
            package='rm_gimbal_demo',
            executable='gimbal_client',
            name='gimbal_client',
            arguments=['0.8', '0.4'],                           # 传给 main 的 argv
        ),
    ])
```
要点：
- `name=`：重命名节点（默认用代码里的名字）。
- `parameters=[{...}]`：覆盖节点的参数（对应第 5 章的 `declare_parameter`）。
- `arguments=[...]`：像命令行参数一样传给节点（`gimbal_client` 里用 `sys.argv[1]` 读）。
运行并观察参数生效（0.8/0.4 在 1.2/0.6 限幅内，所以原样返回）：
```bash
ros2 launch rm_gimbal_demo gimbal.launch.py
```
### 4. launch 文件要装进包里
`setup.py` 里要声明 launch 目录，否则 `ros2 launch` 找不到：
```python
import os
from glob import glob

data_files=[
    ...
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
],
```
### 5. 更高级（了解即可）
- **DeclareLaunchArgument**：在命令行传入启动参数（`ros2 launch ... param:=value`）。
- **IncludeLaunchDescription**：一个 launch 里嵌套另一个 launch。
- **条件/延时**：`IfCondition`、`TimerAction` 等。
初学先把 `Node` + `parameters` + `arguments` 用熟，其余用到再查。
### 6. 本节要点
1. launch = 一次启动多节点；`Node(package=, executable=)` 是基本单位。
2. `parameters` 传参、`arguments` 传 argv。
3. launch 文件要写进 `setup.py` 的 `data_files`。
4. 调试技巧：`ros2 launch` 出问题先单独 `ros2 run` 每个节点定位。
## 07 TF2 坐标变换
### 1. 为什么 RM 必须懂 TF
机器人上有很多坐标系，每个传感器/部件的数据都在**自己的坐标系**里：
```text
base_link（底盘）──▶ gimbal_link（云台）──▶ camera_link（相机）
```
视觉节点在 `camera_link` 坐标系下看到目标，但底盘要在 `base_link` 下运动。要"翻译"这两个坐标系之间的关系，就要靠 **TF2**。
TF 就是一张**坐标系关系图**：每个节点广播"我（子坐标系）相对父坐标系在哪/朝向哪"，任何节点都能查询任意两个坐标系之间的变换。
### 2. 核心概念
| 概念 | 含义 |
|---|---|
| `frame_id` | 坐标系名字（如 `base_link`） |
| `TransformStamped` | 一条变换：父/子 frame、平移、旋转（四元数） |
| 静态变换 | 不动的（如云台装在底盘上方 0.2 m） |
| 动态变换 | 随时变的（如云台 pitch 转动） |
### 3. 广播 tf_broadcaster.py
对应代码 `ros2_ws/src/rm_tf_demo/rm_tf_demo/tf_broadcaster.py`。
#### 静态变换（底盘 → 云台）
```python
from tf2_ros import StaticTransformBroadcaster

self.static_br = StaticTransformBroadcaster(self)
static_tf = TransformStamped()
static_tf.header.frame_id = 'base_link'      # 父
static_tf.child_frame_id = 'gimbal_link'     # 子
static_tf.transform.translation.z = 0.2      # 云台在底盘上方 0.2 m
static_tf.transform.rotation.w = 1.0         # 无旋转（四元数）
self.static_br.sendTransform(static_tf)
```
#### 动态变换（云台 → 相机，pitch 摆动）
```python
from tf2_ros import TransformBroadcaster

self.dynamic_br = TransformBroadcaster(self)
self.timer = self.create_timer(0.1, self.broadcast_camera)

def broadcast_camera(self):
    pitch = 0.6 * math.sin(self.get_clock().now().nanoseconds * 1e-9)
    tf = TransformStamped()
    tf.header.stamp = self.get_clock().now().to_msg()   # 时间戳！动态变换必须带
    tf.header.frame_id = 'gimbal_link'
    tf.child_frame_id = 'camera_link'
    tf.transform.translation.x = 0.1                    # 相机在云台前方 0.1 m
    # 绕 Y 轴转 pitch（四元数）
    tf.transform.rotation.y = math.sin(pitch / 2.0)
    tf.transform.rotation.w = math.cos(pitch / 2.0)
    self.dynamic_br.sendTransform(tf)
```
### 4. 监听 tf_listener.py
```python
from tf2_ros import Buffer, TransformListener

self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)
self.timer = self.create_timer(0.5, self.lookup)

def lookup(self):
    tf = self.tf_buffer.lookup_transform(
        'base_link', 'camera_link', rclpy.time.Time())   # 查相机在底盘下的位姿
    t = tf.transform.translation
    self.get_logger().info(f'camera 相对 base: x={t.x:.2f} z={t.z:.2f}')
```
- `lookup_transform(target, source, time)`：查 `source` 相对 `target` 的变换。
- `rclpy.time.Time()` 表示"最新可用"。
- 链上任何一段缺失/时间戳不对都会抛异常，所以要 `try/except`。
### 5. 运行验证
```bash
cd ros2_ws && source install/setup.bash

# 终端 A：广播（先跑，否则查不到）
ros2 run rm_tf_demo tf_broadcaster

# 终端 B：监听
ros2 run rm_tf_demo tf_listener
# 输出：camera 相对 base: x=0.10 z=0.20
```
### 6. 命令行调试工具
```bash
ros2 run tf2_ros tf2_echo base_link camera_link   # 实时打印某条变换
ros2 run tf2_ros tf2_monitor                      # 打印所有帧和频率
ros2 run tf2_tools view_frames                    # 生成 frames.pdf 图
```
### 7. 本节要点
1. TF = 坐标系关系图；每个节点广播自己到父系的变换。
2. 静态用 `StaticTransformBroadcaster`，动态用 `TransformBroadcaster`（**必须带时间戳**）。
3. 查询用 `Buffer.lookup_transform`，要 `try/except`。
4. RM 里典型链：`odom → base_link → gimbal_link → camera_link`。
## 08 串口对接电控板
### 1. 为什么需要串口桥接
ROS2 跑在**上位机**（迷你主机 / Jetson / 妙算），而电机、裁判系统、IMU 在 **STM32 电控板**上，两者通过串口（UART/USB 转串口）通信。
上位机的 ROS2 节点不能直接读串口，需要一个**桥接节点**：把串口数据翻译成话题，把话题数据翻译成串口帧。
```text
┌──────────────┐   /serial/tx   ┌─────────────┐   UART   ┌───────────┐
│ 决策/控制节点  │ ◀────────────▶ │ 串口桥接节点  │ ◀──────▶ │ STM32 板   │
└──────────────┘   /serial/rx   └─────────────┘          └───────────┘
```
这套逻辑和队里的电控框架（SYSU Hero 的云台/底盘板，需向电控组另行获取，本仓库不包含）正好衔接：STM32 侧已经跑着电机控制和裁判系统解析，上位机侧只需要把数据桥接到 ROS2。
### 2. 依赖
在第 1 章创建的 conda 环境里安装 Python 串口库（已按第 1 章装过可跳过）：
```bash
conda activate rm_ros2
python -m pip install pyserial
```
不要加 `--user`：本教程已用 `PYTHONNOUSERSITE=1` 禁用用户目录里的包。
### 3. 桥接节点骨架
下面是一个**可运行的最小骨架**（协议格式要按你们实际帧协议改）。它做两件事：
1. 后台线程持续读串口，收到一帧就发到 `/serial/rx`；
2. 订阅 `/serial/tx`，把消息写进串口。
```python
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        # 参数：串口设备、波特率
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        self.ser = serial.Serial(port, baud, timeout=0.1)

        # 话题：rx = 收自电控板，tx = 发往电控板
        self.rx_pub = self.create_publisher(String, 'serial/rx', 10)
        self.create_subscription(String, 'serial/tx', self.on_tx, 10)

        # 后台读串口线程
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f'串口已打开: {port} @ {baud}')

    def read_loop(self):
        """持续读串口，按行/帧切分后发布。"""
        buf = b''
        while rclpy.ok():
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception as e:
                self.get_logger().error(f'读串口失败: {e}')
                break
            if not data:
                continue
            buf += data
            # 按换行符切帧（协议不同请改这里）
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                msg = String()
                msg.data = line.decode('utf-8', errors='ignore')
                self.rx_pub.publish(msg)

    def on_tx(self, msg):
        """收到要发送的消息，写进串口。"""
        try:
            self.ser.write((msg.data + '\n').encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'写串口失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
> 注意：`read_loop` 在**独立线程**里跑，因为串口 `read` 会阻塞，不能放在回调里阻塞 spin。这是桥接节点的标准写法。
### 4. 运行与调试
本章没有预建的串口包。需先自行新建 `ament_python` 包（下方的 `<pkg>`），将骨架保存为 `serial_bridge.py`，参考第 3 章在 `setup.py` 中注册 `serial_bridge = <pkg>.serial_bridge:main`，声明 `rclpy` / `std_msgs` 依赖，构建并 source 工作区后才能运行：
```bash
ros2 run <pkg> serial_bridge --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
```
调试时没有真机，可以用 `socat` 造一对虚拟串口，或者直接 `ros2 topic pub /serial/tx ...` 看发送、`ros2 topic echo /serial/rx` 看接收。
### 5. 实际要做的三件事
骨架只是框架，落地时需要：
1. **定协议**：帧头 + 校验 + 字段（参考电控组现有的串口协议）。
2. **换成自定义消息**：把 `String` 换成第 4 章那种 `ChassisCmd` / `GimbalTarget` 结构，别传裸字符串。
3. **拆帧/打包**：写 `parse_frame()` 和 `build_frame()`，把字节流和 ROS 消息互转。
### 6. 本节要点
1. 桥接节点 = 串口线程 + 话题收发，串口读写放后台线程。
2. 用 `pyserial`，参数化 `port`/`baudrate`。
3. 别传裸字符串，用自定义消息。
## 09 实战：MuJoCo 接入 ROS2
把前面学的串起来，做一件真事：**把队里 [Wheel_Leg](Wheel_Leg/README.md) 的 MuJoCo 轮足仿真，封装成一个 ROS2 节点**，对外发布 IMU / 关节状态，并接收速度指令。
对应代码：`ros2_ws/src/wheel_leg_bridge/`（已在本机验证可跑）。
### 1. 目标架构
原来的仿真是一个大 while 循环（`robot_control_main.py`），传感器和控制全揉在一起。我们要把它拆成：
```text
┌──────────────────────────┐
│  wheel_leg_bridge 节点    │
│                          │
│  MuJoCo 模型 + 仿真循环    │──发布──▶ /imu (sensor_msgs/Imu)
│                          │──发布──▶ /joint_states (sensor_msgs/JointState)
│                          │◀─订阅── /cmd_vel (geometry_msgs/Twist)
└──────────────────────────┘
```
- **发出去**：别的节点（决策/可视化）能订阅机器人状态。
- **收进来**：别的节点能发 `cmd_vel` 控制轮子。
这就是"把非 ROS 程序接入 ROS2"的标准套路：**在仿真循环外裹一层 ROS2 的话题/订阅**。
### 2. 数据从哪来
MuJoCo 模型 [robot.xml](Wheel_Leg/LQR_VMC/mjcf/robot.xml) 里已经定义好了传感器，原代码 [robot_info_update.py](Wheel_Leg/LQR_VMC/robot_mujoco_sensor_feedback/robot_info_update.py) 负责读它们。我们做的，是把这些原始数据**映射成标准 ROS 消息**：
| MuJoCo 传感器 | 内容 | ROS 消息 |
|---|---|---|
| `orientation` (framequat) | 姿态四元数 `[w,x,y,z]` | `Imu.orientation`（ROS 要 `[x,y,z,w]`，注意换序） |
| `angular_vel` (gyro) | 角速度 `[x,y,z]` | `Imu.angular_velocity` |
| `right1_pos/vel` … `leftwheel_pos/vel` | 6 个关节位置/速度 | `JointState.position/velocity` |
### 3. 节点代码逐段看
完整代码在 `wheel_leg_bridge/wheel_leg_bridge.py`。
#### 加载模型 + 声明参数
```python
import mujoco
from ament_index_python.packages import get_package_share_directory

self.declare_parameter('robot_xml', '')
xml = self.get_parameter('robot_xml').value
if not xml:
    xml = os.path.join(
        get_package_share_directory('wheel_leg_bridge'),   # 找包安装目录
        'robots', 'robot.xml')                             # 模型文件

self.model = mujoco.MjModel.from_xml_path(xml)
self.data = mujoco.MjData(self.model)
self.dt = self.model.opt.timestep    # 0.002 s
```
- 模型文件 `robot.xml`/`obstacles.xml` 被复制进包并装到 `share/`，用 `get_package_share_directory` 定位，**不写死绝对路径**。
- 也可以 `--ros-args -p robot_xml:=/自定义路径/robot.xml` 覆盖。
#### 发布者 + 订阅者
```python
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist

self.imu_pub = self.create_publisher(Imu, 'imu', 10)
self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
```
#### 控制循环（定时器 + 步进仿真）
```python
self.timer = self.create_timer(0.02, self.step)   # 50 Hz

def step(self):
    # 执行器顺序：0-3 腿关节，4=右轮，5=左轮
    self.data.ctrl[0:4] = 0.0
    self.data.ctrl[4] = self.right_torque
    self.data.ctrl[5] = self.left_torque

    for _ in range(10):                            # 10 × 0.002 = 0.02 s
        mujoco.mj_step(self.model, self.data)

    self.publish_imu()
    self.publish_joints()
```
- MuJoCo 的 `dt=0.002`（500 Hz），ROS 定时器跑 50 Hz，所以每次回调**步进 10 次**。
- 订阅回调里只存目标值，真正下发在 `step()` 里做，保持节奏稳定。
#### cmd_vel → 轮力矩（差速）
```python
def cmd_vel_cb(self, msg):
    k = 1.0
    self.left_torque  = k * (msg.linear.x - msg.angular.z * self.track / 2.0)
    self.right_torque = -k * (msg.linear.x + msg.angular.z * self.track / 2.0)
```
`linear.x` 驱动前进，`angular.z` 做差动转弯。模型左右轮转轴方向相反，所以右轮力矩要取负：纯前进时左右执行器力矩异号，但两轮向同一方向滚动。
#### 发布 IMU（注意四元数换序）
```python
def publish_imu(self):
    msg = Imu()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'imu'

    w, x, y, z = self.data.sensor('orientation').data   # MuJoCo: [w,x,y,z]
    msg.orientation.x = float(x)
    msg.orientation.y = float(y)
    msg.orientation.z = float(z)
    msg.orientation.w = float(w)                        # ROS: [x,y,z,w]

    gx, gy, gz = self.data.sensor('angular_vel').data
    msg.angular_velocity.x = float(gx)
    msg.angular_velocity.y = float(gy)
    msg.angular_velocity.z = float(gz)
    self.imu_pub.publish(msg)
```
### 4. 运行验证
依赖（MuJoCo）装在 conda 环境 `rm_ros2` 里，**构建和运行都要先激活它**（创建方法和完整依赖清单见[第 1 章](#01-环境准备与工作区)第 4 节）：
```bash
conda activate rm_ros2
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# 终端 A：启动桥接节点
ros2 run wheel_leg_bridge wheel_leg_bridge

# 终端 B：看话题
ros2 topic list | grep -E 'imu|joint_states|cmd_vel'
ros2 topic echo /imu --once          # 看姿态四元数
ros2 topic echo /joint_states --once # 看 6 个关节

# 终端 C：发速度指令（机器人轮子会转）
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```
> 说明：`wheel_leg_bridge` 只做了"数据通路"，没有接 LQR/VMC 平衡控制器，所以车体会倒——这是**故意的**。平衡控制正是下一节大作业要你做的事。
### 5. 大作业：把控制解算搬到 ROS2 里
#### 5.1 题目
第 9 章前面的 `wheel_leg_bridge` 是"仿真 + 控制揉在一个节点里"——它自己把 `cmd_vel` 换算成轮力矩。本次作业要彻底拆开：
> **MuJoCo 只当仿真器和执行器出口（节点已提供，不许改）；LQR / VMC 等全部解算由你写在独立的控制器节点里；两者只用 ROS2 话题通信。**
#### 5.2 架构契约
```text
┌──────────────────────────┐   /imu  /joint_states   ┌─────────────────────────┐
│  仿真节点 wheel_leg_sim   │ ──────────────────────▶ │   控制器节点（你写）      │
│  （已提供，学生不改）      │                         │  · 四元数 → 欧拉角       │
│  · mj_step，固定 dt=0.002 │                         │  · VMC 五连杆正运动学    │
│  · 读传感器 → 发话题       │     /actuator_cmd       │  · LQR 状态更新 + K 矩阵  │
│  · 订阅指令 → data.ctrl   │ ◀────────────────────── │  · VMC 力矩映射          │
│  · 不含任何控制律          │                         │  → 6 个执行器力矩        │
└──────────────────────────┘                         └─────────────────────────┘
```
仿真节点就是 `src/wheel_leg_sim/`，已在本机验证可跑。
#### 5.3 接口（必须严格照此实现）
| 方向 | 话题 | 类型 | 内容 |
|---|---|---|---|
| 仿真→控制器 | `/imu` | `sensor_msgs/msg/Imu` | `orientation` 机体姿态四元数 (x,y,z,w)；`angular_velocity` 机体角速度 (rad/s) |
| 仿真→控制器 | `/joint_states` | `sensor_msgs/msg/JointState` | name 依次是 `right1,right2,left1,left2,rightwheel,leftwheel`；`position`(rad)、`velocity`(rad/s) |
| 控制器→仿真 | `/actuator_cmd` | `rm_msgs/msg/WheelLegCmd` | 6 个关节力矩 (N·m) |
| 外部→控制器 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 目标速度（阶段 2/5 用），映射到 LQR 的 `v_set` |
`WheelLegCmd` 的字段顺序与 [robot.xml](Wheel_Leg/LQR_VMC/mjcf/robot.xml) 里 `<actuator>` 的定义一致，正好对应 MuJoCo 的 `data.ctrl[0..5]`：
```text
float64 right_phi1    # 右腿近端  joint_right_phi1   -> ctrl[0]  (限幅 ±6.0)
float64 right_phi4    # 右腿远端  joint_right_phi4   -> ctrl[1]  (限幅 ±6.0)
float64 left_phi1     # 左腿近端  joint_left_phi1    -> ctrl[2]  (限幅 ±6.0)
float64 left_phi4     # 左腿远端  joint_left_phi4    -> ctrl[3]  (限幅 ±6.0)
float64 right_wheel   # 右轮      right_wheel_torque -> ctrl[4]  (限幅 ±2.0)
float64 left_wheel    # 左轮      left_wheel_torque  -> ctrl[5]  (限幅 ±2.0)
```
#### 5.4 硬性约束（本次的考点）
1. **控制器节点里不得出现 `mujoco`**；**仿真节点里不得出现任何控制律**（PID / LQR / VMC 一律不许）。
2. 两个节点必须能**各自独立 `ros2 run`** 启动，禁止进程内共享变量或全局状态。
3. **允许并鼓励**复用 [Wheel_Leg/LQR_VMC/](Wheel_Leg/LQR_VMC) 下的 `vmc_cal.py`、`lqr_update.py`、`roll_leg_control.py`、`ground_detection.py`——把这几个文件复制进你的包，或封装成一个库包再 import。
4. **不允许**把 [robot_control_main.py](Wheel_Leg/LQR_VMC/robot_control_main.py) 的主循环整段搬过来——那样等于没拆分。
> 换句话说：**这次考的不是控制算法，而是"把控制回路拆成两个 ROS2 节点"这件事。** 算法直接抄现成的没问题，接口和架构必须自己搭对。
#### 5.5 怎么跑
**先自己创建控制器包（只执行一次）**。先完成第 4 节的构建，然后在同一个 `ros2_ws/` 工作区执行；如果已经创建过这个包，不要重复创建：
```bash
conda activate rm_ros2
source /opt/ros/humble/setup.bash
# 当前目录应是 Tutorial/ros2_ws/，执行 ls 能看到 src/
ros2 pkg create wheel_leg_controller \
  --build-type ament_python \
  --node-name wheel_leg_controller \
  --destination-directory src \
  --dependencies rclpy sensor_msgs geometry_msgs rm_msgs
colcon build --symlink-install --packages-up-to wheel_leg_controller
source install/setup.bash
```
生成的入口文件是 `src/wheel_leg_controller/wheel_leg_controller/wheel_leg_controller.py`，`setup.py` 会自动注册同名可执行命令。**生成的程序只打印 `Hi from wheel_leg_controller.` 后退出，还不是常驻 ROS 节点**。先用下面的命令确认入口能找到，再参考第 3 章把它改成继承 `Node`、使用 `rclpy.spin()` 的节点，并实现第 5.3 节的话题接口和控制回调。
复用算法时，把文件放到内层 `wheel_leg_controller/` 中，与入口文件同级，并使用相对导入，例如 `from .vmc_cal import five_links_param`。不要导入原工程的 MuJoCo 传感器读取或整个主循环。
启动时开两个终端，**每个终端都先执行** `conda activate rm_ros2`、`source /opt/ros/humble/setup.bash`，进入同一个工作区并 `source install/setup.bash`，再分别运行：
```bash
# 终端 A：仿真节点
ros2 run wheel_leg_sim wheel_leg_sim
```
```bash
# 终端 B：你的控制器
ros2 run wheel_leg_controller wheel_leg_controller
```
必做任务只需仿真，不需要实体机器人、电控板或手柄。**仿真节点默认会打开 MuJoCo 查看器窗口**；收到控制器的有效指令后才开始推进，在控制器尚未实现发布力矩时模型不动是正常现象。镜头默认跟踪机体，常用参数：
| 参数 | 默认 | 说明 |
|---|---|---|
| `viewer` | true | 设 `false` 关闭 GUI；Linux 未设置 `DISPLAY` / `WAYLAND_DISPLAY` 时自动跳过查看器 |
| `cam_distance` | 1.2 | 镜头到机体的距离 (m) |
| `cam_azimuth` / `cam_elevation` | 120 / -12 | 方位角、俯仰角 (度) |
| `viewer_fps` | 60 | 渲染帧率 |
```bash
# 无界面运行（远程 SSH、或配合自动化脚本）
ros2 run wheel_leg_sim wheel_leg_sim --ros-args -p viewer:=false

# 想换个角度看
ros2 run wheel_leg_sim wheel_leg_sim --ros-args -p cam_distance:=2.0 -p cam_elevation:=-25
```
若显示变量已设置但显示服务不可用（如失效的 SSH 显示转发），请显式设置 `viewer:=false`；GLFW 的后台初始化错误不一定能被 Python 异常捕获。
> 开着 GUI 时仿真是**慢动作**运行（渲染要占时间），属正常现象——控制周期仍是"一条指令 = 一个 dt"，与墙钟速度无关，所以平衡结果不受影响。
![机器人保持平衡时的 MuJoCo 查看器](Tutorial/ROS2_入门/images/wheel_leg_balance_gui.png)
检查数据流是否通：
```bash
ros2 topic hz /joint_states      # 有稳定频率 → 仿真在步进
ros2 topic hz /actuator_cmd      # 有频率     → 你的控制器在发指令
```
> ⚠️ **别和 `wheel_leg_bridge` 同时跑。** 第 9 章教学版的 `wheel_leg_bridge` 用的是**同名话题**
> （`/imu`、`/joint_states`）。一个话题上挂两个发布者时，你的控制器会收到**两个仿真器交错的状态**，
> 必然失稳——而且看起来像是"我代码没错但它就是摔"。
> 所以动手前先确认只有它一个在跑：
> ```bash
> ros2 node list        # 应只看到 /wheel_leg_sim（外加你自己的控制器）
> ```
> 如果确实需要两个一起跑，给其中一个加命名空间隔离：
> `ros2 run wheel_leg_sim wheel_leg_sim --ros-args -r __ns:=/sim`（此时话题变成 `/sim/imu` 等）。
#### 5.6 三个必须知道的工程细节
**① 仿真节点是"指令驱动"的：一条指令 = 一个 dt。**
`wheel_leg_sim` 不按墙钟自己步进，而是**每收到一条 `/actuator_cmd` 才步进一个 `dt`（0.002 s）**，然后回发一次状态。这样你控制器里的 `dt` 恒等于 0.002，与 `lqr_update.py` 里那套 K 矩阵（按 dt=0.002 离散整定）严格一致。
> 代价是仿真推进速度取决于你的发布速度（可能快于或慢于实时）。这不影响正确性，你也不需要去"调仿真频率"。
**② 不用怕启动顺序：仿真节点有"心跳"。**
超过 0.5 s 没收到指令，仿真节点会**补发一次当前状态**，所以你的控制器晚启动、中途重启都能自动接上，不会双方互相空等。
**③ 传输延迟真实存在，但实测扛得住。**
拆成两个进程后，每个控制周期比原来内联循环多 0.2–2 ms 的 DDS 传输延迟。作业的参考解在本机实测：**10 s 站桩，后半程 pitch 稳定在 0.24°**。所以只要按接口实现，平衡是能达到的。
#### 5.7 分阶段任务与验收
建议按顺序做，每阶段都能独立验收：
| 阶段 | 任务 | 验收标准 |
|---|---|---|
| 1 打通链路 | 控制器先只发固定值/正弦，把三条话题跑起来 | `/joint_states`、`/actuator_cmd` 都有稳定频率，轮子有响应 |
| 2 轮速控制 | 只用 `/joint_states` 的轮速做 PI，让它按 `/cmd_vel` 前后走（先不管平衡） | 发 `linear.x=0.5` 时轮速跟随，误差 < 20% |
| 3 VMC 接入 | 引入 `vmc_cal.py`，用腿关节角算 `L_0`/`theta`，输出腿力矩 + 腿长 PID | `L_0` 稳定在目标值 ±5 mm |
| 4 **LQR 平衡（必做）** | 引入 `lqr_update.py`，用 IMU 的 pitch/gyro + 轮里程计做状态更新，输出轮力矩 | **原地平衡 ≥ 5 s**，机体 pitch 偏差 < 10° |
| 5 加分项 | 平衡状态下跟踪 `/cmd_vel`；加速度限制；抗扰（推一把）；**接手柄遥控**（见 5.10） | 以 0.3 m/s 前进 3 m 不摔 |
#### 5.8 提交物
1. 一个新包 `src/wheel_leg_controller/`（`ament_python`），`colcon build` 通过；
2. 该包的 `README.md`：说明你的控制结构、K/参数来源，以及**实测的平衡时长与 pitch 曲线**；
3. 一段 10–20 s 录屏，或 `ros2 bag record /imu /joint_states /actuator_cmd` 的 bag 文件。
把录屏 / bag 的可访问链接写进控制包的 `README.md`，不要把大型录像或 bag 直接塞进 Git。确认 `origin` 是**自己的 Fork**，再在仓库根目录提交：
```bash
cd "$(git rev-parse --show-toplevel)"
git remote -v
git status
git add 02-ros2-wheelleg/Tutorial/ros2_ws/src/wheel_leg_controller
git commit -m "Complete ROS2 wheel-leg controller assignment"
git push origin main
```
提交自己的仓库链接（如 `https://github.com/USERNAME/training`），并注明控制包位置和录屏 / bag 链接。不要直接推送队伍公共仓库，也不要修改冻结的 `wheel_leg_sim`。
#### 5.9 参考资料
| 文件 | 内容 |
|---|---|
| [robot_control_main.py](Wheel_Leg/LQR_VMC/robot_control_main.py) | 原版单进程控制主循环（移植蓝本） |
| [robot_info_update.py](Wheel_Leg/LQR_VMC/robot_mujoco_sensor_feedback/robot_info_update.py) | 四元数→欧拉角、6 关节读取的算法 |
| `Wheel_Leg/LQR_VMC/vmc_cal.py` | 五连杆正运动学 + VMC 力矩映射 |
| `Wheel_Leg/LQR_VMC/lqr_update.py` | LQR 状态更新 + K 矩阵多项式 |
> 一个容易踩的移植细节：`vmc_cal.py` 里 `last_L_0`/`last_phi_0`/`last_alpha` 是在赋值的**下一行**就被更新的，所以 `d_L_0`、`d_phi_0`、`d_alpha` **恒为 0**。这是原版的既有行为，照原样移植即可——"顺手修好"反而可能不稳。
> **另一个更隐蔽、后果更严重的坑：`x_b_set` 不能每步都写。** 原版是这么写的：
> ```python
> if not (current_v == 0.0 and teleop.target_v == 0):   # 站桩时【不更新】
>     lqr_left.x_b_set  = (common_motor_pos * wheel_radius)
>     lqr_right.x_b_set = (common_motor_pos * wheel_radius)
> ```
> 也就是说**只在运动时**让 `x_b_set` 跟着当前位移走；站桩时冻结它。这样 `x_b - x_b_set` 才表示"停下来之后漂了多远"，`K[4]`/`K[5]` 两个位置项才能构成位置环。
> 如果你图省事写成每步都 `x_b_set = 当前位移`，那么 `x_b - x_b_set ≡ 0`，**位置环直接失效**。现象很有迷惑性：车能稳稳站住、pitch 也正常，但会以**恒定速度一直往前滑走**，完全不收敛——看着就像"一辆会站立的小车"，而不是轮腿平衡控制器。
> 实测对比（站桩 13 s）：出错时位置从 −83 rad 线性涨到 −223 rad（**滑走 8.4 m**，轮速恒定 1.75 rad/s）；修正后位置**锁死在 −1.46 rad ± 0.01**。
#### 5.10 可选：接手柄遥控
如果想让机器人能手柄遥控（阶段 5 的加分项），**推荐走 ROS2 标准的 `/joy` 话题**，而不是在控制器里直接读硬件（`pygame` / `evdev`）：
```text
/dev/input/js0 ──▶ joy_node ──/joy (sensor_msgs/Joy)──▶ 你的控制器 ──▶ 摇杆映射成 v/w
```
- `joy_node`（`ros-humble-joy`）负责读设备并发布 `/joy`；
- 你的控制器只订阅 `/joy`，把摇杆映射成 LQR 的 `v_set`、按键映射成目标腿长；
- 好处：**硬件与算法解耦**、换手柄只改参数、**没有实体手柄也能用合成消息测试**。
先确认设备与数据是否正常：
```bash
sudo apt install -y ros-humble-joy     # 如果还没装
ros2 run joy joy_enumerate_devices     # 列出设备（应看到你的手柄）
ros2 run joy joy_node                  # 另一个终端
ros2 topic echo /joy --once            # 看 axes / buttons 布局
```
参考解沿用了队里 `remote/xbox_controller.py` 的习惯（Xbox 360 布局）：
| 操作 | 轴/键 | 效果 |
|---|---|---|
| 左摇杆 上下 | `axes[1]` | 前进/后退，`v = -axis × 0.6` m/s |
| 右摇杆 左右 | `axes[3]` | 左转/右转，`w = -axis × 2.0` rad/s |
| RB / LB | `buttons[5]` / `[4]` | 目标腿长 ±0.005 m（限位 0.10–0.14） |
死区取 **0.1**；按键做**边沿触发**（按一次改一次，长按不狂涨）。
> ⚠️ **两个容易翻车的地方**
> 1. **必须给线速度加爬坡限幅**。摇杆从 0 猛推到满杆，目标速度会瞬间跳 0.6 m/s，倒立摆上一下就顶翻。用 `util/climb_traj.py` 的 `VelocityRampFilter`（原版就是 0.8 / 1.4 m/s²）。
> 2. **手柄和 `/cmd_vel` 要有优先级约定**。建议手柄优先，并在超过 `joy_timeout`（如 0.5 s）收不到 `/joy` 时自动退回 `/cmd_vel`——这样上课能用手柄、自动化测试还能用话题。
### 6. 本节要点（串起全部章节）
| 用到的知识 | 对应章节 |
|---|---|
| conda 环境与依赖管理 | 01 |
| 建包、colcon、entry_points | 01、03 |
| 发布者/订阅者 | 03 |
| 标准消息 `sensor_msgs` / `geometry_msgs` | 03 |
| 自定义消息 `WheelLegCmd` | 04 |
| 参数 `robot_xml` | 05 |
| 定时器 / 回调驱动的控制循环 | 03、09 |
| 把非 ROS 程序裹成节点 | 09 |
| **把控制解算拆进独立节点（大作业）** | 09 |
到这里，你已经走完"从零 → 概念 → 工具 → 自写节点 → 消息/服务/TF → 串口 → 实战接入仿真"的完整链路。剩下的就是大作业和真实项目了。
## 附录：VS Code 插件
在扩展市场搜索安装：
| 插件 ID | 用途 |
|---|---|
| `ms-iot.vscode-ros` | ROS 开发核心扩展（构建/调试/启动） |
| `ms-python.python` | Python 支持 |
| `ms-vscode.cmake-tools` | CMake（接口包、看包结构） |
| `ms-vscode.cpptools` | C++（看 ROS2 头文件/源码） |
| `redhat.vscode-yaml` | launch/参数 YAML 高亮 |
| `redhat.vscode-xml` | URDF/XACRO 高亮 |
| `smilerobotics.urdf` | URDF 模型预览 |
| `yzhang.markdown-all-in-one` | 写文档/教程 |
