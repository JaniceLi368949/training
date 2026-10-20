# ROS2 入门教程（RoboMaster 算法组）

面向**会用 Linux、但没接触过 ROS** 的新队员。以 **Python** 为主线，从零搭工作区，最后把队里现有的 [Wheel_Leg](../../Wheel_Leg/README.md) MuJoCo 仿真接入 ROS2。

**合订版：** [ROS2 入门完整教程](../../ROS2_入门_完整教程.md)——仓库结构说明、00–09 全部 10 篇正文和大作业，可用于连续阅读或导入飞书。

## 学习路线

| 章节 | 内容 | 对应代码 |
|---|---|---|
| [00 为什么用 ROS2](00_为什么用ROS2.md) | 概念：节点/话题/服务/TF，为什么要模块化 | — |
| [01 环境准备与工作区](01_环境准备与工作区.md) | 装 ROS2、colcon、工作区结构、常见坑 | `ros2_ws` |
| [02 turtlesim 初体验](02_turtlesim初体验.md) | 用官方 demo 建立心智模型，rqt 可视化 | — |
| [03 第一个 Python 节点](03_第一个Python节点.md) | publisher / subscriber | `my_first_pkg` |
| [04 自定义消息](04_自定义消息.md) | 定义 RM 接口（msg/srv） | `rm_msgs` |
| [05 服务与参数](05_服务与参数.md) | request/response、参数 | `rm_gimbal_demo` |
| [06 launch 文件](06_launch文件.md) | 一次启动多个节点、传参 | 各包 `launch/` |
| [07 TF2 坐标变换](07_TF2坐标变换.md) | 云台↔相机↔底盘坐标系链 | `rm_tf_demo` |
| [08 串口对接电控板](08_串口对接电控板.md) | 上位机 ROS2 ↔ STM32 | 概念 + 骨架 |
| [09 实战：MuJoCo 接入 ROS2](09_实战_MuJoCo接入ROS2.md) | 把轮足仿真封装成节点 | `wheel_leg_bridge` |

## 环境要求

- Ubuntu 22.04 + **ROS 2 Humble**（安装见[第 1 章](01_环境准备与工作区.md)第 1 节）
- **conda 环境 `rm_ros2`（Python 3.10）**，装 MuJoCo 等第三方依赖——第 8、9 章需要，创建方法和完整依赖清单见[第 1 章](01_环境准备与工作区.md)第 4 节
- 编辑器：VS Code（推荐装下面这些插件）
- **必做作业不需要硬件**：第 08 章实体串口可先跳过；第 09 章在 MuJoCo 中完成，手柄只是加分项。没有图形桌面时可用 `viewer:=false` 运行仿真。

### VS Code 插件清单

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

## 示例工作区

本教程所有代码都在 [ros2_ws](../ros2_ws/) 里，结构：

```text
ros2_ws/
└── src/
    ├── my_first_pkg/        # 第3章：talker / listener
    ├── rm_msgs/             # 第4章：自定义消息接口（ament_cmake）
    ├── rm_gimbal_demo/      # 第5、6章：服务 + 参数 + launch
    ├── rm_tf_demo/          # 第7章：TF2 广播/监听
    ├── wheel_leg_bridge/    # 第9章：MuJoCo ↔ ROS2 桥接（教学版）
    └── wheel_leg_sim/       # 第9章作业：纯仿真节点（不含任何控制律）
```

> 代码已在本机 `colcon build --symlink-install` 验证可编译、可运行（构建环境为 conda env `rm_ros2`）。照着章节做就能跑通。

## 环境踩坑速查（本机已修复，重装时注意）

1. **Python 环境**：第三方依赖（MuJoCo 等）统一装在 conda 环境 `rm_ros2`（Python 3.10）里，**并且要在该环境里构建工作区**——否则 `ros2 run` 会因 shebang 指向系统 python 而报 `No module named 'mujoco'`。详见[第 1 章](01_环境准备与工作区.md)第 4 节。
2. **setuptools 必须 `<72`**：72 版移除了 colcon 依赖的 `setup.py develop`，会报 `option --uninstall/--editable not recognized`。
3. **empy 必须 `<4`**：`rosidl_adapter` 用了 `em.BUFFERED_OPT`，4.x 已删除。
4. **lark 要单独装**：`rosidl_parser` 依赖它，系统上由 apt 提供，隔离环境里没有。
5. **设 `PYTHONNOUSERSITE=1`**：CPython 把 `~/.local` 排在 conda 环境之前，不关掉会遮蔽环境里的包。
6. **接口包用 ament_cmake**：定义 `.msg/.srv` 的包（`rm_msgs`）要用 `ament_cmake`，即使业务代码全是 Python。
7. **别用 `handle` 当方法名**：`Node` 基类自带 `handle` 属性，子类定义同名方法会遮蔽它，导致 `AttributeError: __enter__`。

## 快速验证

```bash
conda activate rm_ros2
source /opt/ros/humble/setup.bash
# 路径以第一次作业的克隆位置为例，请按实际位置调整
cd ~/venom_cpp_basics/training/02-ros2-wheelleg/Tutorial/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run my_first_pkg talker        # 终端 A：每 0.5 秒发布消息
```

另开终端 B，不能省略环境加载：

```bash
conda activate rm_ros2
source /opt/ros/humble/setup.bash
cd ~/venom_cpp_basics/training/02-ros2-wheelleg/Tutorial/ros2_ws
source install/setup.bash
ros2 run my_first_pkg listener      # 收到消息即成功
```
