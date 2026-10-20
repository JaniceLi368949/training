# 节点回归测试

在已安装 `mujoco` 的 Python 环境中，从仓库根目录运行：

```bash
python -m unittest discover -s 02-ros2-wheelleg/Tutorial/tests -v
```

测试使用真实 MuJoCo 模型和物理步进，以轻量替身代替 ROS API，因此不需要安装 ROS2、构建工作区或打开 GUI。

覆盖：

- 无界面启动及心跳发布有效姿态，且不推进仿真时间；
- 一条执行器指令仍恰好推进一个 `dt`，6 路力矩顺序不变；
- Linux 没有显示环境时跳过查看器，有显示环境时保留查看器启动路径；
- 左右轮镜像安装下的前进、后退和转向力矩符号；
- 相机 pitch 四元数绕 Y 轴而不是 X 轴旋转。

这些是节点逻辑测试，**不替代** Ubuntu 22.04 / ROS2 Humble 下的 `colcon build`、真实 GUI 启动、DDS 通信及双节点闭环验收。
