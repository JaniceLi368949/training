# 06 launch 文件

前面都是一个一个 `ros2 run` 启动节点。真实系统里动辄十几个节点，手动起不现实。**launch 文件**让你一次启动多个节点、顺便传参。

对应代码：各包的 `launch/*.launch.py`。

## 1. 最简单的 launch

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

## 2. 运行

```bash
cd ros2_ws && source install/setup.bash
ros2 launch my_first_pkg talker_listener.launch.py
```

一个命令同时起两个节点，`Ctrl+C` 一次全停。

## 3. 带参数的 launch

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

## 4. launch 文件要装进包里

`setup.py` 里要声明 launch 目录，否则 `ros2 launch` 找不到：

```python
import os
from glob import glob

data_files=[
    ...
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
],
```

## 5. 更高级（了解即可）

- **DeclareLaunchArgument**：在命令行传入启动参数（`ros2 launch ... param:=value`）。
- **IncludeLaunchDescription**：一个 launch 里嵌套另一个 launch。
- **条件/延时**：`IfCondition`、`TimerAction` 等。

初学先把 `Node` + `parameters` + `arguments` 用熟，其余用到再查。

## 6. 本节要点

1. launch = 一次启动多节点；`Node(package=, executable=)` 是基本单位。
2. `parameters` 传参、`arguments` 传 argv。
3. launch 文件要写进 `setup.py` 的 `data_files`。
4. 调试技巧：`ros2 launch` 出问题先单独 `ros2 run` 每个节点定位。

---

下一章：[07 TF2 坐标变换](07_TF2坐标变换.md)
