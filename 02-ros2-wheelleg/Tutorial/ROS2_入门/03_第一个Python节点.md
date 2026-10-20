# 03 第一个 Python 节点

从这一章开始写代码。目标：写一个**发布者**（每 0.5 秒发一条消息）和一个**订阅者**（收到就打印），看它们互相通信。

对应代码：`ros2_ws/src/my_first_pkg/`。

## 1. 包的结构

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

### package.xml（关键字段）

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

### setup.py（决定哪些是可执行节点）

```python
entry_points={
    'console_scripts': [
        'talker = my_first_pkg.talker:main',     # 命令名 = 模块:入口函数
        'listener = my_first_pkg.listener:main',
    ],
},
```

这一行把 `ros2 run my_first_pkg talker` 映射到 `my_first_pkg/talker.py` 里的 `main()`。

## 2. 发布者 talker.py

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

## 3. 订阅者 listener.py

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

## 4. 构建并运行

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

## 5. 用命令行验证

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

## 6. 本节要点

1. 一个节点 = 继承 `Node` 的类 + `main()` 里 `init/spin/shutdown`。
2. 发布用 `create_publisher` + `publish()`；订阅用 `create_subscription` + 回调。
3. `package.xml` 声明依赖，`setup.py` 的 `entry_points` 声明可执行命令。
4. 加了新包/改了 setup.py 后要重新 `colcon build`（`--symlink-install` 下改 `.py` 代码本身不用重建）。

---

下一章：[04 自定义消息](04_自定义消息.md)
