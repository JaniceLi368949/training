# 02 turtlesim 初体验

先用官方自带的小海龟 demo，**不写一行代码**，直观感受节点、话题、服务长什么样。

## 1. 启动海龟

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

## 2. 看节点（node）

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

## 3. 看话题（topic）

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

## 4. 看服务（service）

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

## 5. 可视化拓扑：rqt_graph

```bash
rqt_graph
```

会弹出一个图，椭圆是**节点**，方框是**话题**，箭头表示数据流向。这就是我们调试复杂系统时最常用的"地图"。

## 6. 心智模型总结

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

---

下一章：[03 第一个 Python 节点](03_第一个Python节点.md)
