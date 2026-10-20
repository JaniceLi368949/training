cmake_minimum_required(VERSION 3.8)
project(rm_msgs)

# 使用 rosidl 默认生成器（会为 msg/srv 生成 C++/Python 等多种语言代码）
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalTarget.msg"
  "msg/ChassisCmd.msg"
  "msg/WheelLegCmd.msg"
  "srv/SetGimbalAngle.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
