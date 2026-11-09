cmake_minimum_required(VERSION 3.5)
project(vesc_msgs)

# 원본(f1tenth/vesc ros2) 은 ament_cmake_auto 를 쓴다. 없는 배포판이 있어 ament_cmake 로 바꿨다 (sim/README.md).
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(vesc_msgs
  "msg/VescState.msg"
  "msg/VescStateStamped.msg"
  "msg/VescImu.msg"
  "msg/VescImuStamped.msg"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
