cmake_minimum_required(VERSION 3.8)
project(scservo_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(arm_interface REQUIRED)

# 包含 scservo_sdk 目录下的头文件
include_directories(src)

# 收集 SCServo 驱动的所有源文件
file(GLOB SCSERVO_SOURCES "src/scservo_sdk/*.cpp")

# ========== 定义控制节点可执行文件 ==========
add_executable(scservo_node 
  src/scservo_node.cpp
  ${SCSERVO_SOURCES}
)

ament_target_dependencies(scservo_node 
  rclcpp 
  std_msgs
  arm_interface
)

# ========== 定义反馈节点可执行文件 ========== ✅ 新增
add_executable(scservo_feedback_node 
  src/scservo_feedback_node.cpp
  ${SCSERVO_SOURCES}
)

ament_target_dependencies(scservo_feedback_node 
  rclcpp 
  std_msgs
  arm_interface
)

# 安装目标
install(TARGETS
  scservo_node
  scservo_feedback_node  # ✅ 新增
  DESTINATION lib/${PROJECT_NAME})

ament_package()