cmake_minimum_required(VERSION 3.8)
project(arm_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)  # ✅ 新增：MotorFeedback需要时间戳

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseCommand.msg"
  "msg/MotorPos.msg"
  "msg/MotorFeedback.msg"  # ✅ 新增
  DEPENDENCIES builtin_interfaces  # ✅ 新增：声明依赖
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()