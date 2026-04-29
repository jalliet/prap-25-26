cmake_minimum_required(VERSION 3.8)
project(scservo_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Added -Wno-vla to suppress the SDK warnings
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-vla)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(poker_interfaces REQUIRED)

include_directories(src)

# Compile SDK sources directly
file(GLOB SDK_SOURCES "src/scservo_sdk/*.cpp")

add_executable(driver_node 
  src/scservo_driver_node.cpp
  ${SDK_SOURCES}
)

ament_target_dependencies(driver_node 
  rclcpp 
  poker_interfaces
)

install(TARGETS driver_node DESTINATION lib/${PROJECT_NAME})
ament_package()