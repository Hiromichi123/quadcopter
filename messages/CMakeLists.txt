cmake_minimum_required(VERSION 3.8)
project(messages)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas -Wno-unused-function) # 忽视region、未使用function的警告
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/VisionMsg.msg" "msg/Vision.msg")
ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})

ament_package()
