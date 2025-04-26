cmake_minimum_required(VERSION 3.8)
project(core)

set(CMAKE_CXX_STANDARD 17)           # 设置 C++ 标准为 C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # 必须使用指定的标准
set(CMAKE_CXX_EXTENSIONS OFF)        # 禁用编译器特定的扩展

# 为对应编译器启用更严格的编译警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 基础编译包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# 接口包
find_package(rosidl_default_generators REQUIRED)
# ros2消息包
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
# 自定义工具/消息包
find_package(ros2_tools REQUIRED)
find_package(vision_py REQUIRED)

# 消息包声明，向外部暴露接口
#rosidl_generate_interfaces(${PROJECT_NAME} "msg/BallPosition.msg" "msg/Calibrate.msg" "msg/BallDepth.msg" "srv/PickRelease.srv")
#ament_export_dependencies(rosidl_default_runtime)

# 可执行文件编译
add_executable(quad_node src/classes/flight_controller.cpp src/classes/quadcopter.cpp)

# 可执行节点包含头文件安装
target_include_directories(quad_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/src/classes
)

# cpp包内消息支持
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(quad_node ${cpp_typesupport_target})

# 可执行文件编译依赖
ament_target_dependencies(quad_node rclcpp geometry_msgs nav_msgs std_msgs mavros_msgs ros2_tools vision_py)
set(CMAKE_INSTALL_RPATH /usr/local/lib)

# 可执行文件安装
install(TARGETS quad_node DESTINATION lib/${PROJECT_NAME})
#install(DIRECTORY msg srv DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/)

ament_package()