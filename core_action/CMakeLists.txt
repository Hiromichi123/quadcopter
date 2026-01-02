cmake_minimum_required(VERSION 3.8)
project(core_action)

# 设置 CMake 策略以消除警告
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖包 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成 Action 接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToTarget.action"
  "action/FlyByVelocity.action"
  DEPENDENCIES geometry_msgs
)

# 包含目录
include_directories(include)

# Action Server 节点
add_executable(flight_action_server src/flight_action_server.cpp)
ament_target_dependencies(flight_action_server rclcpp rclcpp_action rclcpp_components geometry_msgs nav_msgs)
# 让 action server 能使用自己包内的 action 定义
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(flight_action_server "${cpp_typesupport_target}")

# Action Client 节点
add_executable(flight_action_client src/flight_action_client.cpp)
ament_target_dependencies(flight_action_client rclcpp rclcpp_action geometry_msgs)
target_link_libraries(flight_action_client "${cpp_typesupport_target}")

# Advanced Flight Client 节点
add_executable(advanced_flight_client src/advanced_flight_client.cpp)
ament_target_dependencies(advanced_flight_client rclcpp rclcpp_action geometry_msgs)
target_link_libraries(advanced_flight_client "${cpp_typesupport_target}")

# Velocity Action Server 节点
add_executable(velocity_action_server src/velocity_action_server.cpp)
ament_target_dependencies(velocity_action_server rclcpp rclcpp_action rclcpp_components geometry_msgs nav_msgs)
target_link_libraries(velocity_action_server "${cpp_typesupport_target}")

# Velocity Action Client 节点
add_executable(velocity_action_client src/velocity_action_client.cpp)
ament_target_dependencies(velocity_action_client rclcpp rclcpp_action geometry_msgs)
target_link_libraries(velocity_action_client "${cpp_typesupport_target}")

# Mixed Flight Mission 综合示例
add_executable(mixed_flight_mission src/mixed_flight_mission.cpp)
ament_target_dependencies(mixed_flight_mission rclcpp rclcpp_action geometry_msgs)
target_link_libraries(mixed_flight_mission "${cpp_typesupport_target}")

# 安装可执行文件
install(TARGETS
  flight_action_server
  flight_action_client
  advanced_flight_client
  velocity_action_server
  velocity_action_client
  mixed_flight_mission
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include) # 安装头h/hpp
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/) # 安装 launch 文件

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
