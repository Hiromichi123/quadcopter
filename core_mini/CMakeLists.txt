cmake_minimum_required(VERSION 3.8)
project(core_mini)

# 设置 C++ 标准为 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 启用编译警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(ros2_tools REQUIRED)

# 编译可执行文件
add_executable(quad_mini_node src/quadcopter_mini.cpp)

# 包含头文件目录
target_include_directories(quad_mini_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 链接依赖
ament_target_dependencies(quad_mini_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  std_msgs 
  mavros_msgs 
  ros2_tools
)

# 安装可执行文件
install(TARGETS quad_mini_node 
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
