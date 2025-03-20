cmake_minimum_required(VERSION 3.8)
project(cv_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ros2_tools REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Vision.msg")
ament_export_dependencies(rosidl_default_runtime)

add_executable(vision_node src/vision_node.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vision_node ${cpp_typesupport_target})

ament_target_dependencies(vision_node rclcpp sensor_msgs OpenCV cv_bridge ros2_tools)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS vision_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})

ament_package()
