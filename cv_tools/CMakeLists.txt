cmake_minimum_required(VERSION 3.8)
project(cv_tools)

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
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/VisionMsg.msg" "msg/Vision.msg")
ament_export_dependencies(rosidl_default_runtime)

add_executable(vision_node src/d435.cpp)

target_include_directories(vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vision_node ${cpp_typesupport_target})

ament_target_dependencies(vision_node rclcpp sensor_msgs OpenCV cv_bridge)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS vision_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
