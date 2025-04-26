cmake_minimum_required(VERSION 3.8)
project(vision_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Vision.msg" DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME} USE_SOURCE_PERMISSIONS )
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()