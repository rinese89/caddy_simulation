cmake_minimum_required(VERSION 3.8)
project(caddy_ai2_ros2_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs)
find_package(tf2_ros)
find_package(tf2_msgs)

add_executable(cmd_vel_pub code/src/cmd_vel_pub.cpp)
ament_target_dependencies(cmd_vel_pub rclcpp geometry_msgs)
install(TARGETS cmd_vel_pub DESTINATION lib/${PROJECT_NAME})

add_executable(odom2tf_broadcaster code/src/odom2tf_broadcaster.cpp)
ament_target_dependencies(odom2tf_broadcaster rclcpp tf2_ros tf2_msgs)
install(TARGETS odom2tf_broadcaster DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY bringup/launch/
  DESTINATION share/${PROJECT_NAME}/bringup/launch
)

install(
  DIRECTORY bringup/config/
  DESTINATION share/${PROJECT_NAME}/bringup/config
)

install(
  DIRECTORY description/urdf/
  DESTINATION share/${PROJECT_NAME}/description/urdf
)

install(
  DIRECTORY tools/rviz/
  DESTINATION share/${PROJECT_NAME}/tools/rviz
)

install(
  DIRECTORY meshes/stl/
  DESTINATION share/${PROJECT_NAME}/meshes/stl
)

ament_package()
