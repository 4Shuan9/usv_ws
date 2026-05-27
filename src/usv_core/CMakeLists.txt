cmake_minimum_required(VERSION 3.8)
project(usv_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(px4_odom_bridge src/px4_odom_bridge.cpp)
ament_target_dependencies(px4_odom_bridge rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros px4_msgs)

add_executable(cmd_vel_to_px4 src/cmd_vel_to_px4.cpp)
ament_target_dependencies(cmd_vel_to_px4 rclcpp geometry_msgs px4_msgs tf2)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  px4_odom_bridge
  cmd_vel_to_px4
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()