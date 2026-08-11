cmake_minimum_required(VERSION 3.8)
project(skidsteer_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_executable(skidsteer_controller_node
  src/main.cpp
  src/skidsteer_controller.cpp
)
ament_target_dependencies(skidsteer_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
)

add_executable(collision_guard_node
  src/collision_guard_main.cpp
  src/collision_guard.cpp
)
ament_target_dependencies(collision_guard_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

install(TARGETS
  skidsteer_controller_node
  collision_guard_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
