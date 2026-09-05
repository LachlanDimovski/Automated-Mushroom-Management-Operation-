cmake_minimum_required(VERSION 3.8)
project(quadcopter_mission)
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
 
add_executable(quadcopter_mission_node src/quadcopter_mission.cpp)
ament_target_dependencies(quadcopter_mission_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  std_srvs
)
 
install(TARGETS
  quadcopter_mission_node
  DESTINATION lib/${PROJECT_NAME}
)
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
 
ament_package()
 