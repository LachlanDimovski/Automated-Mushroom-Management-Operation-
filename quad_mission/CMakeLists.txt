cmake_minimum_required(VERSION 3.8)
project(quad_mission)
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
 
include_directories(include)
 
add_executable(quad_mission_node
  src/main.cpp
  src/quadcopter.cpp
  src/mission.cpp
)
ament_target_dependencies(quad_mission_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_srvs
)
 
install(TARGETS
  quad_mission_node
  DESTINATION lib/${PROJECT_NAME}
)
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
 
ament_package()
 