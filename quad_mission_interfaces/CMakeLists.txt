cmake_minimum_required(VERSION 3.8)
project(quad_mission_interfaces)
 
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
 
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartSearch.srv"
)
 
ament_package()
 