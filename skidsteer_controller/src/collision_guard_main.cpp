#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "skidsteer_controller/collision_guard.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionGuard>());
  rclcpp::shutdown();
  return 0;
}
