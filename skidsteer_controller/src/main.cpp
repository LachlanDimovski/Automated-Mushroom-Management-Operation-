#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "skidsteer_controller/skidsteer_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SkidsteerController>();

  std::vector<geometry_msgs::msg::Point> goals;
  geometry_msgs::msg::Point p1;
  p1.x = 3.0;
  p1.y = 0.0;
  goals.push_back(p1);

  geometry_msgs::msg::Point p2;
  p2.x = 3.0;
  p2.y = 3.0;
  goals.push_back(p2);

  geometry_msgs::msg::Point p3;
  p3.x = 0.0;
  p3.y = 0.0;
  goals.push_back(p3);

  node->setGoals(goals);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
