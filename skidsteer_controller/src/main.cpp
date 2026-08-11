#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "skidsteer_controller/skidsteer_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SkidsteerController>();

  // Two-ring sweep covering the scatter radius used by
  // skidsteer_test.launch.py (targets/decoys land within ~9m of spawn): an
  // inner ring at 4m and an outer ring at 8m, each visited at 8 points
  // around the circle. Denser than a single 4-corner loop, so the search
  // actually passes near most of the area rather than cutting straight
  // lines through the middle of it. Combined with the look-around spin at
  // each waypoint (see performLookAround), this is the "searching" pass.
  std::vector<geometry_msgs::msg::Point> goals;
  for (double radius : {4.0, 8.0}) {
    for (int i = 0; i < 8; ++i) {
      double angle = i * (2.0 * M_PI / 8.0);
      geometry_msgs::msg::Point p;
      p.x = radius * std::cos(angle);
      p.y = radius * std::sin(angle);
      goals.push_back(p);
    }
  }

  node->setGoals(goals);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
