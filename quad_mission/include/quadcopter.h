#ifndef QUAD_MISSION__QUADCOPTER_H_
#define QUAD_MISSION__QUADCOPTER_H_

#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

class Quadcopter : public rclcpp::Node
{
public:
  Quadcopter();
  ~Quadcopter();
 
  void setGoal(geometry_msgs::msg::Pose goal);
  void setActive(bool active);
  nav_msgs::msg::Odometry getOdom();
  bool isGoalReached();
 
private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();
  void threadFunction();
  geometry_msgs::msg::Twist computeVelocity();
 
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVel_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trail_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread * thread_;
 
  std::mutex odomMtx_;
  std::mutex goalMtx_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::Pose goal_;
  nav_msgs::msg::Path trail_;  // accumulated flight path for RViz visualization
  bool active_;  // NOTE: matches original — read from timer + log thread without a lock
  bool goalSet_;
 
  static constexpr double GOAL_TOLERANCE = 0.3;
  static constexpr double MAX_LINEAR_VEL = 1.0;
  static constexpr double MAX_ANGULAR_VEL = 1.0;
  static constexpr double TRAIL_MIN_SPACING = 0.1;  // metres between recorded trail points
  static constexpr size_t TRAIL_MAX_POINTS = 2000;  // cap so the Path message doesn't grow unbounded
};
 
#endif  // QUAD_MISSION__QUADCOPTER_H_
