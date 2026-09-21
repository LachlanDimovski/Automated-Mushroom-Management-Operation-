// Low-level quad controller — drives towards a set goal pose via
// proportional control on cmd_vel. Called by Mission; not meant to be
// commanded directly outside this package.
// Made and Upkept by Darcy Moyes - 25280025
// last patch was 4 September 26
#ifndef QUADCOPTER_MISSION__QUADCOPTER_HPP_
#define QUADCOPTER_MISSION__QUADCOPTER_HPP_
 
#include <mutex>
#include <thread>
 
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
 
// Low-level drone controller: holds a goal pose and drives towards it with
// proportional control, same structure as the sjtu_drone-based Quadcopter
// this was ported from. Adapted for the Parrot (parrot1) from the 41068
// bringup package instead of sjtu_drone.
//
// NOTE: the Parrot is documented as "essentially a Husky-like robot
// floating near the ground, gravity disabled" - driven directly by cmd_vel,
// with no separate arm/disarm flight-controller step. So there is no
// /parrot1/takeoff or /parrot1/land topic equivalent to sjtu_drone's -
// takeoff/land here are just pose goals with a target z, using the same
// setGoal()/computeVelocity() path as horizontal motion. Verify this
// assumption by checking `ros2 topic list | grep parrot1` on your VM;
// if a dedicated takeoff/land topic does turn out to exist, it's easy to
// add back as an extra publish alongside the pose-goal approach.
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread * thread_;
 
  std::mutex odomMtx_;
  std::mutex goalMtx_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::Pose goal_;
  bool active_;  // NOTE: matches original — read from timer + log thread without a lock
  bool goalSet_;
 
  static constexpr double GOAL_TOLERANCE = 0.3;
  static constexpr double MAX_LINEAR_VEL = 1.0;
  static constexpr double MAX_ANGULAR_VEL = 1.0;
};
 
#endif  // QUADCOPTER_MISSION__QUADCOPTER_HPP_
 