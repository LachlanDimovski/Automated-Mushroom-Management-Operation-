#ifndef SKIDSTEER_CONTROLLER_HPP
#define SKIDSTEER_CONTROLLER_HPP

#include <cmath>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class SkidsteerController : public rclcpp::Node
{
public:
  SkidsteerController();

  // Patrol waypoint loop, driven only while no YOLO target is visible.
  // Once the last point is reached the robot returns to the first, so it
  // keeps exploring indefinitely until a target is spotted.
  void setGoals(const std::vector<geometry_msgs::msg::Point> & goals);

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void targetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void controlStep();
  void driveToTarget();
  void driveToWaypoint();
  void sendCmd(double turn_l_r, double move_f_b);
  void stopPlatform();

  // Ported from skeleton/a2_skeleton/skidsteer.h, then slowed down a lot -
  // the original speeds outran YOLO's ~2-3Hz detection rate, so the robot
  // would blow past targets before a detection could ever lock on.
  static constexpr double MAX_LINEAR_SPEED = 1.0;
  static constexpr double MAX_ANGULAR_SPEED = 0.8;
  static constexpr double SPEED_RAMP_DIST_M = 5.0;
  static constexpr double SLOW_ZONE_M = 2.0;
  static constexpr double SLOW_SPEED_MPS = 0.3;
  static constexpr double ANGLE_THRESHOLD_RAD = 0.3;
  static constexpr double ANGULAR_GAIN = 2.0;
  static constexpr double GOAL_TOLERANCE_M = 0.5;
  static constexpr int CONTROL_STEP_MS = 15;
  static constexpr int STOP_CMD_COUNT = 10;

  // Visual-servo tuning for chasing a YOLO target to a colour-gated stop distance.
  static constexpr double VISUAL_ANGULAR_GAIN = 1.0;
  static constexpr double VISUAL_BEARING_THRESHOLD = 0.15;  // normalised (-1..1)
  static constexpr double VISUAL_DISTANCE_TOLERANCE_M = 0.3;
  static constexpr double VISUAL_LINEAR_GAIN = 0.6;
  // Target considered stale (and dropped back to waypoint fallback) after this long.
  static constexpr double TARGET_TIMEOUT_S = 1.0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  bool haveOdom_ = false;

  bool targetDetected_ = false;
  double targetBearing_ = 0.0;
  double targetDistance_ = -1.0;
  double targetStopDistance_ = 0.0;
  rclcpp::Time targetStamp_;

  std::vector<geometry_msgs::msg::Point> goals_;
  size_t currentGoalIdx_ = 0;
};

#endif  // SKIDSTEER_CONTROLLER_HPP
