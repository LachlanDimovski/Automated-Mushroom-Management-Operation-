#include "skidsteer_controller/skidsteer_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

SkidsteerController::SkidsteerController()
: Node("skidsteer_controller")
{
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("husky1/cmd_vel_raw", 10);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "husky1/odometry", 10,
    std::bind(&SkidsteerController::odomCallback, this, std::placeholders::_1));
  target_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "husky1/target_info", 10,
    std::bind(&SkidsteerController::targetCallback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(CONTROL_STEP_MS),
    std::bind(&SkidsteerController::controlStep, this));
}

void SkidsteerController::setGoals(const std::vector<geometry_msgs::msg::Point> & goals)
{
  std::lock_guard<std::mutex> lock(mtx_);
  goals_ = goals;
  currentGoalIdx_ = 0;
}

void SkidsteerController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  std::lock_guard<std::mutex> lock(mtx_);
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  yaw_ = yaw;
  haveOdom_ = true;
}

// husky1/target_info layout: [detected(0/1), bearing(-1..1), distance_m, stop_distance_m]
void SkidsteerController::targetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    return;
  }
  std::lock_guard<std::mutex> lock(mtx_);
  targetDetected_ = msg->data[0] > 0.5f;
  targetBearing_ = msg->data[1];
  targetDistance_ = msg->data[2];
  targetStopDistance_ = msg->data[3];
  targetStamp_ = now();
}

void SkidsteerController::controlStep()
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (!haveOdom_) {
    return;
  }

  bool targetFresh = targetDetected_ &&
    (now() - targetStamp_).seconds() < TARGET_TIMEOUT_S;

  if (targetFresh) {
    driveToTarget();
  } else {
    driveToWaypoint();
  }
}

// Visual servo: turn to centre the target in frame, then approach it until
// (distance - stop_distance) is within tolerance. stop_distance is set by
// the perception node based on the target's colour (red vs. not).
void SkidsteerController::driveToTarget()
{
  if (targetDistance_ < 0.0) {
    // Detected but no valid depth reading yet - hold rather than guess.
    stopPlatform();
    return;
  }

  double rangeError = targetDistance_ - targetStopDistance_;
  if (std::fabs(rangeError) <= VISUAL_DISTANCE_TOLERANCE_M &&
      std::fabs(targetBearing_) <= VISUAL_BEARING_THRESHOLD)
  {
    stopPlatform();
    return;
  }

  double angular = std::max(
    -MAX_ANGULAR_SPEED,
    std::min(MAX_ANGULAR_SPEED, -targetBearing_ * VISUAL_ANGULAR_GAIN * MAX_ANGULAR_SPEED));

  double linear = 0.0;
  if (std::fabs(targetBearing_) < VISUAL_BEARING_THRESHOLD) {
    linear = std::max(
      -MAX_LINEAR_SPEED,
      std::min(MAX_LINEAR_SPEED, rangeError * VISUAL_LINEAR_GAIN));
  }

  sendCmd(angular, linear);
}

void SkidsteerController::driveToWaypoint()
{
  if (currentGoalIdx_ >= goals_.size()) {
    return;
  }

  const auto & goal = goals_[currentGoalIdx_];
  double dx = goal.x - x_;
  double dy = goal.y - y_;
  double dist = std::hypot(dx, dy);

  double targetAngle = std::atan2(dy, dx);
  double angleDiff = std::atan2(std::sin(targetAngle - yaw_), std::cos(targetAngle - yaw_));

  double angular = std::max(
    -MAX_ANGULAR_SPEED, std::min(MAX_ANGULAR_SPEED, angleDiff * ANGULAR_GAIN));

  double linear = 0.0;
  if (std::fabs(angleDiff) < ANGLE_THRESHOLD_RAD) {
    if (dist < SLOW_ZONE_M) {
      linear = std::min(SLOW_SPEED_MPS, dist);
    } else {
      double speedScale = std::min(1.0, dist / SPEED_RAMP_DIST_M);
      linear = std::min(MAX_LINEAR_SPEED * speedScale, dist);
    }
  }

  if (dist <= GOAL_TOLERANCE_M) {
    stopPlatform();
    ++currentGoalIdx_;
    RCLCPP_INFO(get_logger(), "Reached goal %zu", currentGoalIdx_);
    return;
  }

  sendCmd(angular, linear);
}

void SkidsteerController::sendCmd(double turn_l_r, double move_f_b)
{
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = turn_l_r;
  cmd.linear.x = move_f_b;
  cmd_vel_pub_->publish(cmd);
}

void SkidsteerController::stopPlatform()
{
  for (int i = 0; i < STOP_CMD_COUNT; ++i) {
    sendCmd(0.0, 0.0);
  }
}
