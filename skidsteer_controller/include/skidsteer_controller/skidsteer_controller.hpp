#ifndef SKIDSTEER_CONTROLLER_HPP
#define SKIDSTEER_CONTROLLER_HPP

#include <cmath>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

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
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void controlStep();
  void driveToTarget();
  void driveToWaypoint();
  bool performBackAway();
  bool performLookAround();
  bool avoidObstacle();
  bool checkStuck();
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
  // After stopping at a reached target, ignore detections for this long so the
  // robot resumes patrolling instead of parking on the same target forever.
  static constexpr double TARGET_COOLDOWN_S = 5.0;
  // Forced reverse immediately after reaching a target, before cooldown
  // starts counting: a stationary target sitting right in front of the
  // robot would otherwise still be fresh and in-tolerance the instant
  // cooldown ends, re-triggering "reached" forever instead of ever
  // resuming the search.
  static constexpr double BACK_AWAY_DURATION_S = 1.0;

  // Reactive obstacle avoidance (patrol only - visual servo is allowed to
  // approach close to a detected target on purpose). Wide margins and a wide
  // detection cone on purpose: the robot has physical width a single centre
  // beam doesn't capture, so avoidance has to start turning away well before
  // anything is actually close enough to clip.
  static constexpr double OBSTACLE_STOP_DIST_M = 3.0;
  static constexpr double OBSTACLE_REVERSE_DIST_M = 1.5;  // close enough to back away rather than just turn
  static constexpr double OBSTACLE_FORWARD_HALF_ARC_RAD = 1.0;  // ~57 deg either side of front
  static constexpr double AVOID_ANGULAR_SPEED = 2.0;
  static constexpr double AVOID_REVERSE_SPEED = -0.6;

  // Odometry-based stuck watchdog: catches cases the lidar cone misses
  // (wedged against something, scraping past a trunk at an angle, etc.).
  // If the robot hasn't actually moved for STUCK_TIMEOUT_S despite the
  // patrol loop trying to drive it, force a recovery manoeuvre. Turning
  // is tried first (cheaper, doesn't risk the rear); only if that alone
  // fails to free it does the next attempt add a short reverse.
  static constexpr double STUCK_CHECK_PERIOD_S = 2.0;
  static constexpr double STUCK_MOVE_THRESHOLD_M = 0.1;
  static constexpr double STUCK_TIMEOUT_S = 4.0;
  static constexpr double RECOVERY_TURN_ONLY_S = 2.0;   // first attempt: turn only
  static constexpr double RECOVERY_REVERSE_S = 0.8;     // escalated attempt: brief reverse...
  static constexpr double RECOVERY_TURN_S = 1.5;        // ...then turn
  // If checkStuck() wasn't reached for longer than this (robot was doing a
  // look-around spin, chasing a target, etc.), that gap doesn't count as
  // "not moving" - restart the progress baseline instead of falsely
  // tripping the watchdog the instant patrol driving resumes.
  static constexpr double STUCK_CHECK_GAP_RESET_S = 0.5;

  // Look-around search: on reaching each waypoint, rotate in place through a
  // full turn (rather than immediately driving on) so the forward-facing
  // camera actually sweeps past everything nearby instead of only seeing
  // whatever happened to be dead ahead on arrival.
  static constexpr double LOOK_AROUND_ANGULAR_SPEED = 1.1;
  static constexpr double LOOK_AROUND_DURATION_S = (2.0 * M_PI) / 1.1;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr soccer_ball_tally_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
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
  double targetClassId_ = 0.0;  // 0.0 = sports ball, 1.0 = frisbee
  rclcpp::Time targetStamp_;
  int soccerBallTally_ = 0;
  rclcpp::Time cooldownUntil_;
  bool inCooldown_ = false;

  std::vector<geometry_msgs::msg::Point> goals_;
  size_t currentGoalIdx_ = 0;

  std::vector<float> scanRanges_;
  float scanAngleMin_ = 0.0f;
  float scanAngleIncrement_ = 0.0f;
  bool haveScan_ = false;

  double lastProgressX_ = 0.0;
  double lastProgressY_ = 0.0;
  rclcpp::Time lastProgressTime_;
  bool haveProgressCheck_ = false;
  rclcpp::Time lastCheckStuckTime_;
  bool haveLastCheckStuckTime_ = false;

  bool recovering_ = false;
  bool recoveringReversePhase_ = true;
  double recoveryTurnDir_ = 1.0;
  rclcpp::Time recoveryPhaseUntil_;
  // First stuck trigger tries turning only; only if that fails to produce
  // real progress and it gets stuck again right away does the next
  // attempt escalate to a brief reverse. Reset once real progress is made.
  bool nextRecoveryTriesReverse_ = false;

  bool lookingAround_ = false;
  rclcpp::Time lookAroundUntil_;

  bool backingAway_ = false;
  rclcpp::Time backAwayUntil_;
};

#endif  // SKIDSTEER_CONTROLLER_HPP
