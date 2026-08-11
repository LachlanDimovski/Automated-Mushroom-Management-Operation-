#include "skidsteer_controller/skidsteer_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

SkidsteerController::SkidsteerController()
: Node("skidsteer_controller")
{
  // Published as "raw" - collision_guard_node is the final gate that actually
  // reaches husky1/cmd_vel, so this patrol/visual-servo avoidance and the
  // safety layer both apply regardless of which node is driving.
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("husky1/cmd_vel_raw", 10);
  soccer_ball_tally_pub_ = create_publisher<std_msgs::msg::Int32>(
    "husky1/soccer_ball_tally",
    rclcpp::QoS(10).transient_local());
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "husky1/odometry", 10,
    std::bind(&SkidsteerController::odomCallback, this, std::placeholders::_1));
  target_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "husky1/target_info", 10,
    std::bind(&SkidsteerController::targetCallback, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "husky1/scan", 10,
    std::bind(&SkidsteerController::scanCallback, this, std::placeholders::_1));

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

// husky1/target_info layout: [detected(0/1), bearing(-1..1), distance_m,
// stop_distance_m, class_id(0=sports ball, 1=frisbee)]
void SkidsteerController::targetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 5) {
    return;
  }
  std::lock_guard<std::mutex> lock(mtx_);
  targetDetected_ = msg->data[0] > 0.5f;
  targetBearing_ = msg->data[1];
  targetDistance_ = msg->data[2];
  targetStopDistance_ = msg->data[3];
  targetClassId_ = msg->data[4];
  targetStamp_ = now();
}

void SkidsteerController::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  scanRanges_ = msg->ranges;
  scanAngleMin_ = msg->angle_min;
  scanAngleIncrement_ = msg->angle_increment;
  haveScan_ = true;
}

// Reactive avoidance for the patrol loop: if something is close ahead, turn
// in place towards whichever side (left/right of the forward cone) is more
// open, rather than blindly pushing into it. Returns true (and issues a
// turn command) when it took over control this step.
bool SkidsteerController::avoidObstacle()
{
  if (!haveScan_ || scanRanges_.empty() || scanAngleIncrement_ == 0.0f) {
    return false;
  }

  float leftMin = std::numeric_limits<float>::infinity();
  float rightMin = std::numeric_limits<float>::infinity();
  float forwardMin = std::numeric_limits<float>::infinity();

  for (size_t i = 0; i < scanRanges_.size(); ++i) {
    float range = scanRanges_[i];
    if (!std::isfinite(range) || range <= 0.0f) {
      continue;
    }
    // Normalise into (-pi, pi]: this lidar reports angle_min=0/angle_max=2pi,
    // so "forward" can appear as angle near 0 OR near 2pi depending on
    // mounting - without wrapping, obstacles approaching from one side of
    // dead-ahead were invisible to the front-cone check entirely.
    float angle = std::remainder(
      scanAngleMin_ + static_cast<float>(i) * scanAngleIncrement_,
      2.0f * static_cast<float>(M_PI));
    if (std::fabs(angle) > OBSTACLE_FORWARD_HALF_ARC_RAD) {
      continue;
    }
    forwardMin = std::min(forwardMin, range);
    if (angle >= 0.0f) {
      leftMin = std::min(leftMin, range);
    } else {
      rightMin = std::min(rightMin, range);
    }
  }

  if (forwardMin > OBSTACLE_STOP_DIST_M) {
    return false;
  }

  double turnDirection = (leftMin >= rightMin) ? 1.0 : -1.0;
  if (forwardMin < OBSTACLE_REVERSE_DIST_M) {
    // Too close to just turn on the spot (e.g. already touching a tree
    // trunk) - back away first, curving towards the more open side.
    sendCmd(turnDirection * AVOID_ANGULAR_SPEED * 0.5, AVOID_REVERSE_SPEED);
  } else {
    sendCmd(turnDirection * AVOID_ANGULAR_SPEED, 0.0);
  }
  return true;
}

// Watchdog independent of the lidar cone: if the robot hasn't actually
// moved for STUCK_TIMEOUT_S despite the patrol loop trying to drive it
// (wedged at an angle the forward cone doesn't see, scraping a trunk,
// etc.), force a reverse-then-turn recovery manoeuvre. Returns true (and
// issues a command) while a recovery is in progress.
bool SkidsteerController::checkStuck()
{
  rclcpp::Time nowTime = now();

  if (!haveProgressCheck_) {
    lastProgressX_ = x_;
    lastProgressY_ = y_;
    lastProgressTime_ = nowTime;
    haveProgressCheck_ = true;
  }

  if (haveLastCheckStuckTime_ &&
      (nowTime - lastCheckStuckTime_).seconds() > STUCK_CHECK_GAP_RESET_S)
  {
    // Wasn't reached for a while (look-around spin, chasing a target,
    // cooldown, ...) - that time doesn't count as failing to make progress.
    lastProgressX_ = x_;
    lastProgressY_ = y_;
    lastProgressTime_ = nowTime;
  }
  lastCheckStuckTime_ = nowTime;
  haveLastCheckStuckTime_ = true;

  if (recovering_) {
    if (nowTime >= recoveryPhaseUntil_) {
      if (recoveringReversePhase_) {
        recoveringReversePhase_ = false;
        recoveryPhaseUntil_ = nowTime + rclcpp::Duration::from_seconds(RECOVERY_TURN_S);
      } else {
        recovering_ = false;
        lastProgressX_ = x_;
        lastProgressY_ = y_;
        lastProgressTime_ = nowTime;
        RCLCPP_INFO(get_logger(), "Recovery complete, resuming patrol");
        return false;
      }
    }
    if (recoveringReversePhase_) {
      sendCmd(0.0, AVOID_REVERSE_SPEED);
    } else {
      sendCmd(recoveryTurnDir_ * AVOID_ANGULAR_SPEED, 0.0);
    }
    return true;
  }

  double moved = std::hypot(x_ - lastProgressX_, y_ - lastProgressY_);
  if (moved > STUCK_MOVE_THRESHOLD_M) {
    lastProgressX_ = x_;
    lastProgressY_ = y_;
    lastProgressTime_ = nowTime;
    nextRecoveryTriesReverse_ = false;  // real progress - back to trying turn-only first
    return false;
  }

  if ((nowTime - lastProgressTime_).seconds() < STUCK_TIMEOUT_S) {
    return false;
  }

  bool useReverse = nextRecoveryTriesReverse_;
  nextRecoveryTriesReverse_ = true;  // if this attempt also fails, escalate further next time
  recovering_ = true;
  if (useReverse) {
    RCLCPP_WARN(get_logger(), "Still stuck - turning alone didn't work, reversing then turning to recover");
    recoveringReversePhase_ = true;
    recoveryPhaseUntil_ = nowTime + rclcpp::Duration::from_seconds(RECOVERY_REVERSE_S);
  } else {
    RCLCPP_WARN(get_logger(), "Stuck - not moving, turning to recover");
    recoveringReversePhase_ = false;
    recoveryPhaseUntil_ = nowTime + rclcpp::Duration::from_seconds(RECOVERY_TURN_ONLY_S);
  }

  double leftMin = std::numeric_limits<float>::infinity();
  double rightMin = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < scanRanges_.size(); ++i) {
    float range = scanRanges_[i];
    if (!std::isfinite(range) || range <= 0.0f) {
      continue;
    }
    // Normalise into (-pi, pi]: this lidar reports angle_min=0/angle_max=2pi,
    // so "forward" can appear as angle near 0 OR near 2pi depending on
    // mounting - without wrapping, obstacles approaching from one side of
    // dead-ahead were invisible to the front-cone check entirely.
    float angle = std::remainder(
      scanAngleMin_ + static_cast<float>(i) * scanAngleIncrement_,
      2.0f * static_cast<float>(M_PI));
    if (angle >= 0.0f) {
      leftMin = std::min(leftMin, static_cast<double>(range));
    } else {
      rightMin = std::min(rightMin, static_cast<double>(range));
    }
  }
  recoveryTurnDir_ = (leftMin >= rightMin) ? 1.0 : -1.0;

  sendCmd(0.0, AVOID_REVERSE_SPEED);
  return true;
}

// Forced reverse right after reaching a target, overriding everything
// (including a fresh re-detection of the same still-visible target) so the
// robot is physically further away before it's ever allowed to re-evaluate
// whether it has "reached" something.
bool SkidsteerController::performBackAway()
{
  if (!backingAway_) {
    return false;
  }

  if (now() >= backAwayUntil_) {
    backingAway_ = false;
    return false;
  }

  sendCmd(0.0, AVOID_REVERSE_SPEED);
  return true;
}

void SkidsteerController::controlStep()
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (!haveOdom_) {
    return;
  }

  if (inCooldown_ && now() >= cooldownUntil_) {
    inCooldown_ = false;
    RCLCPP_INFO(get_logger(), "Cooldown over, resuming search for the next target");
  }

  if (performBackAway()) {
    return;
  }

  bool targetFresh = !inCooldown_ && targetDetected_ &&
    (now() - targetStamp_).seconds() < TARGET_TIMEOUT_S;

  if (targetFresh) {
    lookingAround_ = false;  // a real detection takes priority over a scan-in-progress
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
    RCLCPP_INFO(get_logger(), "Reached target at %.2fm - alerting, then resuming search", targetDistance_);
    if (targetClassId_ < 0.5) {  // 0.0 = sports ball
      ++soccerBallTally_;
      std_msgs::msg::Int32 tallyMsg;
      tallyMsg.data = soccerBallTally_;
      soccer_ball_tally_pub_->publish(tallyMsg);
      RCLCPP_INFO(get_logger(), "Soccer ball tally: %d", soccerBallTally_);
    }
    stopPlatform();
    backingAway_ = true;
    backAwayUntil_ = now() + rclcpp::Duration::from_seconds(BACK_AWAY_DURATION_S);
    inCooldown_ = true;
    cooldownUntil_ = now() + rclcpp::Duration::from_seconds(TARGET_COOLDOWN_S);
    if (!goals_.empty()) {
      currentGoalIdx_ = (currentGoalIdx_ + 1) % goals_.size();
    }
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

// Ported from skeleton/a2_skeleton/skidsteer.cpp Skidsteer::runControlStep
void SkidsteerController::driveToWaypoint()
{
  if (currentGoalIdx_ >= goals_.size()) {
    return;
  }

  if (performLookAround()) {
    return;
  }

  if (checkStuck()) {
    return;
  }

  if (avoidObstacle()) {
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
    size_t reachedIdx = currentGoalIdx_;
    // Advance now, not when the look-around finishes: a transient target
    // detection can interrupt/restart the look-around (see controlStep),
    // and if the index only advanced on completion, a still-current
    // waypoint would just keep re-triggering "reached" forever instead of
    // ever letting the interrupted look-around's next attempt move on.
    if (!goals_.empty()) {
      currentGoalIdx_ = (currentGoalIdx_ + 1) % goals_.size();
    }
    lookingAround_ = true;
    lookAroundUntil_ = now() + rclcpp::Duration::from_seconds(LOOK_AROUND_DURATION_S);
    RCLCPP_INFO(get_logger(), "Reached waypoint %zu, looking around before moving on", reachedIdx);
    return;
  }

  sendCmd(angular, linear);
}

// Rotate in place for a full turn on arrival at a waypoint, so the
// forward-facing camera actually sweeps past everything nearby instead of
// only ever seeing whatever was dead ahead when the robot stopped. The
// waypoint index was already advanced when the look-around started.
bool SkidsteerController::performLookAround()
{
  if (!lookingAround_) {
    return false;
  }

  if (now() >= lookAroundUntil_) {
    lookingAround_ = false;
    RCLCPP_INFO(get_logger(), "Look-around done, patrolling to waypoint %zu", currentGoalIdx_);
    return false;
  }

  sendCmd(LOOK_AROUND_ANGULAR_SPEED, 0.0);
  return true;
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
