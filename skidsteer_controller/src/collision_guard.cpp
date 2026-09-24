#include "skidsteer_controller/collision_guard.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>

using namespace std::chrono_literals;

CollisionGuard::CollisionGuard()
: Node("collision_guard")
{
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("husky1/cmd_vel", 10);
  cmd_vel_raw_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "husky1/cmd_vel_raw", 10,
    std::bind(&CollisionGuard::cmdCallback, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "husky1/scan", 10,
    std::bind(&CollisionGuard::scanCallback, this, std::placeholders::_1));
  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "husky1/camera/depth/image", 10,
    std::bind(&CollisionGuard::depthCallback, this, std::placeholders::_1));
  guard_timer_ = create_wall_timer(
    std::chrono::milliseconds(GUARD_TIMER_MS),
    std::bind(&CollisionGuard::guardTimerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "Collision guard active: full 360-degree envelope, front/rear stop %.2fm, side scrape guard %.2fm",
    FORWARD_STOP_DIST_M, SIDE_STOP_DIST_M);
}

void CollisionGuard::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  scanRanges_ = msg->ranges;
  scanAngleMin_ = msg->angle_min;
  scanAngleIncrement_ = msg->angle_increment;
  haveScan_ = true;
}

// Depth image is 32-bit float metres per pixel (encoding "32FC1"), so we
// read it directly rather than pulling in cv_bridge for one min-reduction.
// The lidar can't physically tilt down, so its single fixed-height plane
// misses low obstacles entirely - tree roots, low grass clumps, anything
// below ~0.68m in this forest world. The crop is biased towards the bottom
// of the frame (closer to the ground) to cover that gap, but only down to
// a row whose ray would hit flat ground at GROUND_SAFE_DIST_M or further -
// past that point every row sees bare earth close up regardless of any
// real obstacle, which would permanently read as "blocked" on open ground.
void CollisionGuard::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg->encoding != "32FC1" || msg->height == 0 || msg->width == 0) {
    return;
  }

  double verticalHalfFov = std::atan(std::tan(CAMERA_HORIZONTAL_FOV_RAD / 2.0) / CAMERA_ASPECT);
  double maxSafeDepressionRad = std::atan(CAMERA_HEIGHT_M / GROUND_SAFE_DIST_M);
  double safeFractionBelowCenter = std::clamp(maxSafeDepressionRad / verticalHalfFov, 0.0, 1.0);

  uint32_t xStart = msg->width / 4;
  uint32_t xEnd = msg->width - xStart;
  uint32_t yStart = msg->height * 3 / 8;  // skip the upper canopy/sky band
  uint32_t yEnd = msg->height / 2 +
    static_cast<uint32_t>(safeFractionBelowCenter * static_cast<double>(msg->height) / 2.0);
  yEnd = std::min(yEnd, msg->height);

  float minDepth = std::numeric_limits<float>::infinity();
  for (uint32_t y = yStart; y < yEnd; ++y) {
    const uint8_t * row = msg->data.data() + static_cast<size_t>(y) * msg->step;
    for (uint32_t x = xStart; x < xEnd; ++x) {
      float value;
      std::memcpy(&value, row + static_cast<size_t>(x) * sizeof(float), sizeof(float));
      if (std::isfinite(value) && value > 0.0f) {
        minDepth = std::min(minDepth, value);
      }
    }
  }

  depthForwardMin_ = minDepth;
  haveDepth_ = true;

  // Cliff check: sample a row whose ray should hit flat ground at
  // CLIFF_CHECK_EXPECTED_DIST_M under normal terrain. If the floor is
  // there, this reads close to that distance; if it isn't (a drop-off),
  // the ray either finds nothing (infinite) or hits ground much further
  // away/below, both of which read as "much further than expected".
  double cliffDepressionRad = std::atan(CAMERA_HEIGHT_M / CLIFF_CHECK_EXPECTED_DIST_M);
  double cliffFractionBelowCenter = std::clamp(cliffDepressionRad / verticalHalfFov, 0.0, 1.0);
  uint32_t cliffRow = msg->height / 2 +
    static_cast<uint32_t>(cliffFractionBelowCenter * static_cast<double>(msg->height) / 2.0);
  cliffRow = std::min(cliffRow, msg->height - 1);

  float cliffMax = 0.0f;
  bool cliffAnyValid = false;
  const uint8_t * cliffRowPtr = msg->data.data() + static_cast<size_t>(cliffRow) * msg->step;
  for (uint32_t x = xStart; x < xEnd; ++x) {
    float value;
    std::memcpy(&value, cliffRowPtr + static_cast<size_t>(x) * sizeof(float), sizeof(float));
    if (std::isfinite(value) && value > 0.0f) {
      cliffMax = std::max(cliffMax, value);
      cliffAnyValid = true;
    }
  }

  cliffCheckDist_ = cliffAnyValid ? cliffMax : std::numeric_limits<float>::infinity();
  cliffDetected_ = cliffCheckDist_ > CLIFF_CHECK_TRIGGER_DIST_M;
}

void CollisionGuard::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  lastRawCmd_ = *msg;
  lastRawCmdStamp_ = now();
  haveRawCmd_ = true;
  cmd_vel_pub_->publish(applyGuard(lastRawCmd_));
}

// Re-applies the gate to whatever was last commanded, independent of
// whether a new raw command has actually arrived - this is what makes the
// guard react to a newly-close obstacle even mid-drive, not just at the
// instant a command is issued.
void CollisionGuard::guardTimerCallback()
{
  if (!haveRawCmd_) {
    return;
  }

  if ((now() - lastRawCmdStamp_).seconds() > RAW_CMD_TIMEOUT_S) {
    haveRawCmd_ = false;  // stop once, then go quiet so Nav2 / the unguarded ALIGN path own cmd_vel
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  cmd_vel_pub_->publish(applyGuard(lastRawCmd_));
}

// Hardcoded, full-circle safety envelope applied to every command that
// reaches the robot, regardless of source (teleop, autonomous patrol,
// visual servo, recovery, back-away). The scan is split into four
// non-overlapping bands covering the whole 360 degrees:
//   front - gates forward motion
//   rear  - gates reverse motion
//   left/right sides - gate turning into something already at the flank
// so nothing approaching from any direction can be driven into unchecked.
geometry_msgs::msg::Twist CollisionGuard::applyGuard(geometry_msgs::msg::Twist cmd)
{
  if (!haveScan_ || scanRanges_.empty() || scanAngleIncrement_ == 0.0f) {
    return cmd;
  }

  float frontMin = std::numeric_limits<float>::infinity();
  float rearMin = std::numeric_limits<float>::infinity();
  float leftSideMin = std::numeric_limits<float>::infinity();
  float rightSideMin = std::numeric_limits<float>::infinity();
  // Left/right split within just the front cone, used only to pick which
  // way the steering assist should nudge (not the scrape guard below).
  float frontLeftMin = std::numeric_limits<float>::infinity();
  float frontRightMin = std::numeric_limits<float>::infinity();

  const float pi = static_cast<float>(M_PI);

  for (size_t i = 0; i < scanRanges_.size(); ++i) {
    float range = scanRanges_[i];
    if (!std::isfinite(range) || range <= 0.0f) {
      continue;
    }
    // Normalise into (-pi, pi]: this lidar reports angle_min=0/angle_max=2pi,
    // so without wrapping, obstacles approaching from one side of
    // dead-ahead were invisible to the front-cone check entirely.
    float angle = std::remainder(
      scanAngleMin_ + static_cast<float>(i) * scanAngleIncrement_, 2.0f * pi);
    float absAngle = std::fabs(angle);

    if (absAngle <= FRONT_HALF_ARC_RAD) {
      frontMin = std::min(frontMin, range);
      if (angle >= 0.0f) {
        frontLeftMin = std::min(frontLeftMin, range);
      } else {
        frontRightMin = std::min(frontRightMin, range);
      }
    } else if (absAngle >= pi - REAR_HALF_ARC_RAD) {
      rearMin = std::min(rearMin, range);
    } else if (angle > 0.0f) {
      leftSideMin = std::min(leftSideMin, range);
    } else {
      rightSideMin = std::min(rightSideMin, range);
    }
  }

  if (cmd.linear.x > 0.0) {
    if (frontMin < ASSIST_START_DIST_M) {
      // Closer obstacle -> stronger nudge. Scales from 0 at ASSIST_START_DIST_M
      // up to full strength at FORWARD_STOP_DIST_M.
      double closeness = (ASSIST_START_DIST_M - frontMin) /
        (ASSIST_START_DIST_M - FORWARD_STOP_DIST_M);
      closeness = std::clamp(closeness, 0.0, 1.0);
      double assistDir = (frontLeftMin >= frontRightMin) ? 1.0 : -1.0;
      double assistTurn = assistDir * ASSIST_MAX_ANGULAR * closeness;
      cmd.angular.z = std::clamp(cmd.angular.z + assistTurn, -MAX_ASSISTED_ANGULAR, MAX_ASSISTED_ANGULAR);
    }

    if (frontMin < FORWARD_STOP_DIST_M) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Collision guard: blocking forward motion, obstacle %.2fm ahead (lidar)", frontMin);
      cmd.linear.x = 0.0;
    }

    // Independent depth-camera check: catches low obstacles (roots, grass
    // clumps) and anything else outside the lidar's single scan plane.
    if (haveDepth_ && depthForwardMin_ < DEPTH_FORWARD_STOP_DIST_M) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Collision guard: blocking forward motion, obstacle %.2fm ahead (depth camera)", depthForwardMin_);
      cmd.linear.x = 0.0;
    }

    // Cliff/edge check: the ground itself is missing ahead (drop-off at
    // the edge of the mapped terrain, a pit, etc.) - neither the lidar nor
    // the obstacle-distance check above can ever catch this, since both
    // only look for something closer than expected, not the floor vanishing.
    if (haveDepth_ && cliffDetected_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Collision guard: blocking forward motion, no floor detected ahead (cliff/edge)");
      cmd.linear.x = 0.0;
    }
  } else if (cmd.linear.x < 0.0) {
    if (rearMin < REAR_STOP_DIST_M) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Collision guard: blocking reverse motion, obstacle %.2fm behind", rearMin);
      cmd.linear.x = 0.0;
    }
  }

  // Never turn further into a side that's already almost touching,
  // independent of whether the robot is also moving forward/backward.
  if (cmd.angular.z > 0.0 && leftSideMin < SIDE_STOP_DIST_M) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Collision guard: blocking turn, obstacle %.2fm on the left", leftSideMin);
    cmd.angular.z = 0.0;
  } else if (cmd.angular.z < 0.0 && rightSideMin < SIDE_STOP_DIST_M) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Collision guard: blocking turn, obstacle %.2fm on the right", rightSideMin);
    cmd.angular.z = 0.0;
  }

  return cmd;
}
