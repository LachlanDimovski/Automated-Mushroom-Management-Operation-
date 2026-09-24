#include "skidsteer_controller/collision_guard.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

CollisionGuard::CollisionGuard()
: Node("collision_guard")
{
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("husky1/cmd_vel", 10);
  obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("husky1/depth_obstacles", 10);
  cmd_vel_raw_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "husky1/cmd_vel_raw", 10,
    std::bind(&CollisionGuard::cmdCallback, this, std::placeholders::_1));
  // Nav2's velocity smoother output (cage_mission.launch.py remaps it here).
  // Only the depth checks apply to it: Nav2's own costmap already keeps it off
  // lidar obstacles, and the lidar stop/steering assist would fight its path
  // follower between trunks. Depth covers what that costmap cannot see.
  cmd_vel_nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "husky1/cmd_vel_nav_guarded", 10,
    std::bind(&CollisionGuard::navCallback, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "husky1/scan", 10,
    std::bind(&CollisionGuard::scanCallback, this, std::placeholders::_1));
  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "husky1/camera/depth/image", 10,
    std::bind(&CollisionGuard::depthCallback, this, std::placeholders::_1));
  base_frame_ = declare_parameter("base_frame", "husky1_base_link");
  // wheel axle z (0.0328) - wheel radius (0.1651) in husky.urdf.xacro
  base_height_ = declare_parameter("base_height_m", 0.1323);
  // calibration knob: rougher ground than the generated forests may need this raised
  depth_limits_.min_obstacle_height = declare_parameter("min_obstacle_height_m", depth_limits_.min_obstacle_height);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

// Depth image is 32-bit float metres per pixel ("32FC1"), read directly
// rather than via cv_bridge. The lidar's single plane misses low obstacles
// (logs, roots) - the depth camera covers them. Pixels are projected into
// base_link with the camera pose from TF, so flat ground is height ~0
// wherever the camera is mounted or tilted (see depth_check.hpp).
void CollisionGuard::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg->encoding != "32FC1" || msg->height == 0 || msg->width == 0) {
    return;
  }
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Collision guard: depth check off, no TF %s <- %s (%s)", base_frame_.c_str(),
      msg->header.frame_id.c_str(), e.what());
    return;
  }
  const auto & q = tf.transform.rotation;
  const auto & p = tf.transform.translation;
  tf2::Matrix3x3 m(tf2::Quaternion(q.x, q.y, q.z, q.w));
  depth_check::Geometry g{
    {m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]},
    {p.x, p.y, p.z}, CAMERA_HORIZONTAL_FOV_RAD, base_height_};

  auto r = depth_check::check(msg->data.data(), msg->width, msg->height, msg->step, g, depth_limits_);
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = msg->header.stamp;
  cloud.header.frame_id = base_frame_;
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(r.points.size());
  sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
  for (const auto & pt : r.points) {
    *ix = pt[0]; *iy = pt[1]; *iz = pt[2];
    ++ix; ++iy; ++iz;
  }
  obstacle_pub_->publish(cloud);

  depthForwardMin_ = r.obstacle_dist;
  cliffDetected_ = r.cliff;
  depthStamp_ = now();
  haveDepth_ = true;
}

bool CollisionGuard::depthBlocksForward(double vx)
{
  if (vx <= 0.0 || !haveDepth_ || (now() - depthStamp_).seconds() >= DEPTH_STALE_S) {
    return false;
  }
  if (depthForwardMin_ < DEPTH_FORWARD_STOP_DIST_M) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Collision guard: blocking forward motion, obstacle %.2fm ahead (depth camera)", depthForwardMin_);
    return true;
  }
  if (cliffDetected_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Collision guard: blocking forward motion, no floor detected ahead (cliff/edge)");
    return true;
  }
  return false;
}

void CollisionGuard::navCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  geometry_msgs::msg::Twist cmd = *msg;
  if (depthBlocksForward(cmd.linear.x)) {
    cmd.linear.x = 0.0;
  }
  cmd_vel_pub_->publish(cmd);
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

    // Independent depth-camera check: low obstacles (logs, roots) under the
    // lidar plane, and the floor vanishing ahead.
    if (depthBlocksForward(cmd.linear.x)) {
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
