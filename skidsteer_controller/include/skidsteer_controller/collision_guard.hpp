#ifndef SKIDSTEER_CONTROLLER_COLLISION_GUARD_HPP
#define SKIDSTEER_CONTROLLER_COLLISION_GUARD_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include "skidsteer_controller/depth_check.hpp"

// Final safety gate between whatever is commanding the robot (teleop
// keyboard, the autonomous controller, anything else) and the actual
// husky1/cmd_vel that reaches the simulator. Applies regardless of source,
// so a human driver gets the same protection the autonomous patrol has.
class CollisionGuard : public rclcpp::Node
{
public:
  CollisionGuard();

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void hazardCallback(const std_msgs::msg::Bool::SharedPtr msg);
  bool humanBlocksForward() const;
  void navCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  bool depthBlocksForward(double vx);
  struct Sectors
  {
    float front, frontLeft, frontRight, rear, left, right;
  };
  Sectors scanSectors() const;
  float leftClear(const Sectors & s) const;
  float rightClear(const Sectors & s) const;
  void guardTimerCallback();
  geometry_msgs::msg::Twist applyGuard(geometry_msgs::msg::Twist cmd);

  // Re-check and republish on a timer, not only when a new raw command
  // arrives: teleop_twist_keyboard mostly sends on keypress, not
  // continuously, so without this an obstacle that appears mid-drive with
  // no new keypress would never get re-evaluated against the last command.
  static constexpr int GUARD_TIMER_MS = 50;
  // If no raw command has arrived in this long, stop rather than keep
  // replaying a stale command indefinitely.
  static constexpr double RAW_CMD_TIMEOUT_S = 0.5;

  // Full 360-degree hardcoded safety envelope. The scan is split into four
  // non-overlapping quadrants covering the entire circle - front, rear,
  // left, right - so nothing approaching from any direction is unguarded.
  // This applies to every command that reaches the robot (teleop,
  // autonomous patrol, visual servo, recovery, back-away - all of it),
  // regardless of whether the code that issued the command itself checked
  // for obstacles.
  static constexpr double FRONT_HALF_ARC_RAD = 0.9;  // ~52 deg either side of front
  static constexpr double REAR_HALF_ARC_RAD = 0.9;   // ~52 deg either side of rear

  static constexpr double FORWARD_STOP_DIST_M = 1.0;
  static constexpr double REAR_STOP_DIST_M = 1.0;
  // Second, independent sensing modality for the front-stop check only: the
  // 2D lidar only sees one fixed horizontal plane, so part of an obstacle
  // (a tree's root flare, a low branch, anything that bulges at a
  // different height) can sit entirely outside that plane and never
  // register at all. The depth camera sees the full vertical field of
  // view, so it can catch what the lidar's single slice misses.
  static constexpr double DEPTH_FORWARD_STOP_DIST_M = 1.0;

  // husky.gazebo.xacro rgbd_camera. The camera pose itself comes from TF
  // (base_frame <- the depth image's frame), so moving or re-tilting the
  // camera in the URDF needs no change here.
  static constexpr double CAMERA_HORIZONTAL_FOV_RAD = 2.0944;
  static constexpr double DEPTH_STALE_S = 1.0;  // older depth results are ignored, not trusted
  // A person the rover's detector has identified, from perception's
  // hazard_brake on husky1/hazard_stop. Forward motion is blocked while it is
  // true; turning and reversing are deliberately left alone, so the rover can
  // always steer or back out of a hazard stop instead of sitting in one.
  //
  // This gate is here and not only in hazard_brake because this node is the
  // single publisher of husky1/cmd_vel. hazard_brake cancels the Nav2 goal,
  // which is right for an autonomous run and does nothing at all for a rover
  // being hand-driven from the GUI's teleop panel - and a hand-driven rover is
  // exactly the one with a person in front of it and someone holding W down.
  //
  // Stale means clear: if hazard_brake dies or was never launched (hazard:=false,
  // which is the default), the rover must not be left unable to drive forward.
  static constexpr double HAZARD_STALE_S = 2.0;
  // Sides only need to guard against scraping while turning - a much
  // tighter threshold than front/rear, since normal avoidance turns
  // routinely pass within a few metres of things on the flank on purpose.
  static constexpr double SIDE_STOP_DIST_M = 0.4;

  // Cliff/edge ("negative obstacle") detection: see depth_check.hpp - rays
  // that should meet flat ground 1.2-1.8 m ahead mostly find no floor.

  // Steering assist: once something is close ahead, nudge the driver's turn
  // command towards whichever side is more open, rather than just stopping
  // dead and leaving the human to find a way around unaided.
  static constexpr double ASSIST_START_DIST_M = 2.0;
  static constexpr double ASSIST_MAX_ANGULAR = 1.2;
  static constexpr double MAX_ASSISTED_ANGULAR = 2.5;  // clamp after adding the assist nudge

  // Escape on the Nav2 path: when the depth check has held Nav2's forward command at zero this long, back out
  // toward the more open side (lidar flanks and the depth camera's view past the front corners) so Nav2 can
  // replan from somewhere it is not wedged, instead of timing out against the same spot.
  static constexpr double ESCAPE_AFTER_S = 2.0;
  static constexpr double BLOCK_GAP_S = 3.0;  // turns in place between depth stops don't reset the block timer
  static constexpr double ESCAPE_S = 2.0;
  static constexpr double ESCAPE_SPEED = 0.2;
  static constexpr double ESCAPE_TURN = 0.5;
  static constexpr int ESCAPE_MAX = 3;           // escapes allowed within ESCAPE_WINDOW_S, then Nav2's own recovery
  static constexpr double ESCAPE_WINDOW_S = 30.0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;  // depth obstacles for Nav2's costmap
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_nav_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hazard_sub_;
  rclcpp::TimerBase::SharedPtr guard_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string base_frame_;
  depth_check::Limits depth_limits_;
  double base_height_ = 0.1323;

  std::vector<float> scanRanges_;
  float scanAngleMin_ = 0.0f;
  float scanAngleIncrement_ = 0.0f;
  bool haveScan_ = false;

  float depthForwardMin_ = 0.0f;  // obstacle distance ahead of the robot's front
  bool haveDepth_ = false;
  rclcpp::Time depthStamp_;
  bool cliffDetected_ = false;
  float depthLeft_ = 0.0f, depthRight_ = 0.0f;  // flank gaps past the front corners (depth camera)

  bool hazardStop_ = false;      // a person is in the rover's way
  bool haveHazard_ = false;
  rclcpp::Time hazardStamp_;

  rclcpp::Time blockedSince_, lastBlocked_, escapeUntil_;
  bool blocked_ = false, escaping_ = false;
  double escapeTurn_ = 0.0;
  std::vector<rclcpp::Time> escapes_;

  geometry_msgs::msg::Twist lastRawCmd_;
  rclcpp::Time lastRawCmdStamp_;
  bool haveRawCmd_ = false;
};

#endif  // SKIDSTEER_CONTROLLER_COLLISION_GUARD_HPP
