#ifndef SKIDSTEER_CONTROLLER_COLLISION_GUARD_HPP
#define SKIDSTEER_CONTROLLER_COLLISION_GUARD_HPP

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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

  // Camera geometry (husky.urdf.xacro camera_joint: z=0.25, no tilt;
  // husky.gazebo.xacro rgbd_camera: horizontal_fov=2.0944 rad, 720x480).
  // Used to work out how far down the depth crop can look before its rows
  // are just staring at bare flat ground rather than a real obstacle - the
  // bottom row of the raw image hits the ground only ~0.2m out, which
  // would permanently read as "obstacle ahead" with no obstacle present.
  static constexpr double CAMERA_HEIGHT_M = 0.25;
  static constexpr double CAMERA_HORIZONTAL_FOV_RAD = 2.0944;
  static constexpr double CAMERA_ASPECT = 720.0 / 480.0;
  // Crop cutoff chosen so flat ground is never reported closer than this,
  // safely above DEPTH_FORWARD_STOP_DIST_M with margin.
  static constexpr double GROUND_SAFE_DIST_M = 2.0;
  // Sides only need to guard against scraping while turning - a much
  // tighter threshold than front/rear, since normal avoidance turns
  // routinely pass within a few metres of things on the flank on purpose.
  static constexpr double SIDE_STOP_DIST_M = 0.4;

  // Cliff/edge ("negative obstacle") detection. Every check above only
  // looks for something closer than expected - nothing checks for the
  // ground disappearing (a drop-off at the edge of the mapped terrain).
  // Sample a row whose ray should hit flat ground at CLIFF_CHECK_EXPECTED_DIST_M
  // under normal terrain; if that row instead returns no depth or a much
  // longer one, the floor isn't there and forward motion is blocked.
  static constexpr double CLIFF_CHECK_EXPECTED_DIST_M = 1.0;
  static constexpr double CLIFF_CHECK_TRIGGER_DIST_M = 1.6;  // measured further than this -> no floor

  // Steering assist: once something is close ahead, nudge the driver's turn
  // command towards whichever side is more open, rather than just stopping
  // dead and leaving the human to find a way around unaided.
  static constexpr double ASSIST_START_DIST_M = 2.0;
  static constexpr double ASSIST_MAX_ANGULAR = 1.2;
  static constexpr double MAX_ASSISTED_ANGULAR = 2.5;  // clamp after adding the assist nudge

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr guard_timer_;

  std::vector<float> scanRanges_;
  float scanAngleMin_ = 0.0f;
  float scanAngleIncrement_ = 0.0f;
  bool haveScan_ = false;

  float depthForwardMin_ = 0.0f;
  bool haveDepth_ = false;
  float cliffCheckDist_ = 0.0f;
  bool cliffDetected_ = false;

  geometry_msgs::msg::Twist lastRawCmd_;
  rclcpp::Time lastRawCmdStamp_;
  bool haveRawCmd_ = false;
};

#endif  // SKIDSTEER_CONTROLLER_COLLISION_GUARD_HPP
