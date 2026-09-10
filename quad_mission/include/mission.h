// Top level quad state machine
// Call this for all your quad needs!
// Made and Upkept by Darcy Moyes - 25280025
// last patch was 6 September 26
// -- Quadcopter States --
// IDLE          = Motors are idle, here when waiting for a command
// TAKEOFF       = Start to move to a coasting height
// SEARCH        = Following path, looking for mushrooms, avoid obstacles -- the big one
// RETURN        = Returning home to start point - doesnt search in this mode
// LAND          = self explanatory - lands the quadcopter
// GROUNDED      = on the ground - can lead to takeoff
#ifndef QUADCOPTER_MISSION__MISSION_HPP_
#define QUADCOPTER_MISSION__MISSION_HPP_
 
#include <memory>
#include <optional>
#include <utility>
#include <vector>
 
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/trigger.hpp"
 
#include "quadcopter.h"
 
enum class MissionState {  // refer to header comment above for state meanings
  IDLE,
  TAKEOFF,
  SEARCH,
  RETURN,
  LAND,
  GROUNDED
};
 
std::string missionStateToString(MissionState state);
 

class Mission : public rclcpp::Node
{
public:
  explicit Mission(std::shared_ptr<Quadcopter> quad);
 
private:
  using Trigger = std_srvs::srv::Trigger;
 
  void stateLoop();
 
  void handleLiftoff(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void handleStartSearch(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void handleReturnHome(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void handleLand(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void handleAbort(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
 
  std::vector<std::pair<double, double>> generateBoustrophedonPath();
  geometry_msgs::msg::Pose makePose(double x, double y, double z);
  void sendNextWaypoint();
 
  std::shared_ptr<Quadcopter> quad_;
  MissionState state_;
 
  std::vector<std::pair<double, double>> waypoints_;
  size_t currentWpIdx_ {0};
  std::optional<std::pair<double, double>> homeXY_;
 
  // Parameters
  double searchMinX_, searchMaxX_, searchMinY_, searchMaxY_, laneSpacing_;
  double takeoffAltitude_;
 
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<Trigger>::SharedPtr liftoffSrv_;
  rclcpp::Service<Trigger>::SharedPtr startSearchSrv_;
  rclcpp::Service<Trigger>::SharedPtr returnHomeSrv_;
  rclcpp::Service<Trigger>::SharedPtr landSrv_;
  rclcpp::Service<Trigger>::SharedPtr abortSrv_;
};
 
#endif  // QUADCOPTER_MISSION__MISSION_HPP_
 