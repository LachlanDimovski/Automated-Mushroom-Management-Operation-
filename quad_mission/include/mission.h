// Top level quad state machine
// Call this for all your quad needs!
// Made and Upkept by Darcy Moyes - 25280025
// last patch was 22 September 26
// -- Quadcopter States --
// IDLE          = Motors are idle, here when waiting for a command
// TAKEOFF       = Start to move to a coasting height
// SEARCH        = Following path, looking for mushrooms, avoid obstacles -- the big one
// RETURN        = Returning home to start point - doesnt search in this mode
// LAND          = self explanatory - lands the quadcopter
// GROUNDED      = on the ground - can lead to takeoff
#ifndef QUAD_MISSION__MISSION_H_
#define QUAD_MISSION__MISSION_H_
 
#include <memory>
#include <optional>
#include <utility>
#include <vector>
 
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "quad_mission_interfaces/srv/start_search.hpp"
 
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
 
// Mission-level state machine, same split as A3's Mission/Controller:
// this node holds a shared_ptr to the Quadcopter node and drives it by
// calling its public setGoal()/isGoalReached()/setActive() interface
// directly (both nodes are spun together in the same executor in
// main.cpp), rather than talking to it over topics.
//
// Liftoff and land are just pose goals with a target z (see quadcopter.hpp
// for why — no dedicated flight-controller topic on this drone model).
//
// There's no separate FAILSAFE state: /mission/abort cancels the current
// goal, zeroes cmd_vel, and drops straight to GROUNDED, since GROUNDED is
// already "on the ground, can lead to takeoff" — the right place to sit
// after an aborted mission just as much as after a normal landing.
class Mission : public rclcpp::Node
{
public:
  explicit Mission(std::shared_ptr<Quadcopter> quad);
 
private:
  using Trigger = std_srvs::srv::Trigger;
  using StartSearch = quad_mission_interfaces::srv::StartSearch;
 
  void stateLoop();
 
  void handleLiftoff(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void handleStartSearch(
    const std::shared_ptr<StartSearch::Request> request,
    std::shared_ptr<StartSearch::Response> response);
  void handleReturnHome(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void handleLand(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void handleAbort(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
 
  // Path shapes. "rect"/"square" trace the search-area perimeter, "circle"
  // traces a circle inscribed in it, anything else (including empty/unknown)
  // falls back to the original full-coverage boustrophedon sweep.
  std::vector<std::pair<double, double>> generateShapePath(const std::string & shape);
  std::vector<std::pair<double, double>> generateBoustrophedonPath();
  std::vector<std::pair<double, double>> generateRectPath();
  std::vector<std::pair<double, double>> generateCirclePath(int numPoints = 24);
 
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
  rclcpp::Service<StartSearch>::SharedPtr startSearchSrv_;
  rclcpp::Service<Trigger>::SharedPtr returnHomeSrv_;
  rclcpp::Service<Trigger>::SharedPtr landSrv_;
  rclcpp::Service<Trigger>::SharedPtr abortSrv_;
};
 
#endif  // QUAD_MISSION__MISSION_H_
 