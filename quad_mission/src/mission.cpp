#include "mission.h"
#include <algorithm>
#include <cmath>
#include <chrono>
 
using namespace std::chrono_literals;
using namespace std::placeholders;
 
std::string missionStateToString(MissionState state) {
    switch (state) {
        case MissionState::IDLE: return "IDLE";
        case MissionState::TAKEOFF: return "TAKEOFF";
        case MissionState::SEARCH: return "SEARCH";
        case MissionState::RETURN: return "RETURN";
        case MissionState::LAND: return "LAND";
        case MissionState::GROUNDED: return "GROUNDED";
    }
    return "UNKNOWN";
}
 
Mission::Mission(std::shared_ptr<Quadcopter> quad)
: Node("mission"), quad_(quad), state_(MissionState::IDLE) {
 
    // Parameters — same search-area shape as the earlier boustrophedon nodes
    searchMinX_ = this->declare_parameter<double>("search_min_x", -5.0);
    searchMaxX_ = this->declare_parameter<double>("search_max_x", 5.0);
    searchMinY_ = this->declare_parameter<double>("search_min_y", -5.0);
    searchMaxY_ = this->declare_parameter<double>("search_max_y", 5.0);
    laneSpacing_ = this->declare_parameter<double>("lane_spacing", 1.0);
    takeoffAltitude_ = this->declare_parameter<double>("takeoff_altitude", 2.0);
 
    // External control services
    liftoffSrv_ = this->create_service<Trigger>(
        "/parrot1/mission/liftoff", std::bind(&Mission::handleLiftoff, this, _1, _2));
    startSearchSrv_ = this->create_service<StartSearch>(
        "/parrot1/mission/start_search", std::bind(&Mission::handleStartSearch, this, _1, _2));
    returnHomeSrv_ = this->create_service<Trigger>(
        "/parrot1/mission/return_home", std::bind(&Mission::handleReturnHome, this, _1, _2));
    landSrv_ = this->create_service<Trigger>(
        "/parrot1/mission/land", std::bind(&Mission::handleLand, this, _1, _2));
    abortSrv_ = this->create_service<Trigger>(
        "/parrot1/mission/abort", std::bind(&Mission::handleAbort, this, _1, _2));
 
    // State-check loop — lighter weight than Quadcopter's own 20Hz control
    // timer, since this just polls isGoalReached() and advances state.
    timer_ = this->create_wall_timer(200ms, std::bind(&Mission::stateLoop, this));
 
    RCLCPP_INFO(this->get_logger(), "Mission node ready in state %s",
        missionStateToString(state_).c_str());
}
 
void Mission::handleLiftoff(
    const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
    // Accepts from IDLE (first ever liftoff) or GROUNDED (relaunch after a
    // landing or an aborted mission).
    if (state_ != MissionState::IDLE && state_ != MissionState::GROUNDED) {
        response->success = false;
        response->message = "Cannot liftoff from state " + missionStateToString(state_);
        return;
    }
    // Capture current (x, y) as home before climbing.
    auto odom = quad_->getOdom();
    homeXY_ = {odom.pose.pose.position.x, odom.pose.pose.position.y};
 
    geometry_msgs::msg::Pose goal = makePose(
        odom.pose.pose.position.x, odom.pose.pose.position.y, takeoffAltitude_);
    quad_->setGoal(goal);
    quad_->setActive(true);
 
    RCLCPP_INFO(this->get_logger(), "Liftoff initiated, climbing to %.1fm.", takeoffAltitude_);
 
    state_ = MissionState::TAKEOFF;
    response->success = true;
    response->message = "Liftoff initiated";
}
 
void Mission::handleStartSearch(
    const std::shared_ptr<StartSearch::Request> request, std::shared_ptr<StartSearch::Response> response) {
    if (state_ != MissionState::TAKEOFF || !quad_->isGoalReached()) {
        response->success = false;
        response->message = "Cannot start search from state " + missionStateToString(state_) +
            (state_ == MissionState::TAKEOFF ? " (not yet at altitude)" : "");
        return;
    }
    waypoints_ = generateShapePath(request->shape);
    currentWpIdx_ = 0;
    state_ = MissionState::SEARCH;
    sendNextWaypoint();
    response->success = true;
    response->message = "Search started (shape='" + request->shape + "'), " +
        std::to_string(waypoints_.size()) + " waypoints";
}
 
void Mission::handleReturnHome(
    const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
    if (state_ != MissionState::SEARCH && state_ != MissionState::TAKEOFF) {
        response->success = false;
        response->message = "Cannot return home from state " + missionStateToString(state_);
        return;
    }
    if (!homeXY_.has_value()) {
        response->success = false;
        response->message = "No home position captured";
        return;
    }
    auto [hx, hy] = homeXY_.value();
    quad_->setGoal(makePose(hx, hy, takeoffAltitude_));
    state_ = MissionState::RETURN;
    response->success = true;
    response->message = "Returning home";
}
 
void Mission::handleLand(
    const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
    if (state_ != MissionState::RETURN || !quad_->isGoalReached()) {
        response->success = false;
        response->message = "Cannot land from state " + missionStateToString(state_) +
            (state_ == MissionState::RETURN ? " (not yet home)" : "");
        return;
    }
    auto [hx, hy] = homeXY_.value();
    quad_->setGoal(makePose(hx, hy, 0.0));
    state_ = MissionState::LAND;
    response->success = true;
    response->message = "Landing initiated";
}
 
void Mission::handleAbort(
    const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
    quad_->setActive(false);   // zeroes cmd_vel immediately, see Quadcopter::setActive
    state_ = MissionState::GROUNDED;
    response->success = true;
    response->message = "Aborted — dropped to GROUNDED";
}
 
void Mission::stateLoop() {
    switch (state_) {
        case MissionState::IDLE:
            break;  // waiting for /liftoff
        case MissionState::TAKEOFF:
            break;  // waiting for operator to call /start_search once at altitude
        case MissionState::SEARCH:
            if (quad_->isGoalReached()) {
                currentWpIdx_++;
                sendNextWaypoint();
            }
            break;
        case MissionState::RETURN:
            break;  // waiting for operator to call /land once isGoalReached()
        case MissionState::LAND:
            if (quad_->isGoalReached()) {
                quad_->setActive(false);
                state_ = MissionState::GROUNDED;
                RCLCPP_INFO(this->get_logger(), "Landed.");
            }
            break;
        case MissionState::GROUNDED:
            break;  // terminal until /mission/liftoff is called again
    }
}
 
void Mission::sendNextWaypoint() {
    if (currentWpIdx_ >= waypoints_.size()) {
        RCLCPP_INFO(this->get_logger(), "Search pattern complete.");
        state_ = MissionState::RETURN;
        if (homeXY_.has_value()) {
            auto [hx, hy] = homeXY_.value();
            quad_->setGoal(makePose(hx, hy, takeoffAltitude_));
        }
        return;
    }
    auto [x, y] = waypoints_[currentWpIdx_];
    quad_->setGoal(makePose(x, y, takeoffAltitude_));
}
 
std::vector<std::pair<double, double>> Mission::generateBoustrophedonPath() {
    std::vector<std::pair<double, double>> waypoints;
    bool goingRight = true;
    for (double y = searchMinY_; y <= searchMaxY_ + 1e-6; y += laneSpacing_) {
        if (goingRight) {
            waypoints.emplace_back(searchMinX_, y);
            waypoints.emplace_back(searchMaxX_, y);
        } else {
            waypoints.emplace_back(searchMaxX_, y);
            waypoints.emplace_back(searchMinX_, y);
        }
        goingRight = !goingRight;
    }
    return waypoints;
}
 
std::vector<std::pair<double, double>> Mission::generateShapePath(const std::string & shape) {
    if (shape == "rect" || shape == "square") {
        return generateRectPath();
    } else if (shape == "circle") {
        return generateCirclePath();
    }
    // Empty or unrecognised shape -> original full-coverage sweep. Keeps
    // old test commands (Trigger-style calls with no shape) working the
    // same way they always did.
    return generateBoustrophedonPath();
}
 
std::vector<std::pair<double, double>> Mission::generateRectPath() {
    // Perimeter loop around the search bounds, starting and ending at the
    // same corner so the shape closes visibly in the RViz trail.
    // "square" uses the same bounding box rather than forcing equal side
    // lengths — the search area is already whatever shape the operator/GUI
    // configured, so a "square" request just traces it without distorting it.
    std::vector<std::pair<double, double>> waypoints;
    waypoints.emplace_back(searchMinX_, searchMinY_);
    waypoints.emplace_back(searchMaxX_, searchMinY_);
    waypoints.emplace_back(searchMaxX_, searchMaxY_);
    waypoints.emplace_back(searchMinX_, searchMaxY_);
    waypoints.emplace_back(searchMinX_, searchMinY_);  // close the loop
    return waypoints;
}
 
std::vector<std::pair<double, double>> Mission::generateCirclePath(int numPoints) {
    // Circle inscribed in the search bounding box: centred on the box,
    // radius limited by the shorter dimension so it stays inside the
    // configured search area.
    double centreX = (searchMinX_ + searchMaxX_) / 2.0;
    double centreY = (searchMinY_ + searchMaxY_) / 2.0;
    double width = searchMaxX_ - searchMinX_;
    double height = searchMaxY_ - searchMinY_;
    double radius = std::min(width, height) / 2.0;
 
    std::vector<std::pair<double, double>> waypoints;
    for (int i = 0; i <= numPoints; ++i) {  // <= to close the loop back to start
        double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(numPoints);
        waypoints.emplace_back(centreX + radius * std::cos(angle), centreY + radius * std::sin(angle));
    }
    return waypoints;
}
 
geometry_msgs::msg::Pose Mission::makePose(double x, double y, double z) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = 1.0;  // no particular heading requested
    return pose;
}