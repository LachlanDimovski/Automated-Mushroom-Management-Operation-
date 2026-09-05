//Top level quad state machine
// Call this for all your quad needs!
// Made and Upkept by Darcy Moyes - 25280025
// last patch was 4 September 26

// -- Quadcopter States --
// IDLE          = Motors are idle, here when waiting for a command
// TAKEOFF       = Start to move to a coasting height
// SEARCH        = Following path, looking for mushrooms, avoid obstacles -- the big one
// RETURN        = Returning home to start point - doesnt search in this mode
// LAND          = self explanatory - lands the quadcopter
// GROUNDED       = on the ground - can lead to takeoff


#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

//provided libraries
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

enum class MissionState { //refer to above for state meanings
    IDLE,
    TAKEOFF,
    SEARCH,
    RETURN,
    LAND,
    GROUNDED
};

std::string to_string(MissionState state) {
    switch(state){
        case MissionState::IDLE:
            return "IDLE";
        case MissionState::TAKEOFF:
            return "TAKEOFF";
        case MissionState::SEARCH:
            return "SEARCH";
        case MissionState::RETURN:
            return "RETURN";
        case MissionState::LAND:
            return "LAND";
        case MissionState::GROUNDED:
            return "GROUNDED";
    }
}

