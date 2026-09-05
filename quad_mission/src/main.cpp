#include "quadcopter.h"
#include "mission.h"
 
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
 
    auto quad = std::make_shared<Quadcopter>();
    auto mission = std::make_shared<Mission>(quad);
 
    // Both nodes spun together in one executor — Mission calls Quadcopter's
    // public interface directly (setGoal/isGoalReached/setActive), same
    // in-process pattern as A3's Controller + Mission split.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(quad);
    executor.add_node(mission);
    executor.spin();
 
    rclcpp::shutdown();
    return 0;
}
 