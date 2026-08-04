#ifndef QUADCOPTER_H
#define QUADCOPTER_H
//put a guard in to prevent errors thinking there are multiple calls

#include <thread>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"

/*!
 * @brief This is the Quadcopter node to handle movement and control for the drone
 * Subscribes to odometry. Publishes cmd_vel, takeoff and landing. 
 */
class Quadcopter : public rclcpp::Node {
  
  public:

    /*! 
    * @brief Quadcopter constructor
    * @details Initialises publishers, subscribers, timer and control thread
    */
    Quadcopter();

    /*!
     * @brief Quadcopter destructor
     * @details Joining threads
     */
    ~Quadcopter();

    /*!
    * @brief Set the goal for the quadcopter to fly to
    * @details thread safe, should work without impacting other threads
    * @param[in] goal - goal pose in world coordinates  
    */
    void setGoal(geometry_msgs::msg::Pose goal);

    /*!
    * @brief command for drone to take off
    */
    void takeoff();

    /*!
    * @brief command to land and stop movement
    */
    void land();

    /*!
    * @brief Get current odometry
    * @return current odometry in the custom variable type
    */
   nav_msgs::msg::Odometry getOdom();

   /*!
   * @brief checking if goal is reached
   * @return true if goal is reached
   */
  bool isGoalReached();

   /*! @brief enable or disable control loop
   *  @param[in] active true to enable, false to disable
   */
  void setActive(bool active);

private:

  /*!
  * @brief odometry callback
  * @param[in] msg odometry message
  */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /*!
  * @brief Timer callback to run control loop at the same rate
  */
  void timerCallback();

  /*!
  * @brief control thread function
  * @details computes and publishes velocity commands to reach goal
  */
  void threadFunction();

  /*!
  * @brief compute velocity command to reach goal
  * @return twist message with velocity commands
  */
  geometry_msgs::msg::Twist computeVelocity();

  private:
  //Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoff_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr land_publisher_;

  //Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  //Timer
    rclcpp::TimerBase::SharedPtr timer_;

  //Thread
  std::thread* thread_;

  //Data 
  nav_msgs::msg::Odometry odom_; //current odometry
  geometry_msgs::msg::Pose goal_; //current goal pose

  //Mutexes
  std::mutex odomMtx_; //mutex for odom
  std::mutex goalMtx_; // mutex for goal

  //State 
  bool active_; //whether control is set
  bool goalSet_; // whether a goal has been set

  //Control parameters
  const double GOAL_TOLERANCE = 0.5;  //distance to consider goal is reached
  const double MAX_LINEAR_VEL = 1.0; //max linaer velocity (m/s)
  const double MAX_ANGULAR_VEL = 1.0; // max angular velocity (rad/s)
};

#endif // QUADCOPTER_H
