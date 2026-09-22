// ros2 launch 41068_ignition_bringup 41068_ignition_parrot.launch.py slam:=true nav2:=true rviz:=true world:=large_demo

#include "quadcopter.h"
#include <algorithm>
#include <cmath>
#include <chrono>
 
using namespace std::chrono_literals;
 
Quadcopter::Quadcopter() : Node("quadcopter"), active_(false), goalSet_(false) {
    // Publishers
    cmdVel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/parrot1/cmd_vel", 10);
        // create the publisher attached to this node
        // called in constructor before anything else runs
        // Twist is the type being published for linear and angular
        // publishes to the parrot1 cmd_vel topic (this project's drone, not sjtu_drone)
        // Number at the end is the queue size, so 10 loads 10 messages, which is good for slow processing
 
    // Flight trail for RViz — accumulated in odomCallback, drawn as a Path
    // display pointed at /parrot1/trail (Add -> By display type -> Path).
    trail_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/parrot1/trail", 10);
    trail_.header.frame_id = "parrot1_odom";
 
    // NOTE: no takeoff_publisher_/land_publisher_ here — see the header comment.
    // The Parrot has no arm/disarm flight-controller topic like sjtu_drone did;
    // liftoff/land are handled as z-axis pose goals through setGoal() instead,
    // driven by the Mission node.
 
    // Subscribers
    // NOTE: deliberately /parrot1/odometry, not /parrot1/odom. /odom is the
    // robot_localization EKF-fused output, and the EKF is configured in 2D
    // mode — it zeroes z regardless of what the raw sensor data contains
    // (see the comment on the OdometryPublisher plugin in
    // parrot.gazebo.xacro: "3D so odometry carries altitude (z); the EKF
    // ignores z either way"). /parrot1/odometry is the raw bridge from
    // Gazebo's ignition-gazebo-odometry-publisher-system plugin
    // (gazebo_bridge_parrot1.yaml), configured with <dimensions>3</dimensions>
    // — this is the one that actually reports real altitude.
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/parrot1/odometry", 10, std::bind(&Quadcopter::odomCallback, this, std::placeholders::_1));
    // similar to the publisher but has the additional callback binding to tell ROS what
    // function to call when it arrives — so when a new odometry message arrives, call
    // odomCallback and move the message to it.
 
    // Timer for 50ms (20Hz) control loop
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Quadcopter::timerCallback, this));
 
    // Control loop / status logging thread
    thread_ = new std::thread(&Quadcopter::threadFunction, this);
}
 
Quadcopter::~Quadcopter() {
    thread_->join();
}
 
void Quadcopter::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::unique_lock<std::mutex> lck(odomMtx_);
    odom_ = *msg;  // dereferenced pointer so it is the actual message object
    lck.unlock();
 
    // Trail: only record a new point if we've moved far enough from the
    // last one (keeps the Path from being pointlessly dense while hovering
    // or moving slowly), and cap total points so the message doesn't grow
    // unbounded over a long mission.
    bool should_record = trail_.poses.empty();
    if (!should_record) {
        const auto & last = trail_.poses.back().pose.position;
        double dx = msg->pose.pose.position.x - last.x;
        double dy = msg->pose.pose.position.y - last.y;
        double dz = msg->pose.pose.position.z - last.z;
        should_record = std::sqrt(dx * dx + dy * dy + dz * dz) > TRAIL_MIN_SPACING;
    }
    if (should_record) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = msg->header;
        ps.pose = msg->pose.pose;
        trail_.poses.push_back(ps);
        if (trail_.poses.size() > TRAIL_MAX_POINTS) {
            trail_.poses.erase(trail_.poses.begin());
        }
        trail_.header.stamp = msg->header.stamp;
        trail_publisher_->publish(trail_);
    }
}
 
void Quadcopter::setGoal(geometry_msgs::msg::Pose goal) {
    std::unique_lock<std::mutex> lck(goalMtx_);
    goal_ = goal;
    goalSet_ = true;
}
 
void Quadcopter::setActive(bool active) {
    active_ = active;
    if (!active) {
        // Stops the drone immediately
        geometry_msgs::msg::Twist stop;
        cmdVel_publisher_->publish(stop);
    }
}
 
nav_msgs::msg::Odometry Quadcopter::getOdom() {
    std::unique_lock<std::mutex> lck(odomMtx_);
    return odom_;
}
 
bool Quadcopter::isGoalReached() {
    std::unique_lock<std::mutex> odomLck(odomMtx_);
    nav_msgs::msg::Odometry odom = odom_;
    odomLck.unlock();
    std::unique_lock<std::mutex> goalLck(goalMtx_);
    geometry_msgs::msg::Pose goal = goal_;
    goalLck.unlock();
    if (!goalSet_) {
        return false;
    }
 
    double dx = goal.position.x - odom.pose.pose.position.x;
    double dy = goal.position.y - odom.pose.pose.position.y;
    double dz = goal.position.z - odom.pose.pose.position.z;
    // Real 3D distance — /parrot1/odometry (unlike /parrot1/odom) carries
    // genuine altitude, so liftoff/land goals now converge properly instead
    // of needing the horizontal-only workaround from before that topic was
    // found. See the odom_subscriber_ comment in the constructor for why.
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    return dist < GOAL_TOLERANCE;
}
 
geometry_msgs::msg::Twist Quadcopter::computeVelocity() {
    std::unique_lock<std::mutex> odomLck(odomMtx_);
    nav_msgs::msg::Odometry odom = odom_;
    odomLck.unlock();
    std::unique_lock<std::mutex> goalLck(goalMtx_);
    geometry_msgs::msg::Pose goal = goal_;
    goalLck.unlock();
    geometry_msgs::msg::Twist cmd;
 
    // Error in world frame
    double dx = goal.position.x - odom.pose.pose.position.x;
    double dy = goal.position.y - odom.pose.pose.position.y;
    double dz = goal.position.z - odom.pose.pose.position.z;
 
    // yaw from quaternion
    auto q = odom.pose.pose.orientation;
    double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
 
    // Rotate error into drone body frame
    double forward = dx * cos(yaw) + dy * sin(yaw);
    double lateral = -dx * sin(yaw) + dy * cos(yaw);
 
    // Proportional control
    const double KP = 0.5;
    cmd.linear.x = std::clamp(KP * forward, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    cmd.linear.y = std::clamp(KP * lateral, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    cmd.linear.z = std::clamp(KP * dz, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
 
    // Yaw control to point towards the goal — but atan2(dy, dx) is
    // meaningless/noisy when the horizontal distance is near zero (e.g.
    // liftoff/land goals, which only change z). Skip yaw correction
    // entirely in that case rather than spinning on garbage angles.
    double horizontal_dist = std::sqrt(dx * dx + dy * dy);
    if (horizontal_dist > 0.1) {
        double angle_to_goal = atan2(dy, dx);
        double yaw_error = angle_to_goal - yaw;
        while (yaw_error > M_PI) {
            yaw_error -= 2 * M_PI;
        }
        while (yaw_error < -M_PI) {
            yaw_error += 2 * M_PI;
        }
        cmd.angular.z = std::clamp(KP * yaw_error, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
    } else {
        cmd.angular.z = 0.0;
    }
    return cmd;
}
 
void Quadcopter::timerCallback() {
    if (!active_ || !goalSet_) {
        return;
    }
 
    if (isGoalReached()) {
        // stop at goal
        geometry_msgs::msg::Twist stop;
        cmdVel_publisher_->publish(stop);
        return;
    }
    geometry_msgs::msg::Twist cmd = computeVelocity();
    cmdVel_publisher_->publish(cmd);
}
 
void Quadcopter::threadFunction() {
    rclcpp::Rate rate(10.0);  // 10Hz
    while (rclcpp::ok()) {
        if (active_) {
            std::unique_lock<std::mutex> lck(odomMtx_);
            RCLCPP_DEBUG(this->get_logger(), "Quadcopter active, position: [%.2f, %.2f, %.2f]",
                odom_.pose.pose.position.x,
                odom_.pose.pose.position.y,
                odom_.pose.pose.position.z);
        }
        rate.sleep();
    }
}
 