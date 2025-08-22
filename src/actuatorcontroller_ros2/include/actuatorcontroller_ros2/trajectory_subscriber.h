#ifndef TRAJECTORY_SUBSCRIBER_H
#define TRAJECTORY_SUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "actuatorcontroller.h"
#include <map>
#include <string>
#include <vector>

class TrajectorySubscriber {
public:
    TrajectorySubscriber(std::shared_ptr<rclcpp::Node> node, ActuatorController* controller);
    ~TrajectorySubscriber();

private:
    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void timerCallback();
    void initializeActuatorMapping();
    
    std::shared_ptr<rclcpp::Node> nh_;
    ActuatorController* actuator_controller_;
    
    // Subscriber for joint trajectory from MoveIt2
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    
    // Publisher for joint states
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    
    // Timer for regular updates
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Maps between joint names and actuator IDs
    std::map<std::string, std::string> joint_to_actuator_map_;
    std::map<std::string, std::string> actuator_to_joint_map_;
    
    // Joint names for the Gluon robot
    std::vector<std::string> joint_names_ = {
        "axis_joint_1",
        "axis_joint_2", 
        "axis_joint_3",
        "axis_joint_4",
        "axis_joint_5",
        "axis_joint_6"
    };
};

#endif // TRAJECTORY_SUBSCRIBER_H