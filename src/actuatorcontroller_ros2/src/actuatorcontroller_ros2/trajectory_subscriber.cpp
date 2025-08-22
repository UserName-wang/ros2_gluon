#include "actuatorcontroller_ros2/trajectory_subscriber.h"
#include <iostream>

TrajectorySubscriber::TrajectorySubscriber(std::shared_ptr<rclcpp::Node> node, ActuatorController* controller) 
    : nh_(node), actuator_controller_(controller) {
    
    // Initialize the actuator mapping
    initializeActuatorMapping();
    
    // Subscribe to the trajectory topic from MoveIt2
    trajectory_sub_ = nh_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/gluon_arm_controller/joint_trajectory", 10,
        std::bind(&TrajectorySubscriber::trajectoryCallback, this, std::placeholders::_1));
        
    // Publisher for joint states
    joint_state_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);
        
    // Timer for regular updates
    timer_ = nh_->create_wall_timer(
        std::chrono::milliseconds(100), // 10 Hz
        std::bind(&TrajectorySubscriber::timerCallback, this));
        
    RCLCPP_INFO(nh_->get_logger(), "TrajectorySubscriber initialized");
}

TrajectorySubscriber::~TrajectorySubscriber() {
    // Destructor
}

void TrajectorySubscriber::initializeActuatorMapping() {
    // Initialize the mapping between joint names and actuator IDs
    // This mapping needs to be customized based on your specific setup
    std::vector<uint8_t> actuatorIds = actuator_controller_->getActuatorIdArray();
    
    // Simple mapping - assumes actuators are in order
    if (actuatorIds.size() >= 6) {
        // Convert uint8_t IDs to strings for mapping
        std::vector<std::string> actuatorIdStrings;
        for (const auto& id : actuatorIds) {
            actuatorIdStrings.push_back(std::to_string(id));
        }
        
        joint_to_actuator_map_ = {
            {"axis_joint_1", actuatorIdStrings[0]},
            {"axis_joint_2", actuatorIdStrings[1]},
            {"axis_joint_3", actuatorIdStrings[2]},
            {"axis_joint_4", actuatorIdStrings[3]},
            {"axis_joint_5", actuatorIdStrings[4]},
            {"axis_joint_6", actuatorIdStrings[5]}
        };
        
        actuator_to_joint_map_ = {
            {actuatorIdStrings[0], "axis_joint_1"},
            {actuatorIdStrings[1], "axis_joint_2"},
            {actuatorIdStrings[2], "axis_joint_3"},
            {actuatorIdStrings[3], "axis_joint_4"},
            {actuatorIdStrings[4], "axis_joint_5"},
            {actuatorIdStrings[5], "axis_joint_6"}
        };
    } else {
        RCLCPP_WARN(nh_->get_logger(), "Not enough actuators found. Found %ld actuators.", actuatorIds.size());
    }
    
    // Print the mapping for verification
    for (const auto& pair : joint_to_actuator_map_) {
        RCLCPP_INFO(nh_->get_logger(), "Joint %s -> Actuator %s", 
                   pair.first.c_str(), pair.second.c_str());
    }
}

void TrajectorySubscriber::trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    RCLCPP_INFO(nh_->get_logger(), "Received trajectory with %ld points", msg->points.size());
    
    if (msg->points.empty()) {
        RCLCPP_WARN(nh_->get_logger(), "Received empty trajectory");
        return;
    }
    
    // Process each trajectory point
    for (const auto& point : msg->points) {
        // Send commands to actuators
        for (size_t i = 0; i < msg->joint_names.size() && i < point.positions.size(); i++) {
            std::string joint_name = msg->joint_names[i];
            
            // Check if we have a mapping for this joint
            if (joint_to_actuator_map_.find(joint_name) != joint_to_actuator_map_.end()) {
                std::string actuator_id_str = joint_to_actuator_map_[joint_name];
                uint8_t actuator_id = static_cast<uint8_t>(std::stoi(actuator_id_str));
                double position = point.positions[i];
                
                // Send position command to actuator
                actuator_controller_->setPosition(actuator_id, position);
                
                RCLCPP_DEBUG(nh_->get_logger(), "Set actuator %d (joint %s) to position %f", 
                            static_cast<int>(actuator_id), joint_name.c_str(), position);
            } else {
                RCLCPP_WARN(nh_->get_logger(), "No actuator mapping found for joint %s", joint_name.c_str());
            }
        }
        
        // Wait for the specified time (if needed)
        if (point.time_from_start.sec > 0 || point.time_from_start.nanosec > 0) {
            auto sleep_duration = std::chrono::seconds(point.time_from_start.sec) +
                                 std::chrono::nanoseconds(point.time_from_start.nanosec);
            std::this_thread::sleep_for(sleep_duration);
        }
    }
    
    RCLCPP_INFO(nh_->get_logger(), "Finished executing trajectory");
}

void TrajectorySubscriber::timerCallback() {
    // Publish current joint states
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = nh_->now();
    
    // Get current positions from actuators
    std::vector<uint8_t> actuatorIds = actuator_controller_->getActuatorIdArray();
    
    for (const auto& id : actuatorIds) {
        // Convert ID to string for mapping
        std::string id_str = std::to_string(id);
        
        // Check if we have a mapping for this actuator
        if (actuator_to_joint_map_.find(id_str) != actuator_to_joint_map_.end()) {
            std::string joint_name = actuator_to_joint_map_[id_str];
            // Get position from actuator (don't refresh to avoid blocking)
            double position = actuator_controller_->getPosition(id, false);
            
            joint_state_msg.name.push_back(joint_name);
            joint_state_msg.position.push_back(position);
            
            RCLCPP_DEBUG(nh_->get_logger(), "Actuator %d (joint %s): position %f", 
                        static_cast<int>(id), joint_name.c_str(), position);
        } else {
            RCLCPP_WARN(nh_->get_logger(), "No joint mapping found for actuator %d", static_cast<int>(id));
        }
    }
    
    if (!joint_state_msg.name.empty()) {
        joint_state_pub_->publish(joint_state_msg);
    }
}