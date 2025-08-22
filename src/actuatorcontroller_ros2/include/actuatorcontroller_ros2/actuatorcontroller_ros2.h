#ifndef ACTUATORCONTROLLER_ROS2_H
#define ACTUATORCONTROLLER_ROS2_H

#include "rclcpp/rclcpp.hpp"

// Actuator SDK
#include "actuatorcontroller.h"
#include "actuatordefine.h"

// standard message type
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// custom message type
#include "actuatorcontroller_ros2/msg/actuator_attribute.hpp"
#include "actuatorcontroller_ros2/msg/actuator_command.hpp"
#include "actuatorcontroller_ros2/msg/actuator_modes.hpp"
#include "actuatorcontroller_ros2/msg/actuator_array.hpp"

// custom service type
#include "actuatorcontroller_ros2/srv/attribute_lookup.hpp"
#include "actuatorcontroller_ros2/srv/attribute_query.hpp"
#include "actuatorcontroller_ros2/srv/general_query.hpp"
#include "actuatorcontroller_ros2/srv/attribute_dictionary.hpp"
#include "actuatorcontroller_ros2/srv/debug_query.hpp"
#include "actuatorcontroller_ros2/srv/trivia_query.hpp"
#include "actuatorcontroller_ros2/srv/id_modify.hpp"
#include "actuatorcontroller_ros2/srv/parameters_save.hpp"
#include "actuatorcontroller_ros2/srv/zero_reset.hpp"

// std stuff
#include <map>
#include <string>

class ActuatorController_ROS2 {
public:
    ActuatorController_ROS2(std::shared_ptr<rclcpp::Node> node);

    ~ActuatorController_ROS2();

private:
    std::shared_ptr<rclcpp::Node> nh_;
    ActuatorController* m_pController;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_pubJointState;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subEnableActuator;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subDisableActuator;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_subTargetPosition;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_subTargetVelocity;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_subTargetCurrent;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_subControlMode;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_subJointState;

    // Services
    rclcpp::Service<actuatorcontroller_ros2::srv::AttributeQuery>::SharedPtr m_serAttributeQuery;
    rclcpp::Service<actuatorcontroller_ros2::srv::GeneralQuery>::SharedPtr m_serGeneralQuery;
    rclcpp::Service<actuatorcontroller_ros2::srv::TriviaQuery>::SharedPtr m_serTriviaQuery;
    rclcpp::Service<actuatorcontroller_ros2::srv::DebugQuery>::SharedPtr m_serDebugQuery;
    rclcpp::Service<actuatorcontroller_ros2::srv::AttributeDictionary>::SharedPtr m_serAttributeDictionary;
    rclcpp::Service<actuatorcontroller_ros2::srv::IDModify>::SharedPtr m_serIDChange;
    rclcpp::Service<actuatorcontroller_ros2::srv::ParametersSave>::SharedPtr m_serParametersSave;
    rclcpp::Service<actuatorcontroller_ros2::srv::ZeroReset>::SharedPtr m_mZeroReset;

    // Callback methods
    void subscribeEnableActuator(const std_msgs::msg::Bool::SharedPtr msg);
    void subscribeDisableActuator(const std_msgs::msg::Bool::SharedPtr msg);
    void subscribeSetTargetPosition(const std_msgs::msg::Float64::SharedPtr msg);
    void subscribeSetTargetVelocity(const std_msgs::msg::Float64::SharedPtr msg);
    void subscribeSetTargetCurrent(const std_msgs::msg::Float64::SharedPtr msg);
    void subscribeSetControlMode(const std_msgs::msg::Int8::SharedPtr msg);
    void subscribeJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Service callbacks
    void serviceAttributeQuery(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<actuatorcontroller_ros2::srv::AttributeQuery::Request> req,
        std::shared_ptr<actuatorcontroller_ros2::srv::AttributeQuery::Response> res);
        
    void serviceGeneralQuery(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<actuatorcontroller_ros2::srv::GeneralQuery::Request> req,
        std::shared_ptr<actuatorcontroller_ros2::srv::GeneralQuery::Response> res);
        
    void serviceTriviaQuery(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<actuatorcontroller_ros2::srv::TriviaQuery::Request> req,
        std::shared_ptr<actuatorcontroller_ros2::srv::TriviaQuery::Response> res);
        
    void serviceDebugQuery(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<actuatorcontroller_ros2::srv::DebugQuery::Request> req,
        std::shared_ptr<actuatorcontroller_ros2::srv::DebugQuery::Response> res);
        
    void serviceAttributeDictionary(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<actuatorcontroller_ros2::srv::AttributeDictionary::Request> req,
        std::shared_ptr<actuatorcontroller_ros2::srv::AttributeDictionary::Response> res);
        
    void serviceIDModify(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<actuatorcontroller_ros2::srv::IDModify::Request> req,
        std::shared_ptr<actuatorcontroller_ros2::srv::IDModify::Response> res);
        
    void serviceParameterSave(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<actuatorcontroller_ros2::srv::ParametersSave::Request> req,
        std::shared_ptr<actuatorcontroller_ros2::srv::ParametersSave::Response> res);
        
    void serviceZeroReset(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<actuatorcontroller_ros2::srv::ZeroReset::Request> req,
        std::shared_ptr<actuatorcontroller_ros2::srv::ZeroReset::Response> res);

public:
    void releaseJointStates();
    void updateROSParam();
    std::string m_sINNFOS;
    std::string m_sActuator;
};

#endif // ACTUATORCONTROLLER_ROS2_H