#include "actuatorcontroller_ros2/actuatorcontroller_ros2.h"
#include <iostream>

ActuatorController_ROS2::ActuatorController_ROS2(std::shared_ptr<rclcpp::Node> node) : nh_(node) {
    m_pController = ActuatorController::getInstance();

    m_sINNFOS = "/INNFOS";
    m_sActuator = "/Actuator/";

    // Publishers
    m_pubJointState = nh_->create_publisher<sensor_msgs::msg::JointState>("/INNFOS/actuator_states", 10);

    // Subscribers
    m_subEnableActuator = nh_->create_subscription<std_msgs::msg::Bool>(
        "/INNFOS/enableActuator", 10,
        std::bind(&ActuatorController_ROS2::subscribeEnableActuator, this, std::placeholders::_1));

    m_subDisableActuator = nh_->create_subscription<std_msgs::msg::Bool>(
        "/INNFOS/disableActuator", 10,
        std::bind(&ActuatorController_ROS2::subscribeDisableActuator, this, std::placeholders::_1));

    m_subTargetPosition = nh_->create_subscription<std_msgs::msg::Float64>(
        "/INNFOS/setTargetPosition", 10,
        std::bind(&ActuatorController_ROS2::subscribeSetTargetPosition, this, std::placeholders::_1));

    m_subTargetVelocity = nh_->create_subscription<std_msgs::msg::Float64>(
        "/INNFOS/setTargetVelocity", 10,
        std::bind(&ActuatorController_ROS2::subscribeSetTargetVelocity, this, std::placeholders::_1));

    m_subTargetCurrent = nh_->create_subscription<std_msgs::msg::Float64>(
        "/INNFOS/setTargetCurrent", 10,
        std::bind(&ActuatorController_ROS2::subscribeSetTargetCurrent, this, std::placeholders::_1));

    m_subControlMode = nh_->create_subscription<std_msgs::msg::Int8>(
        "/INNFOS/setControlMode", 10,
        std::bind(&ActuatorController_ROS2::subscribeSetControlMode, this, std::placeholders::_1));

    m_subJointState = nh_->create_subscription<sensor_msgs::msg::JointState>(
        "/INNFOS/actuator_targets", 10,
        std::bind(&ActuatorController_ROS2::subscribeJointState, this, std::placeholders::_1));

    // Services
    m_serAttributeQuery = nh_->create_service<actuatorcontroller_ros2::srv::AttributeQuery>(
        "/INNFOS/AttributeQuery",
        std::bind(&ActuatorController_ROS2::serviceAttributeQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    m_serGeneralQuery = nh_->create_service<actuatorcontroller_ros2::srv::GeneralQuery>(
        "/INNFOS/GeneralQuery",
        std::bind(&ActuatorController_ROS2::serviceGeneralQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    m_serTriviaQuery = nh_->create_service<actuatorcontroller_ros2::srv::TriviaQuery>(
        "/INNFOS/TriviaQuery",
        std::bind(&ActuatorController_ROS2::serviceTriviaQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    m_serDebugQuery = nh_->create_service<actuatorcontroller_ros2::srv::DebugQuery>(
        "/INNFOS/DebugQuery",
        std::bind(&ActuatorController_ROS2::serviceDebugQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    m_serAttributeDictionary = nh_->create_service<actuatorcontroller_ros2::srv::AttributeDictionary>(
        "/INNFOS/Dictionary",
        std::bind(&ActuatorController_ROS2::serviceAttributeDictionary, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    m_serIDChange = nh_->create_service<actuatorcontroller_ros2::srv::IDModify>(
        "/INNFOS/IDChange",
        std::bind(&ActuatorController_ROS2::serviceIDModify, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    m_serParametersSave = nh_->create_service<actuatorcontroller_ros2::srv::ParametersSave>(
        "/INNFOS/ParametersSave",
        std::bind(&ActuatorController_ROS2::serviceParameterSave, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    m_mZeroReset = nh_->create_service<actuatorcontroller_ros2::srv::ZeroReset>(
        "/INNFOS/ZeroReset",
        std::bind(&ActuatorController_ROS2::serviceZeroReset, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

ActuatorController_ROS2::~ActuatorController_ROS2() {
    // Destructor
}

// Subscriber callbacks
void ActuatorController_ROS2::subscribeEnableActuator(const std_msgs::msg::Bool::SharedPtr msg) {
    // Implementation needed
}

void ActuatorController_ROS2::subscribeDisableActuator(const std_msgs::msg::Bool::SharedPtr msg) {
    // Implementation needed
}

void ActuatorController_ROS2::subscribeSetTargetPosition(const std_msgs::msg::Float64::SharedPtr msg) {
    // Implementation needed
}

void ActuatorController_ROS2::subscribeSetTargetVelocity(const std_msgs::msg::Float64::SharedPtr msg) {
    // Implementation needed
}

void ActuatorController_ROS2::subscribeSetTargetCurrent(const std_msgs::msg::Float64::SharedPtr msg) {
    // Implementation needed
}

void ActuatorController_ROS2::subscribeSetControlMode(const std_msgs::msg::Int8::SharedPtr msg) {
    // Implementation needed
}

void ActuatorController_ROS2::subscribeJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Implementation needed
}

// Service callbacks
void ActuatorController_ROS2::serviceAttributeQuery(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<actuatorcontroller_ros2::srv::AttributeQuery::Request> req,
    std::shared_ptr<actuatorcontroller_ros2::srv::AttributeQuery::Response> res) {
    // Implementation needed
}

void ActuatorController_ROS2::serviceGeneralQuery(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<actuatorcontroller_ros2::srv::GeneralQuery::Request> req,
    std::shared_ptr<actuatorcontroller_ros2::srv::GeneralQuery::Response> res) {
    // Implementation needed
}

void ActuatorController_ROS2::serviceTriviaQuery(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<actuatorcontroller_ros2::srv::TriviaQuery::Request> req,
    std::shared_ptr<actuatorcontroller_ros2::srv::TriviaQuery::Response> res) {
    // Implementation needed
}

void ActuatorController_ROS2::serviceDebugQuery(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<actuatorcontroller_ros2::srv::DebugQuery::Request> req,
    std::shared_ptr<actuatorcontroller_ros2::srv::DebugQuery::Response> res) {
    // Implementation needed
}

void ActuatorController_ROS2::serviceAttributeDictionary(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<actuatorcontroller_ros2::srv::AttributeDictionary::Request> req,
    std::shared_ptr<actuatorcontroller_ros2::srv::AttributeDictionary::Response> res) {
    // Implementation needed
}

void ActuatorController_ROS2::serviceIDModify(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<actuatorcontroller_ros2::srv::IDModify::Request> req,
    std::shared_ptr<actuatorcontroller_ros2::srv::IDModify::Response> res) {
    // Implementation needed
}

void ActuatorController_ROS2::serviceParameterSave(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<actuatorcontroller_ros2::srv::ParametersSave::Request> req,
    std::shared_ptr<actuatorcontroller_ros2::srv::ParametersSave::Response> res) {
    // Implementation needed
}

void ActuatorController_ROS2::serviceZeroReset(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<actuatorcontroller_ros2::srv::ZeroReset::Request> req,
    std::shared_ptr<actuatorcontroller_ros2::srv::ZeroReset::Response> res) {
    // Implementation needed
}

void ActuatorController_ROS2::releaseJointStates() {
    // Implementation needed
}

void ActuatorController_ROS2::updateROSParam() {
    // Implementation needed
}