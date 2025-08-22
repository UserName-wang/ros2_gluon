#include "actuatorcontroller_ros2/actuatorcontroller_ros2.h"
#include "actuatorcontroller_ros2/trajectory_subscriber.h"
#include "actuatorcontroller.h"
#include "rclcpp/rclcpp.hpp"

std::shared_ptr<ActuatorController_ROS2> INNFOS_ptr;
std::shared_ptr<TrajectorySubscriber> trajectory_subscriber_ptr;

void actuator_event_callback()
{
    ActuatorController::getInstance()->processEvents();
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("innfos_actuator");
    RCLCPP_INFO(node->get_logger(), "Started Actuator ROS2 Node");
    
    ActuatorController::initController();
    
    Actuator::ErrorsDefine _errorCode;
    ActuatorController::getInstance()->lookupActuators(_errorCode);
    
    INNFOS_ptr = std::make_shared<ActuatorController_ROS2>(node);
    
    // Create trajectory subscriber for MoveIt2 integration
    trajectory_subscriber_ptr = std::make_shared<TrajectorySubscriber>(node, ActuatorController::getInstance());
    
    rclcpp::WallRate loop_rate(1000); // 1000 Hz
    
    // Timer for actuator events
    auto timer1 = node->create_wall_timer(
        std::chrono::milliseconds(1),
        []() { actuator_event_callback(); });
    
    // Check if a control rate is given
    int control_rate = 100; // Default rate
    // In ROS2, parameter handling is different - this is a simplified approach
    // Proper implementation would use node->declare_parameter and node->get_parameter
    
    auto timer2 = node->create_wall_timer(
        std::chrono::microseconds(1000000/control_rate),
        []() { 
            if (INNFOS_ptr) {
                INNFOS_ptr->releaseJointStates();
                INNFOS_ptr->updateROSParam();
            }
        });
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}