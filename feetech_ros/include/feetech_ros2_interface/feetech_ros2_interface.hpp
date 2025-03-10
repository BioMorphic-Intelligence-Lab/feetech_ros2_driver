#include "rclcpp/rclcpp.hpp"
#include "feetech_lib/feetech_lib.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

class FeetechROS2Interface : public rclcpp::Node
{
public:
    FeetechROS2Interface();
    ~FeetechROS2Interface();

private:
    /// @brief Callback for the timer, execute interface loop
    void loop();

    /// @brief Publish the servo state to the ROS2 network
    void publishServoState();

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_reference_subscription_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr servo_state_publisher_;

    // Actions
    rclcpp::Server<feetech_ros2_interfaces::action::Homing>::SharedPtr homing_action_server_;

    // Data
    std::shared_ptr<FeetechServo> driver;
    rclcpp::TimerBase::SharedPtr timer_;

}