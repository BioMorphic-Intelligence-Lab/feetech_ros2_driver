#include "rclcpp/rclcpp.hpp"
#include "feetech_cpp_lib/feetech_lib.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <feetech_ros2/srv/set_mode.hpp>


class FeetechROS2Interface : public rclcpp::Node
{
public:
    FeetechROS2Interface();
    ~FeetechROS2Interface();

    void setModeCallback(const std::shared_ptr<feetech_ros2::srv::SetMode::Request> request,
                         std::shared_ptr<feetech_ros2::srv::SetMode::Response> response);

private:
    /// @brief Callback for the timer, execute interface loop
    void loop();

    /// @brief Publish the servo state to the ROS2 network
    void publishServoState();

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_reference_subscription_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr servo_state_publisher_;

    // Callbacks
    void referenceCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    // Data
    std::shared_ptr<FeetechServo> driver;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<uint8_t> ids_;

};