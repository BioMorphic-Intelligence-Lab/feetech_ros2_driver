#include "rclcpp/rclcpp.hpp"
#include "feetech_cpp_lib/feetech_lib.hpp"

#include <sensor_msgs/msg/joint_state.hpp>
#include "std_srvs/srv/set_bool.hpp"

#include <feetech_ros2/srv/set_mode.hpp>

#include <feetech_ros2/srv/set_mode.hpp>


class FeetechROS2Interface : public rclcpp::Node
{
public:
    FeetechROS2Interface();
    ~FeetechROS2Interface();

    void setModeCallback(const std::shared_ptr<feetech_ros2::srv::SetMode::Request> request,
                         std::shared_ptr<feetech_ros2::srv::SetMode::Response> response);
    
    void resetHomePositionsCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

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

    // Helpers
    /// @brief Apply all the servo parameters from the parameter file
    void applyServoParams();
    
    /// @brief Check if the length of the parameter defining vectors are the same as the number of ids
    void checkParameterSizes(size_t num_servos) const;
    
    // Data
    std::shared_ptr<FeetechServo> driver;
    std::vector<double> start_offsets;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<uint8_t> ids_;

};