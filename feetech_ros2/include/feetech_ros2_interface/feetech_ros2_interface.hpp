#include "rclcpp/rclcpp.hpp"
#include "feetech_cpp_lib/feetech_lib.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <feetech_ros2/srv/set_mode.hpp>

#include <algorithm>
#include <deque>
#include <mutex>
#include <vector>


class FeetechROS2Interface : public rclcpp::Node
{
public:
    FeetechROS2Interface();
    ~FeetechROS2Interface();

    void setModeCallback(const std::shared_ptr<feetech_ros2::srv::SetMode::Request> request,
                         std::shared_ptr<feetech_ros2::srv::SetMode::Response> response);

    /// @brief Whether the node should drive the servos to the safe positions on shutdown
    bool returnToSafeOnShutdown() const { return return_to_safe_on_shutdown_; }

    /// @brief Blocking: drive CONTINUOUS_POSITION servos to the safe positions, then stop them.
    /// Does not use ROS communication, so it can run after rclcpp has shut down.
    void returnToSafePosition();

private:
    /// @brief Callback for the timer, execute interface loop
    void loop();

    /// @brief Publish the servo state to the ROS2 network
    void publishServoState();
    /// @brief Unwrap raw servo positions into multi-turn continuous positions
    void updateContinuousPositions();
    /// @brief Velocity command from the position error, for CONTINUOUS_POSITION servos
    void runPositionControl();
    /// @brief Command zero velocity to all servos
    void stopServos();
    double pushEffortAverage(std::size_t servo_idx, double raw_effort);

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_reference_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr safe_reference_subscription_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr servo_state_publisher_;

    // Callbacks
    void referenceCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void safeReferenceCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    // Data
    std::shared_ptr<FeetechServo> driver;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<uint8_t> ids_;
    std::vector<DriverMode> interface_modes_;
    // Software continuous position control state (radians)
    std::vector<double> continuous_positions_;
    std::vector<double> last_raw_positions_;
    std::vector<double> target_positions_;
    std::vector<double> max_speeds_;
    // Shutdown behaviour
    std::mutex safe_positions_mutex_;
    std::vector<double> safe_positions_;
    bool return_to_safe_on_shutdown_{false};
    double shutdown_timeout_s_{10.0};
    double shutdown_tolerance_{0.05};
    double driver_frequency_{100.0};
    std::size_t effort_average_window_size_{10};
    std::vector<std::deque<double>> effort_history_;

};