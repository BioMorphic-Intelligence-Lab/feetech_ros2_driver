#include "feetech_ros2_interface.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

FeetechROS2Interface::FeetechROS2Interface() : 
    Node("feetech_ros2_interface")
{
    // Declare all parameters
    this->declare_parameter<double>("node.frequency", 20.);
    this->declare_parameter<std::string>("driver.port_name", "/dev/ttyUSB0");
    this->declare_parameter<int64_t>("driver.baud_rate", 1000000);
    this->declare_parameter<double>("driver.frequency", 100.);

    this->declare_parameter<std::vector<int>>("servos.ids", std::vector<int>{1});
    this->declare_parameter("servos.operating_modes", std::vector<int>{4});
    this->declare_parameter("servos.homing_modes", std::vector<int>{0});
    this->declare_parameter("servos.max_speeds", std::vector<double>{250.0});
    this->declare_parameter("servos.max_currents", std::vector<double>{1000.0});
    this->declare_parameter("servos.gear_ratios", std::vector<double>{1.0});
    this->declare_parameter<int>("effort_average_window_size", 10);

    // Subscribers
    servo_reference_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/servo/in/references", 10,
        std::bind(&FeetechROS2Interface::referenceCallback, this, std::placeholders::_1)
    );

    // Publishers
    servo_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/servo/out/state", 10);
    
    // Generate uint8_t vector of ids
    std::vector<long> int_ids = this->get_parameter("servos.ids").as_integer_array();
    ids_.resize(int_ids.size());
    std::transform(int_ids.begin(), int_ids.end(), ids_.begin(),
                    [](int val) { return static_cast<uint8_t>(val); });

    effort_average_window_size_ = static_cast<std::size_t>(std::max(
        static_cast<int64_t>(1),
        this->get_parameter("effort_average_window_size").as_int()));
    effort_history_.assign(ids_.size(), {});

    // Construct Driver
    driver = std::make_shared<FeetechServo>(
        this->get_parameter("driver.port_name").as_string(),
        this->get_parameter("driver.baud_rate").as_int(),
        this->get_parameter("driver.frequency").as_double(),
        ids_
    );
    
    // Optional: Set driver settings
    DriverSettings settings = driver->getDriverSettings();
    driver->setDriverSettings(settings);

    // Set servo settings from parameter file
    std::vector<long> operating_modes = this->get_parameter("servos.operating_modes").as_integer_array();
    interface_modes_.resize(operating_modes.size());
    std::vector<DriverMode> internal_modes(operating_modes.size());
    for (std::size_t i = 0; i < operating_modes.size(); ++i)
    {
        interface_modes_[i] = static_cast<DriverMode>(operating_modes[i]);
        // For CONTINUOUS_POSITION, run the hardware in VELOCITY mode and let this node
        // handle multi-turn position control in software.
        if (interface_modes_[i] == DriverMode::CONTINUOUS_POSITION)
        {
            internal_modes[i] = DriverMode::VELOCITY;
        }
        else
        {
            internal_modes[i] = interface_modes_[i];
        }
    }
    driver->setOperatingModes(internal_modes);

    std::vector<double> gear_ratios = this->get_parameter("servos.gear_ratios").as_double_array();
    driver->setGearRatios(gear_ratios);

    // Read max speeds for software continuous position control (rad/s)
    max_speeds_ = this->get_parameter("servos.max_speeds").as_double_array();

    // Initialize continuous position control state from current positions
    const auto current_positions = driver->getCurrentPositions();
    std::size_t n = ids_.size();
    continuous_positions_.assign(n, 0.0);
    last_raw_positions_.assign(n, 0.0);
    target_positions_.assign(n, 0.0);
    if (current_positions.size() == n)
    {
        for (std::size_t i = 0; i < n; ++i)
        {
            continuous_positions_[i] = current_positions[i];
            last_raw_positions_[i] = current_positions[i];
            target_positions_[i] = current_positions[i];
        }
    }

    // Timer
    double node_frequency_ = this->get_parameter("node.frequency").as_double();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000./node_frequency_)), std::bind(&FeetechROS2Interface::loop, this));
}

FeetechROS2Interface::~FeetechROS2Interface()
{
    // Stop all servos
    driver->stopAll();
    // Close port
    driver->close();
}

void FeetechROS2Interface::loop()
{
    // Update software continuous positions from current raw positions
    const auto raw_positions = driver->getCurrentPositions();
    const std::size_t n = ids_.size();
    if (raw_positions.size() == n)
    {
        if (continuous_positions_.size() != n)
        {
            continuous_positions_.assign(n, 0.0);
            last_raw_positions_.assign(n, 0.0);
        }

        constexpr double PI = M_PI;
        constexpr double TWO_PI = 2.0 * M_PI;

        for (std::size_t i = 0; i < n; ++i)
        {
            double raw = raw_positions[i];
            double & last_raw = last_raw_positions_[i];
            double & cont = continuous_positions_[i];

            double delta = raw - last_raw;
            if (delta > PI)
            {
                delta -= TWO_PI;
            }
            else if (delta < -PI)
            {
                delta += TWO_PI;
            }

            cont += delta;
            last_raw = raw;
        }
    }

    // Absolute multi-turn position control in CONTINUOUS_POSITION mode using velocity commands
    if (target_positions_.size() != n)
    {
        target_positions_ = continuous_positions_;
    }
    for (std::size_t i = 0; i < n; ++i)
    {
        if (interface_modes_[i] == DriverMode::CONTINUOUS_POSITION)
        {
            double error = target_positions_[i] - continuous_positions_[i];

            // Simple P controller on position error -> velocity command
            const double Kp = 1.0;  // rad/s per rad of error (tune as needed)
            double v_cmd = Kp * error;

            double vmax = (i < max_speeds_.size()) ? max_speeds_[i] : std::numeric_limits<double>::infinity();
            if (std::isfinite(vmax))
            {
                if (v_cmd > vmax) v_cmd = vmax;
                else if (v_cmd < -vmax) v_cmd = -vmax;
            }

            driver->setReferenceVelocity(ids_[i], v_cmd);
        }
    }

    // Publish servo state to ROS2 network
    publishServoState();
}

void FeetechROS2Interface::referenceCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if(msg->position.size() == ids_.size())
    {
        for (uint8_t i = 0; i < ids_.size(); i++)
        {
            // Find servo position
            double servo_position = msg->position[i];

            // Absolute multi-turn position command in continuous position mode:
            // store target; control is handled in loop() via velocity commands
            if (interface_modes_[i] == DriverMode::CONTINUOUS_POSITION)
            {
                // Ensure storage is sized correctly in case of parameter changes
                if (target_positions_.size() != ids_.size())
                {
                    target_positions_.assign(ids_.size(), 0.0);
                }

                target_positions_[i] = servo_position;
            }
        }
    }
    if(msg->velocity.size() == ids_.size())
    {
        for (uint8_t i = 0; i < ids_.size(); i++)
        {
            // Find servo velocity
            double servo_velocity = msg->velocity[i];

            // Set servo velocity
            if (interface_modes_[i] == DriverMode::VELOCITY)
            {
                driver->setReferenceVelocity(ids_[i], servo_velocity);
            }
        }
    }
}

void FeetechROS2Interface::setModeCallback(
    const std::shared_ptr<feetech_ros2::srv::SetMode::Request> request,
    std::shared_ptr<feetech_ros2::srv::SetMode::Response> response)
{
    // Set mode for all servos
    for (uint8_t i = 0; i < ids_.size(); i++)
    {
        driver->setOperatingMode(ids_[i], static_cast<DriverMode>(request->operating_mode));
    }
    response->success = true;
}

double FeetechROS2Interface::pushEffortAverage(const std::size_t servo_idx, const double raw_effort)
{
    if (servo_idx >= effort_history_.size())
    {
        return raw_effort;
    }

    std::deque<double> & window = effort_history_[servo_idx];
    window.push_back(raw_effort);
    while (window.size() > effort_average_window_size_)
    {
        window.pop_front();
    }

    double sum = 0.0;
    for (const double sample : window)
    {
        sum += sample;
    }
    return sum / static_cast<double>(window.size());
}

void FeetechROS2Interface::publishServoState()
{
    const auto raw_efforts = driver->getStallEffortAmps();
    auto servo_state_msg = sensor_msgs::msg::JointState();
    servo_state_msg.header.stamp = this->get_clock()->now();
    servo_state_msg.position = driver->getCurrentPositions();
    servo_state_msg.velocity = driver->getCurrentVelocities();
    servo_state_msg.effort.resize(raw_efforts.size());
    for (std::size_t i = 0; i < raw_efforts.size(); ++i)
    {
        servo_state_msg.effort[i] = pushEffortAverage(i, raw_efforts[i]);
    }

    this->servo_state_publisher_->publish(servo_state_msg);
}


int main(int argc, char * argv[])
{
  // Initialize ROS node
  rclcpp::init(argc, argv);
  auto feetech_ros2_interface = std::make_shared<FeetechROS2Interface>();

  rclcpp::Service<feetech_ros2::srv::SetMode>::SharedPtr setModeSrv =
    feetech_ros2_interface->create_service<feetech_ros2::srv::SetMode>("set_servo_mode", 
        std::bind(&FeetechROS2Interface::setModeCallback, 
            feetech_ros2_interface, 
            std::placeholders::_1, 
            std::placeholders::_2));

  rclcpp::spin(feetech_ros2_interface);
  rclcpp::shutdown();

  return 0;
}