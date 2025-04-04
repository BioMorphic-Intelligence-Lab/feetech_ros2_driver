#include "feetech_ros2_interface.hpp"

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

    // Subscribers
    servo_reference_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/servo/in/references/joint_velocities", 10,
        std::bind(&FeetechROS2Interface::referenceCallback, this, std::placeholders::_1)
    );

    // Publishers
    servo_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/servo/out/state", 10);
    
    // Generate uint8_t vector of ids
    std::vector<long> int_ids = this->get_parameter("servos.ids").as_integer_array();
    ids_.resize(int_ids.size());
    std::transform(int_ids.begin(), int_ids.end(), ids_.begin(),
                    [](int val) { return static_cast<uint8_t>(val); });

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
    std::vector<DriverMode> modes(operating_modes.size());
    std::transform(operating_modes.begin(), operating_modes.end(), modes.begin(),
                    [](int val) { return static_cast<DriverMode>(val); });
    driver->setOperatingModes(modes);

    std::vector<double> gear_ratios = this->get_parameter("servos.gear_ratios").as_double_array();
    driver->setGearRatios(gear_ratios);

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

            // Set servo position
            if (driver->getOperatingMode(ids_[i]) == DriverMode::CONTINUOUS_POSITION)
                driver->setReferencePosition(ids_[i], servo_position);
        }
    }
    if(msg->velocity.size() == ids_.size())
    {
        for (uint8_t i = 0; i < ids_.size(); i++)
        {
            // Find servo velocity
            double servo_velocity = msg->velocity[i];

            // Set servo velocity
            if (driver->getOperatingMode(ids_[i]) == DriverMode::VELOCITY)
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

void FeetechROS2Interface::publishServoState()
{
    // TODO: this is a copilot generated code stub, review and replace
    // Publish current servo positions and velocities
    auto servo_state_msg = sensor_msgs::msg::JointState();
    servo_state_msg.header.stamp = this->get_clock()->now();
    servo_state_msg.position = driver->getCurrentPositions();
    servo_state_msg.velocity = driver->getCurrentVelocities();

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