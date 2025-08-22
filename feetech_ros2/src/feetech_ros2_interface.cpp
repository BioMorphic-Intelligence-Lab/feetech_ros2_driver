#include "feetech_ros2_interface.hpp"

FeetechROS2Interface::FeetechROS2Interface() : 
    Node("feetech_ros2_interface")
{
    // Declare all parameters
    // Node parameters
    this->declare_parameter("node.frequency", 20.);
    this->declare_parameter("node.effort_feedback_type", std::string("pwm")); // "current" or "pwm"

    // Driver parameters
    this->declare_parameter("driver.port_name", "/dev/ttyUSB0");
    this->declare_parameter("driver.baud_rate", 1000000);
    this->declare_parameter("driver.frequency", 100.);
    this->declare_parameter("driver.homing", true);
    this->declare_parameter("driver.logging", false);

    // Servo parameters
    this->declare_parameter("servos.ids", std::vector<int>{1});
    this->declare_parameter("servos.operating_modes", std::vector<int>{4});
    this->declare_parameter("servos.homing_modes", std::vector<int>{0});
    this->declare_parameter("servos.directions", std::vector<int>{1});
    this->declare_parameter("servos.max_speeds", std::vector<double>{0.1});
    this->declare_parameter("servos.max_currents", std::vector<double>{1000.0});
    this->declare_parameter("servos.gear_ratios", std::vector<double>{1.0});
    this->declare_parameter("servos.start_offsets", std::vector<double>{0.0});

    // Subscribers
    servo_reference_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/servo/in/state", 10,
        std::bind(&FeetechROS2Interface::referenceCallback, this, std::placeholders::_1)
    );

    // Publishers
    servo_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/servo/out/state", 10);
    
    // Generate uint8_t vector of ids
    std::vector<long> int_ids = this->get_parameter("servos.ids").as_integer_array();
    ids_.resize(int_ids.size());
    std::transform(int_ids.begin(), int_ids.end(), ids_.begin(),
                    [](int val) { return static_cast<uint8_t>(val); });

    // Check if param sizes match
    size_t num_servos = int_ids.size();
    this->checkParameterSizes(num_servos);

    // Construct Driver
    driver = std::make_shared<FeetechServo>(
        this->get_parameter("driver.port_name").as_string(),
        this->get_parameter("driver.baud_rate").as_int(),
        this->get_parameter("driver.frequency").as_double(),
        ids_,
        this->get_parameter("driver.homing").as_bool(),
        this->get_parameter("driver.logging").as_bool()
    );
    RCLCPP_INFO(this->get_logger(), "Driver constructed");

    // Optional: Set driver settings
    DriverSettings settings = driver->getDriverSettings();
    driver->setDriverSettings(settings);

    applyServoParams();

    start_offsets = this->get_parameter("servos.start_offsets").as_double_array();

    // Timer
    double node_frequency_ = this->get_parameter("node.frequency").as_double();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000./node_frequency_)), std::bind(&FeetechROS2Interface::loop, this));
}

FeetechROS2Interface::~FeetechROS2Interface()
{
    std::cout<<"Destroying Feetech ROS2 interface"<<std::endl;
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
    // RCLCPP_INFO(this->get_logger(), "Received servo reference message");
    if(msg->position.size() == ids_.size())
    {
        for (uint8_t i = 0; i < ids_.size(); i++)
        {
            // Find servo position and adjust for starting offset
            // If the commanded position is 0, but the servo start offset was at +1 rad, the servo needs to go to a position
            // that it thinks is -1 rad from its own zero
            double servo_position = msg->position[i] - start_offsets[i];

            // Set servo position
            if (driver->getOperatingMode(ids_[i]) == DriverMode::CONTINUOUS_POSITION)
            {
                // RCLCPP_INFO(this->get_logger(), "Setting reference position for servo %d to %f", ids_[i], servo_position);
                driver->setReferencePosition(ids_[i], servo_position);
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
            if (driver->getOperatingMode(ids_[i]) == DriverMode::VELOCITY)
            {
                // RCLCPP_INFO(this->get_logger(), "Setting reference velocity for servo %d to %f", ids_[i], servo_velocity);
                driver->setReferenceVelocity(ids_[i], servo_velocity);
            }
        }
    }
}

void FeetechROS2Interface::setMaxSpeedCallback(
    const std::shared_ptr<feetech_ros2::srv::SetMaxSpeed::Request> request,
    std::shared_ptr<feetech_ros2::srv::SetMaxSpeed::Response> response)
{
    // Set mode for all servos
    for (uint8_t i = 0; i < ids_.size(); i++)
    {
        driver->setMaxSpeed(ids_[i], static_cast<double>(request->max_speed));
    }
    response->success = true;
}


void FeetechROS2Interface::setModeCallback(
    const std::shared_ptr<feetech_ros2::srv::SetMode::Request> request,
    std::shared_ptr<feetech_ros2::srv::SetMode::Response> response)
{
    int8_t mode;
    // Set mode for all servos
    for (uint8_t i = 0; i < ids_.size(); i++)
    {
        driver->setOperatingMode(ids_[i], static_cast<DriverMode>(request->operating_mode));
        mode = driver->readOperatingMode(ids_[i]);
    }
    // If requested mode is 4 (cont position) and the STSMode is Position, mode change succesfull
    if (request->operating_mode == 4 && mode == STSMode::STS_POSITION)
    {
        response->success = true;
    }
    else if (request->operating_mode == 1 && mode == STSMode::STS_VELOCITY)
    {
        response->success = true;
    } 
    // If requested mode does not correspond to read mode
    else if (request->operating_mode == 1 && mode == STSMode::STS_POSITION)
    {
        response->success = false;
    }
    else if (request->operating_mode == 4 && mode == STSMode::STS_VELOCITY)
    {
        response->success = false;
    }
    // If mode reading (i.e. verification) fails
    else if (mode < 0)
    {
        response->success = false;
    }
}

void FeetechROS2Interface::resetHomePositionsCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if(request->data)
    {
        for (uint8_t i=0; i< ids_.size(); i++)
        {
            driver->resetHomePosition(ids_[i]);
        }
        response->success = true;
    }
}

void FeetechROS2Interface::publishServoState()
{
    // Publish current servo positions and velocities
    std::vector<double> servo_positions = driver->getCurrentPositions();
    std::vector<double> adjusted_servo_positions(servo_positions.size());

    std::vector<std::string> names(servo_positions.size());

    for (uint8_t i=0; i<servo_positions.size(); ++i)
    {
        adjusted_servo_positions[i] = servo_positions[i] + start_offsets[i];
        names[i] = "q" + std::to_string(i);
    }

    auto servo_state_msg = sensor_msgs::msg::JointState();
    servo_state_msg.header.stamp = this->get_clock()->now();
    servo_state_msg.name = names;
    servo_state_msg.position = adjusted_servo_positions;
    servo_state_msg.velocity = driver->getCurrentVelocities();
    if (this->get_parameter("node.effort_feedback_type").as_string() == std::string("pwm"))
    {
        servo_state_msg.effort = driver->getCurrentPWMs();
    }
    else if (this->get_parameter("node.effort_feedback_type").as_string() == std::string("current"))
    {
        servo_state_msg.effort = driver->getCurrentCurrents();
    }

    this->servo_state_publisher_->publish(servo_state_msg);
}

void FeetechROS2Interface::applyServoParams()
{
    // Set servo settings from parameter file
    // operating modes
    std::vector<long> operating_modes = this->get_parameter("servos.operating_modes").as_integer_array();
    std::vector<DriverMode> modes(operating_modes.size());
    std::transform(operating_modes.begin(), operating_modes.end(), modes.begin(),
                    [](int val) { return static_cast<DriverMode>(val); });
    driver->setOperatingModes(modes);

    // directions
    std::vector<long> directions = this->get_parameter("servos.directions").as_integer_array();
    std::vector<int> dirs(directions.size());
    std::transform(directions.begin(), directions.end(), dirs.begin(),
                    [](int val) { return static_cast<int>(val); });
    driver->setVelocityDirections(dirs);

    // gear ratios
    std::vector<double> gear_ratios = this->get_parameter("servos.gear_ratios").as_double_array();
    driver->setGearRatios(gear_ratios);

    // max speeds
    std::vector<double> max_speeds = this->get_parameter("servos.max_speeds").as_double_array();
    driver->setMaxSpeeds(max_speeds);
}

void FeetechROS2Interface::checkParameterSizes(size_t num_servos) const
{
    if (this->get_parameter("servos.operating_modes").as_integer_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.operating_modes not the same size as number of servos!");
        exit(-1);
    }
    if (this->get_parameter("servos.homing_modes").as_integer_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.homing_modes not the same size as number of servos!");
        exit(-1);
    }
    if (this->get_parameter("servos.directions").as_integer_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.directions not the same size as number of servos!");
        exit(-1);
    }
    if (this->get_parameter("servos.max_speeds").as_double_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.max_speeds not the same size as number of servos!");
        exit(-1);
    }
    if (this->get_parameter("servos.max_currents").as_double_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.max_currents not the same size as number of servos!");
        exit(-1);
    }
    if (this->get_parameter("servos.gear_ratios").as_double_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.gear_ratios not the same size as number of servos!");
        exit(-1);
    }
    if (this->get_parameter("servos.start_offsets").as_double_array().size() != num_servos)
    {
        RCLCPP_ERROR(this->get_logger(), "servos.start_offsets not the same size as number of servos!");
        exit(-1);
    }
}


int main(int argc, char * argv[])
{
    // Initialize ROS node
    rclcpp::init(argc, argv);
    auto feetech_ros2_interface = std::make_shared<FeetechROS2Interface>();

    rclcpp::Service<feetech_ros2::srv::SetMode>::SharedPtr setModeSrv =
        feetech_ros2_interface->create_service<feetech_ros2::srv::SetMode>("/set_servo_mode", 
            std::bind(&FeetechROS2Interface::setModeCallback, 
                feetech_ros2_interface, 
                std::placeholders::_1, 
                std::placeholders::_2));

    rclcpp::Service<feetech_ros2::srv::SetMaxSpeed>::SharedPtr setMaxSpeedSrv =
        feetech_ros2_interface->create_service<feetech_ros2::srv::SetMaxSpeed>("set_servo_max_speed", 
            std::bind(&FeetechROS2Interface::setMaxSpeedCallback, 
                feetech_ros2_interface, 
                std::placeholders::_1, 
                std::placeholders::_2));
        
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr resetHomePositionSrv =
        feetech_ros2_interface->create_service<std_srvs::srv::SetBool>("/reset_home_positions",
            std::bind(&FeetechROS2Interface::resetHomePositionsCallback,
                feetech_ros2_interface,
                std::placeholders::_1,
                std::placeholders::_2));

    rclcpp::spin(feetech_ros2_interface);
    rclcpp::shutdown();

    return 0;
}