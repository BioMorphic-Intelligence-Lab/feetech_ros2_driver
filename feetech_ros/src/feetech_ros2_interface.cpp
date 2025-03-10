

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
    this->declare_parameter("servos.homing_modes", std::vector<int>{0});
    this->declare_parameter("servos.max_speeds", std::vector<double>{250});
    this->declare_parameter("servos.max_currents", std::vector<double>{1000});

    // Subscribers
    servo_reference_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/servo/in/reference", 10,
        std::bind(&FeetechROS2Interface::referenceCallback, this, std::placeholders::_1)
    );

    // Publishers
    servo_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/servo/out/state", 10);
    
    // Driver
    driver = std::make_shared<FeetechServo>();
    driver->init(
        this->get_parameter("driver.port_name").as_string(),
        this->get_parameter("driver.baud_rate").as_int(),
        this->get_parameter("driver.frequency").as_double()
    );

    // Timer
    double node_frequency_ = this->get_parameter("node.frequency").as_double();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000./node_frequency_)), std::bind(&FeetechROS2Interface::loop, this));
}

FeetechROS2Interface::~FeetechROS2Interface()
{
    // Close port
    driver->close();
}

void FeetechROS2Interface::loop()
{

    // Publish servo state
    publishServoState();
}

void FeetechROS2Interface::referenceCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (int i = 0; i<msg->name.size(); i++)
    {
        // Find servo ID
        int servo_id = std::stoi(msg->name[i]);
        // Find servo position
        int servo_position = msg->position[i];
        // Find servo velocity
        int servo_velocity = msg->velocity[i];

        // Set servo position
        if (driver->getOperatingMode() == STSMode::POSITION)
            driver->setReferencePosition(servo_id, servo_position);
        else if (driver->getOperatingMode() == STSMode::VELOCITY)
            driver->setReferenceVelocity(servo_id, servo_velocity);
    }
}

void FeetechROS2Interface::publishServoState()
{
    // TODO: this is a copilot generated code stub, review and replace
    // Publish current servo positions and velocities
    auto servo_state_msg = sensor_msgs::msg::JointState();
    servo_state_msg.header.stamp = this->get_clock()->now();
    servo_state_msg.position = driver->getCurrentPositions();;
    servo_state_msg.velocity = driver->getCurrentVelocities();;

    this->servo_state_publisher_->publish(servo_state_msg);
}


int main(int argc, char * argv[])
{
  // Initialize ROS node
  rclcpp::init(argc, argv);
  auto feetech_ros2_interface = std::make_shared<FeetechROS2Interface>();
  rclcpp::spin(feetech_ros2_interface);
  rclcpp::shutdown();

  return 0;
}