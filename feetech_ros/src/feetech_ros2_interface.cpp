

#include "feetech_ros2_driver/feetech_ros2_interface.hpp"

FeetechROS2Interface::FeetechROS2Interface() : 
    Node("feetech_ros2_interface"),
    servo_reference_subscription_(nullptr),
    servo_state_publisher_(nullptr),
    homing_action_server_(nullptr),
    driver(nullptr),
    timer_(nullptr)
{
    // Declare all parameters
    this->declare_parameters(
        namespace="",
        parameters={
            {"node.frequency", 20},
            {"driver.port_name", "/dev/ttyUSB0"},
            {"driver.baud_rate", 1000000},
            {"driver.frequency", 100}
            {"servos.ids", {1}},
            {"servos.homing_modes", {0}},
            {"servos.max_speeds", {250}},
            {"servos.max_currents", {1000}}
        }
    );

    // Subscribers
    servo_reference_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/servo/in/reference", 10,
        std::bind(&FeetechROS2Interface::referenceCallback, this, _1)
    );

    // Publishers
    servo_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/servo/out/state", 10);

    // Actions
    homing_action_server_ = this->create_server<feetech_ros2_interfaces::action::Homing>(
        this,
        "home_servos",
        std::bind(&FeetechROS2Interface::homingCallbackGoal, this, _1, _2),
        std::bind(&FeetechROS2Interface::homingCallbackCancel, this, _1),
        std::bind(&FeetechROS2Interface::homingCallbackAccept, this, _1)
    
    // Driver
    driver = std::make_shared<FeetechServo>();
    driver->init(
        this->get_parameter("driver.port_name").as_string(),
        this->get_parameter("driver.baud_rate").as_int(),
        this->get_parameter("driver.frequency").as_double()
    );

    // Timer
    double node_frequency_ = this->get_parameter("driver.frequency").as_double();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000./node_frequency_), std::bind(&FeetechROS2Interface::loop, this));
    );
}

FeetechROS2Interface::~FeetechROS2Interface()
{
    // Close port
    driver->close();
}

void FeetechROS2Interface::loop()
{
    // Execute driver
    driver->execute();

    // Publish servo state
    publishServoState();
}

void FeetechROS2Interface::publishServoState()
{
    // TODO: this is a copilot generated code stub, review and replace
    // Publish current servo positions and velocities
    auto servo_state_msg = sensor_msgs::msg::JointState();
    std::vector<double> positions, velocities;
    for(uint16_t i = 0; i < this->ids.size(); i++) {
        positions.push_back(driver->mServoData.servo_map[this->ids.at(i)].position);
        velocities.push_back(driver->mServoData.servo_map[this->ids.at(i)].velocity);
    }

    servo_state_msg.header.stamp = this->get_clock()->now();
    servo_state_msg.position = positions;
    servo_state_msg.velocity = velocities;

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