// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
//
// Author: Will Son
*******************************************************************************/

#include "feetech_ros2_driver.hpp"

// Limit switch input pins -- Check gpio readall
#define LIMIT_PIVOT_1 10
#define LIMIT_PIVOT_2 9
#define LIMIT_SHOULDER_1 6
#define LIMIT_SHOULDER_2 13

using std::placeholders::_1;

DriverFeetechServo::DriverFeetechServo()
: Node("feetech_ros2_driver"),
  mServoData(),
  portHandler(nullptr),
  packetHandler(nullptr),
  mErrorCode(0),
  mCommResult(0),
  mHomePositionIncrement(10),
  mCurrentThreshold(100),
  mNodeFrequency(0)
{
  RCLCPP_INFO(this->get_logger(), "Started Feetech servo driver node");

  // Declare all parameter
  this->declare_parameter("motor_ids",
                          std::vector<int>{1});
  this->declare_parameter("frequency", 20);
  // QoS settings
  this->declare_parameter("qos_depth", 10);
  
  this->reference_servo_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/servo/in/reference", 10,
    std::bind(&DriverFeetechServo::referenceServoStateCallback,
              this,
              _1)
  );
  // Publishers
  // Publish relevant information i.e. servo position (calculate in this node?), velocity, torque etc.
  this->current_servo_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/servo/out/current_state", 10);

  // Get all parameters
  this->ids = this->get_parameter("motor_ids").as_integer_array();

  // Initialize Servos
  InitializeServos();

  // Timer
  mNodeFrequency = this->get_parameter("frequency").as_int();
  mTimer = this->create_wall_timer(std::chrono::milliseconds(1000/mNodeFrequency), std::bind(&DriverFeetechServo::timerCallback, this));
}

DriverFeetechServo::~DriverFeetechServo()
{
  // Close port
  portHandler->closePort();
}

void DriverFeetechServo::timerCallback()
{
  // Get all present positions, velocities and currents
  getAllPresentPositions();
  getAllPresentVelocities();
  getAllPresentCurrents();

  // Publish current servo positions and velocities
  PublishServoData();
}

void DriverFeetechServo::PublishServoData()
{
  // Publish current servo positions and velocities
  auto current_servo_state_msg = sensor_msgs::msg::JointState();
  std::vector<double> positions, velocities;
  for(uint16_t i = 0; i < this->ids.size(); i++) {
    positions.push_back(mServoData.servo_map[this->ids.at(i)].position);
    velocities.push_back(mServoData.servo_map[this->ids.at(i)].velocity);
  }

  current_servo_state_msg.header.stamp = this->get_clock()->now();
  current_servo_state_msg.position = positions;
  current_servo_state_msg.velocity = velocities;

  this->current_servo_state_publisher_->publish(current_servo_state_msg);
}

/* Set reference state directly on servo
*/
void DriverFeetechServo::referenceServoStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Set reference position for all servos
  for(uint16_t i = 0; i < this->ids.size(); i++) {
    setPositionReference(this->ids.at(i), msg->position[i]);
    setVelocityReference(this->ids.at(i), msg->velocity[i]);
  }
}

void DriverFeetechServo::HomeAll()
{
  RCLCPP_INFO(this->get_logger(), "Homing all servos not implemented yet");
}

void DriverFeetechServo::HomeSingleServo(const int id)
{
  /*if (mServoData.servo_map[id].homing_mode==SWITCH_BASED)
  {
    if (digitalRead(mServoData.servo_map[id].limit_switch_pin)==HIGH) // Limit switch is pressed
    {
      // Move servo until limit switch is not pressed
      while(digitalRead(mServoData.servo_map[id].limit_switch_pin)==HIGH)
      {
        getSinglePresentPosition(id);
        setPositionReference(id, mServoData.servo_map[id].position+mHomePositionIncrement);
      }
    }
    if (digitalRead(mServoData.servo_map[id].limit_switch_pin)==LOW) // Limit switch is not pressed
    {
      // Move servo until limit switch is pressed
      while(digitalRead(mServoData.servo_map[id].limit_switch_pin)==LOW)
      {
        getSinglePresentPosition(id);
        setPositionReference(id, mServoData.servo_map[id].position-mHomePositionIncrement);
      }
      getSinglePresentPosition(id);
      mServoData.servo_map[id].home_position = mServoData.servo_map[id].position;
    }
  }*/
  if (mServoData.servo_map[id].homing_mode==LOAD_BASED)
  {
    // Read present current and save
    getSinglePresentCurrent(id);
    int baseline_current = mServoData.servo_map[id].current;
    while (mServoData.servo_map[id].current-baseline_current < mCurrentThreshold)
    {
      getSinglePresentPosition(id);
      setPositionReference(id, mServoData.servo_map[id].position+mHomePositionIncrement);
    }
    getSinglePresentPosition(id);
    mServoData.servo_map[id].home_position = mServoData.servo_map[id].position;
  }
}

/*
 * Initialize Servos
 */
void DriverFeetechServo::InitializeServos()
{
  // Set PortHandler and PacketHandler
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  mCommResult = portHandler->openPort();
  if (mCommResult == false) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port
  mCommResult = portHandler->setBaudRate(BAUDRATE);
  if (mCommResult == false) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set the baudrate.");
  }

  // Add all Servos and populate data
  for(uint16_t i = 0; i < this->ids.size(); i++) {
    mServoData.AddServo(ServoState(
      this->ids.at(i), 
      0, 
      0, 
      0, 
      0, 
      0, 
      0, 
      LOAD_BASED, 
      VELOCITY_MODE));
  }

  getAllPresentPositions();
  getAllPresentVelocities();
  getAllPresentCurrents();

  // Set all servos to torque enable
  setAllEnable(ENABLED);

  // home the servos
  //for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
  //  HomeSingleServo(id);
  //}

  // set all servos to velocity mode
  setAllMode(VELOCITY_MODE);
};

/* Get present position for servo ID and set on servo data struct
*/
void DriverFeetechServo::getSinglePresentPosition(const int id)
{
  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  mCommResult = packetHandler->read2ByteTxRx(
    portHandler,
    (uint8_t) mServoData.servo_map[id].id,
    ADDR_PRESENT_POSITION,
    reinterpret_cast<uint16_t *>(&mServoData.servo_map[id].position),
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get present position. Error code %c", mErrorCode);
  } 
  else {
    RCLCPP_INFO(this->get_logger(), "Get [ID: %d] [Present position: %d ticks]",
    mServoData.servo_map[id].id,
    mServoData.servo_map[id].position);
  }
};

void DriverFeetechServo::getSinglePresentVelocity(const int id)
{

  uint16_t data;
  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  mCommResult = packetHandler->read2ByteTxRx(
    portHandler,
    (uint8_t) mServoData.servo_map[id].id,
    ADDR_PRESENT_SPEED,
    &data,
    &mErrorCode
  );

  int signedValue = data & ~0x8000;
  if (data & 0x8000)
      signedValue = -signedValue;
  mServoData.servo_map[id].velocity = signedValue;

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get present velocity. Error code %c", mErrorCode);
  } 
  else {
    RCLCPP_INFO(this->get_logger(), "Get [ID: %d] [Present velocity: %d ticks/s]",
    mServoData.servo_map[id].id,
    mServoData.servo_map[id].velocity);
  }
};

void DriverFeetechServo::getSinglePresentCurrent(const int id)
{
  // Read present current (length: 2 bytes)
  mCommResult = packetHandler->read2ByteTxRx(
    portHandler,
    (uint8_t) mServoData.servo_map[id].id,
    ADDR_PRESENT_CURRENT,
    reinterpret_cast<uint16_t *>(&mServoData.servo_map[id].current),
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get present current. Error code %c", mErrorCode);
  } 
  else {
    RCLCPP_INFO(this->get_logger(), "Get [ID: %d] [Present current: %f mA]",
    mServoData.servo_map[id].id,
    mServoData.servo_map[id].current*6.5);
  }
}

void DriverFeetechServo::getAllPresentPositions()
{
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
      getSinglePresentPosition(id);
  }
};

void DriverFeetechServo::getAllPresentVelocities()
{
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
      getSinglePresentVelocity(id);
  }
};

void DriverFeetechServo::getAllPresentCurrents()
{
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
      getSinglePresentCurrent(id);
  }
};

void DriverFeetechServo::setSingleMode(const int id, const ControlMode &mode)
{
  mServoData.servo_map[id].control_mode = mode;
  // Set all servos to position mode
  mCommResult = packetHandler->write1ByteTxRx(
    portHandler,
    (uint8_t) mServoData.servo_map[id].id,
    ADDR_OPERATING_MODE,
    mode,
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control Mode.Error code %c", mErrorCode);
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set control mode to %d.", mode);
    mServoData.servo_map[id].control_mode = mode;
  }
}

/* Set servo control mode
 * @param servo_id: ID of the servo
 * @param mode: 0 for position mode, 1 for velocity mode
 */
void DriverFeetechServo::setAllMode(const ControlMode &mode)
{
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
    setSingleMode(id, mode);
  }
};

void DriverFeetechServo::setPositionReference(const int id, const int &reference)
{
  mCommResult = packetHandler->write2ByteTxRx(
    portHandler,
    id,
    ADDR_GOAL_POSITION,
    reference,
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set reference. Error code %c", mErrorCode);
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set reference.");
  }
};

void DriverFeetechServo::setVelocityReference(const int id, const int &reference)
{
  uint16_t ref = abs(reference);
  
  if (reference < 0) { 
    ref = 0x8000 | ref;
  } 

  mCommResult = packetHandler->write2ByteTxRx(
    portHandler,
    id,
    ADDR_GOAL_SPEED,
    ref,
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set reference. Error code %c", mErrorCode);
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set reference.");
  }
}

void DriverFeetechServo::setSingleEnable(const int id, const TorqueEnable &enable)
{
  mCommResult = packetHandler->write1ByteTxRx(
    portHandler,
    (uint8_t) id,
    ADDR_TORQUE_ENABLE,
    enable,
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set torque enable. Error code %c", mErrorCode);
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set torque enable.");
  }
}

void DriverFeetechServo::setAllEnable(const TorqueEnable &enable)
{
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
    setSingleEnable(id, enable);
  }
};



int main(int argc, char * argv[])
{
  // Set up the gpio using wiringPi for limit switches
  /*wiringPiSetup();
  pinMode(LIMIT_PIVOT_1, INPUT);
  pinMode(LIMIT_PIVOT_2, INPUT);
  pinMode(LIMIT_SHOULDER_1, INPUT);
  pinMode(LIMIT_SHOULDER_2, INPUT);*/

  // Initialize ROS node
  rclcpp::init(argc, argv);
  auto driverfeetechservo = std::make_shared<DriverFeetechServo>();
  rclcpp::spin(driverfeetechservo);
  rclcpp::shutdown();

  return 0;
}
