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

#include <cstdio>
#include <stdlib.h>     //for using the function sleep
#include <memory>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <wiringPi.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "feetech_ros2_driver/feetech_ros2_driver.hpp"

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
  mMaxVelocity(250.f),
  mCurrentThreshold(6.f),
  mNodeFrequency(0),
  mPositionPGain(0.3f),
  mPositionDGain(0.15f),
  mPositionThreshold(20)
{
  RCLCPP_INFO(this->get_logger(), "Started Feetech servo driver node");

  // parameter
  this->declare_parameter("pivot_1_id",02);
  this->declare_parameter("shoulder_1_id",99);
  this->declare_parameter("elbow_1_id",98);

  this->declare_parameter("pivot_2_id",02);
  this->declare_parameter("shoulder_2_id",99);
  this->declare_parameter("elbow_2_id",98);

  this->declare_parameter("limit_1_pivot", 10);
  this->declare_parameter("limit_1_shoulder", 3);

  this->declare_parameter("limit_2_pivot", 10);
  this->declare_parameter("limit_2_shoulder", 3);

  this->declare_parameter("frequency", 20);

  this->declare_parameter("home_velocity", 3);
  mHomeVelocity = this->get_parameter("home_velocity").as_int();

  this->declare_parameter("port", "/dev/ttyUSB0");
  mDeviceName = this->get_parameter("port").as_string();

  this->declare_parameter("namespace", "servo");

  // QoS settings
  this->declare_parameter("qos_depth", 10);
  // int8_t qos_depth = 0;
  // this->get_parameter("qos_depth", qos_depth);
  // const auto QOS_RKL10V =
  //   rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // Subscriptions
  // Subscribe to topic to set mode and to topic to set reference (i.e. reference--> one messag for all servos)
  mNamespace = this->get_parameter("namespace").as_string();
  reference_servo_position_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/"+mNamespace+"/in/reference_position", 10, std::bind(&DriverFeetechServo::referenceServoPositionCallback, this, _1));
  reference_servo_velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/"+mNamespace+"/in/reference_velocity", 10, std::bind(&DriverFeetechServo::referenceServoVelocityCallback, this, _1));

  // Publishers
  // Publish relevant information i.e. servo position (calculate in this node?), velocity, torque etc.
  current_servo_position_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/"+mNamespace+"/out/current_position", 10);
  current_servo_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/"+mNamespace+"/out/current_velocity", 10);

  // Initialize Servos
  InitializeServos();

  // Timer
  RCLCPP_INFO(this->get_logger(), "Initialization finished, starting timer.");
  mNodeFrequency = this->get_parameter("frequency").as_int();
  mTimer = this->create_wall_timer(std::chrono::milliseconds(1000/mNodeFrequency), std::bind(&DriverFeetechServo::timerCallback, this));
}

DriverFeetechServo::~DriverFeetechServo()
{
  // set all velocities to zero to stop servo
  // Without this the last set velocity will be maintained and the servo will not stop turning!
  setAllMode(VELOCITY_MODE);
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
    setVelocityReference(id, 0);
  }

  // Disable all servos
  setAllEnable(DISABLED);
  
  // Close port
  portHandler->closePort();
}

void DriverFeetechServo::timerCallback()
{
  // Get all present positions, velocities and currents
  getAllPresentPositions();
  getAllPresentVelocities();
  getAllPresentCurrents();

  tMoveToAbsolutePosition();

  // Publish current servo positions and velocities
  PublishServoData();
}

void DriverFeetechServo::tMoveToAbsolutePosition()
{
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
    RCLCPP_DEBUG(this->get_logger(), "===== MOVE ABS POS ID %d =====", id);

    // Calculate ticks to go
    mServoData.servo_map[id].ticks_to_go = (int)((mServoData.servo_map[id].absolute_goal_position - 
    mServoData.servo_map[id].absolute_current_position)*4096*mServoData.servo_map[id].gear_ratio/(2*M_PI));
    RCLCPP_DEBUG(this->get_logger(), "Abs goal pos: %.2f, Abs curr pos: %.2f", mServoData.servo_map[id].absolute_goal_position, mServoData.servo_map[id].absolute_current_position);
    RCLCPP_DEBUG(this->get_logger(), "Ticks to go: %d", mServoData.servo_map[id].ticks_to_go);
    
    // Calculate saturated velocity
    int velocity = (int)std::max(std::min(mPositionPGain*mServoData.servo_map[id].ticks_to_go + mPositionDGain*mServoData.servo_map[id].velocity, mMaxVelocity), -mMaxVelocity);

    // Write velocity
    if (abs(mServoData.servo_map[id].ticks_to_go)<mPositionThreshold)
    { RCLCPP_DEBUG(this->get_logger(), "Stopping servo %d", id);
      setVelocityReference(id, 0);
    }
    else {setVelocityReference(id, velocity*mServoData.servo_map[id].gear_ratio);}
  }
}

void DriverFeetechServo::PublishServoData()
{
  // Publish current servo positions and velocities
  auto current_servo_position_msg = geometry_msgs::msg::TwistStamped();
  auto current_servo_velocity_msg = geometry_msgs::msg::TwistStamped();
  current_servo_position_msg.twist.linear.x = mServoData.servo_map[this->get_parameter("pivot_1_id").as_int()].position;
  current_servo_position_msg.twist.linear.y = mServoData.servo_map[this->get_parameter("shoulder_1_id").as_int()].position;
  current_servo_position_msg.twist.linear.z = mServoData.servo_map[this->get_parameter("elbow_1_id").as_int()].position;

  current_servo_velocity_msg.twist.linear.x = mServoData.servo_map[this->get_parameter("pivot_1_id").as_int()].velocity;
  current_servo_velocity_msg.twist.linear.y = mServoData.servo_map[this->get_parameter("shoulder_1_id").as_int()].velocity;
  current_servo_velocity_msg.twist.linear.z = mServoData.servo_map[this->get_parameter("elbow_1_id").as_int()].velocity;

  current_servo_position_msg.twist.angular.x = mServoData.servo_map[this->get_parameter("pivot_2_id").as_int()].position;
  current_servo_position_msg.twist.angular.y = mServoData.servo_map[this->get_parameter("shoulder_2_id").as_int()].position;
  current_servo_position_msg.twist.angular.z = mServoData.servo_map[this->get_parameter("elbow_2_id").as_int()].position;

  current_servo_velocity_msg.twist.angular.x = mServoData.servo_map[this->get_parameter("pivot_2_id").as_int()].velocity;
  current_servo_velocity_msg.twist.angular.y = mServoData.servo_map[this->get_parameter("shoulder_2_id").as_int()].velocity;
  current_servo_velocity_msg.twist.angular.z = mServoData.servo_map[this->get_parameter("elbow_2_id").as_int()].velocity;

  current_servo_position_msg.header.stamp = this->now();
  current_servo_velocity_msg.header.stamp = this->now();

  current_servo_position_publisher_->publish(current_servo_position_msg);
  current_servo_velocity_publisher_->publish(current_servo_velocity_msg);
}

/* Set reference position of the output in rad w.r.t. the home position
*/
void DriverFeetechServo::referenceServoPositionCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received reference position message");
  // Set absolute goal position on servo object
  mServoData.servo_map[this->get_parameter("pivot_1_id").as_int()].absolute_goal_position = msg->twist.linear.x;
  mServoData.servo_map[this->get_parameter("shoulder_1_id").as_int()].absolute_goal_position = msg->twist.linear.y;
  mServoData.servo_map[this->get_parameter("elbow_1_id").as_int()].absolute_goal_position = msg->twist.linear.z;

  mServoData.servo_map[this->get_parameter("pivot_2_id").as_int()].absolute_goal_position = msg->twist.angular.x;
  mServoData.servo_map[this->get_parameter("shoulder_2_id").as_int()].absolute_goal_position = msg->twist.angular.y;
  mServoData.servo_map[this->get_parameter("elbow_2_id").as_int()].absolute_goal_position = msg->twist.angular.z;
}

/* Set velocity reference directly on servo
*/
void DriverFeetechServo::referenceServoVelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // Set reference velocity for all servos
  setVelocityReference(this->get_parameter("pivot_1_id").as_int(), msg->twist.linear.x);
  setVelocityReference(this->get_parameter("shoulder_1_id").as_int(), msg->twist.linear.y);
  setVelocityReference(this->get_parameter("elbow_1_id").as_int(), msg->twist.linear.z);

  setVelocityReference(this->get_parameter("pivot_2_id").as_int(), msg->twist.angular.x);
  setVelocityReference(this->get_parameter("shoulder_2_id").as_int(), msg->twist.angular.y);
  setVelocityReference(this->get_parameter("elbow_2_id").as_int(), msg->twist.angular.z);
}

void DriverFeetechServo::HomeAll()
{
  RCLCPP_INFO(this->get_logger(), "Homing all servos not implemented yet");
}

void DriverFeetechServo::HomeSingleServo(const int id)
{
  // Logging statements
  RCLCPP_INFO(this->get_logger(), "Homing servo %d", id);

  // Check servo mode and set to velocity if in position mode
  if (mServoData.servo_map[id].control_mode==POSITION_MODE)
  {
    RCLCPP_INFO(this->get_logger(), "Homing mode is POSITION_MODE, switching to velocity");
    setAllMode(VELOCITY_MODE);
  }

  // Switch-based homing -- TO DO: IMPLEMENT load based protection
  if (mServoData.servo_map[id].homing_mode==SWITCH_BASED)
  {
    if (digitalRead(mServoData.servo_map[id].limit_switch_pin)==HIGH) // Limit switch is pressed
    {
      RCLCPP_INFO(this->get_logger(), "Limit switch is pressed");
      
      // Move servo until limit switch is not pressed
      int result;
      int count;
      while(digitalRead(mServoData.servo_map[id].limit_switch_pin)==HIGH)
      {
        getSinglePresentPosition(id);
        result = setVelocityReference(id, -mHomeVelocity);
        if (result==-1){
          count++;
          getSinglePresentVoltage(id);
          RCLCPP_INFO(this->get_logger(), "Voltage: %f V", mServoData.servo_map[id].voltage/10.);
          if (count>10){
          break;}}
        sleep(0.1);
      }
      setVelocityReference(id, 0); // Stop the servo
    }

    // Limit switch is not pressed
    if (digitalRead(mServoData.servo_map[id].limit_switch_pin)==LOW)
    {
      RCLCPP_INFO(this->get_logger(), "Limit switch is not pressed");
      // Move servo until limit switch is pressed
      int result;
      int count;
      while(digitalRead(mServoData.servo_map[id].limit_switch_pin)==LOW)
      {
        getSinglePresentPosition(id);
        result = setVelocityReference(id, mServoData.servo_map[id].gear_ratio*mHomeVelocity);
        if (result==-1){
          count++;
          getSinglePresentVoltage(id);
          RCLCPP_INFO(this->get_logger(), "Voltage: %f V", mServoData.servo_map[id].voltage/10.);
          if (count>10){
          break;}}
        sleep(0.1);
      }

      // Switch high
      RCLCPP_INFO(this->get_logger(), "Limit switch is pressed, slowly moving back");
      while(digitalRead(mServoData.servo_map[id].limit_switch_pin)==HIGH)
      {
        getSinglePresentPosition(id);
        result = setVelocityReference(id, -mHomeVelocity);
        if (result==-1){
          getSinglePresentVoltage(id);
          RCLCPP_INFO(this->get_logger(), "Voltage: %f V", mServoData.servo_map[id].voltage/10.);
          break;}
        sleep(0.1);
      }
      
      // Finalise homing
      setVelocityReference(id, 0); // Stop the servo
      getSinglePresentPosition(id);
      mServoData.servo_map[id].home_position = mServoData.servo_map[id].position;
      RCLCPP_INFO(this->get_logger(), "Homing for %d finished", id);

      moveToRelativePosition(id, mServoData.servo_map[id].home_to_nominal);
    }
  }

  // Load-based homing
  else if (mServoData.servo_map[id].homing_mode==LOAD_BASED)
  {
    RCLCPP_INFO(this->get_logger(), "Load-based homing for servo ID %d", id);
    // Read present current and save
    getSinglePresentCurrent(id);
    int baseline_current = mServoData.servo_map[id].current;
    setVelocityReference(id, mServoData.servo_map[id].gear_ratio*mHomeVelocity); // Set velocity once

    // While current below threshold (divide by 6.5 to convert to mA, threshold specified in mA)
    int result;
    while (abs(mServoData.servo_map[id].current-baseline_current) < mCurrentThreshold)
    {
      result = getSinglePresentCurrent(id);
      RCLCPP_DEBUG(this->get_logger(), "Current diff (/6.5 to mA): %d", abs(mServoData.servo_map[id].current-baseline_current));
      if (result==-1){
        getSinglePresentVoltage(id);
        RCLCPP_INFO(this->get_logger(), "Voltage: %f V", mServoData.servo_map[id].voltage/10.);
        break;}
    }

    // Finalize homing
    setVelocityReference(id, 0); // Stop the servo
    getSinglePresentPosition(id);
    mServoData.servo_map[id].home_position = mServoData.servo_map[id].position;
    RCLCPP_INFO(this->get_logger(), "Homing for %d finished", id);

    moveToRelativePosition(id, mServoData.servo_map[id].home_to_nominal);
  }

  else if (mServoData.servo_map[id].homing_mode==PRESET)
  {
      // Set the current position as the home position
      getSinglePresentPosition(id);
      mServoData.servo_map[id].home_position = mServoData.servo_map[id].position;
      RCLCPP_INFO(this->get_logger(), "Set current position as home for %d", id);
  }

  sleep(1); // Wait 1 second
}

/*
 * Initialize Servos
 */
void DriverFeetechServo::InitializeServos()
{
  std::string message = "Initializing servos on "+std::string(mDeviceName)+" at "+std::to_string(BAUDRATE)+" baudrate";
  RCLCPP_INFO(this->get_logger(), message.c_str());
  // Set PortHandler and PacketHandler
  portHandler = dynamixel::PortHandler::getPortHandler(mDeviceName.c_str());
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  mCommResult = portHandler->openPort();
  if (mCommResult == false) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the port! Is the USB connected and correctly specified?");
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port
  mCommResult = portHandler->setBaudRate(BAUDRATE);
  if (mCommResult == false) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate! Is the USB connected and correctly specified?");
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set the baudrate.");
  }

  // add all the servos and populate data
  // === ARM 1 ===
  mServoData.AddServo(ServoState(
    this->get_parameter("pivot_1_id").as_int(), 
    0,                // Position
    0,                // Velocity
    0,                // Current
    0,                // Voltage   
    this->get_parameter("limit_1_pivot").as_int(), 
    0,                // Continuous position
    0,                // Home position
    -1.5,             // Ticks from home position to nominal position
    4.0,              // Gear ratio
    0.f,              // Absolute current position
    0.f,              // Absolute goal position
    0,                // Ticks to go
    PRESET, 
    POSITION_MODE));
  mServoData.AddServo(ServoState(
    this->get_parameter("shoulder_1_id").as_int(), 
    0,                // Position
    0,                // Velocity
    0,                // Current
    0,                // Voltage
    this->get_parameter("limit_1_shoulder").as_int(), 
    0,                // Continuous position
    0,                // Home position
    -0.65,             // Ticks from home position to nominal position
    1.0,              // Gear ratio
    0.f,              // Absolute current position
    0.f,              // Absolute goal position
    0,                // Ticks to go
    PRESET, 
    POSITION_MODE));
  mServoData.AddServo(ServoState(
    this->get_parameter("elbow_1_id").as_int(), 
    0,                // Position
    0,                // Velocity
    0,                // Current
    0,                // Voltage 
    -1,                // No limit switch
    0,                // Continuous position
    0,                // Home position
    -1.8,                // RAD from home position to nominal position
    2.0,              // Gear ratio
    0.f,              // Absolute current position
    0.f,              // Absolute goal position
    0,                // Ticks to go
    PRESET, 
    POSITION_MODE));
    // === ARM 2 ===
    mServoData.AddServo(ServoState(
      this->get_parameter("pivot_2_id").as_int(), 
      0,                // Position
      0,                // Velocity
      0,                // Current
      0,                // Voltage   
      this->get_parameter("limit_2_pivot").as_int(), 
      0,                // Continuous position
      0,                // Home position
      -1.5,             // Ticks from home position to nominal position
      4.0,              // Gear ratio
      0.f,              // Absolute current position
      0.f,              // Absolute goal position
      0,                // Ticks to go
      PRESET, 
      POSITION_MODE));
    mServoData.AddServo(ServoState(
      this->get_parameter("shoulder_2_id").as_int(), 
      0,                // Position
      0,                // Velocity
      0,                // Current
      0,                // Voltage
      this->get_parameter("limit_2_shoulder").as_int(), 
      0,                // Continuous position
      0,                // Home position
      -0.65,             // Ticks from home position to nominal position
      1.0,              // Gear ratio
      0.f,              // Absolute current position
      0.f,              // Absolute goal position
      0,                // Ticks to go
      PRESET, 
      POSITION_MODE));
    mServoData.AddServo(ServoState(
      this->get_parameter("elbow_2_id").as_int(), 
      0,                // Position
      0,                // Velocity
      0,                // Current
      0,                // Voltage 
      -1,                // No limit switch
      0,                // Continuous position
      0,                // Home position
      -1.8,                // RAD from home position to nominal position
      2.0,              // Gear ratio
      0.f,              // Absolute current position
      0.f,              // Absolute goal position
      0,                // Ticks to go
      PRESET, 
      POSITION_MODE));
  getAllPresentPositions();
  getAllPresentVelocities();
  getAllPresentCurrents();
  getAllPresentVoltages();

  // Set all servos to torque enable
  setAllEnable(DISABLED);

  // home the servos
  setAllMode(VELOCITY_MODE);
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
    HomeSingleServo(id);
  }
};

/* Get present (homed) position for servo ID and set on servo data struct
*/
int DriverFeetechServo::getSinglePresentPosition(const int id)
{
  // Read Present Position (length : 2 bytes) and Convert uint32 -> int32
  uint16_t data;
  mCommResult = packetHandler->read2ByteTxRx(
    portHandler,
    (uint8_t) mServoData.servo_map[id].id,
    ADDR_PRESENT_POSITION,
    &data,
    &mErrorCode
  );
  int homed_position = data-mServoData.servo_map[id].home_position;
  if (homed_position<0)
  {
    homed_position = 4096+homed_position;
  }

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get present position for ID %d. Error code %i", id, mErrorCode);
    return -1;
  } 
  else {
    RCLCPP_DEBUG(this->get_logger(), "Get [ID: %d] [Present position: %d ticks]",
    mServoData.servo_map[id].id,
    mServoData.servo_map[id].position);
    mServoData.servo_map[id].position = homed_position;
    return 0;
  }
};

int DriverFeetechServo::getSinglePresentVelocity(const int id)
{
  uint16_t data;
  // Read Present Velocity (length : 2 bytes)
  mCommResult = packetHandler->read2ByteTxRx(
    portHandler,
    (uint8_t) mServoData.servo_map[id].id,
    ADDR_PRESENT_SPEED,
    &data,
    &mErrorCode
  );
  
  // Check and convert sign
  int signedValue = data & ~0x8000;
  if (data & 0x8000)
      signedValue = -signedValue;
  mServoData.servo_map[id].velocity = signedValue;
  
  // Error handling
  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get present velocity for ID %d. Error code %i", id, mErrorCode);
    return -1;
  }
  else {
    RCLCPP_DEBUG(this->get_logger(), "Get [ID: %d] [Present velocity: %d ticks/s]",
    mServoData.servo_map[id].id,
    mServoData.servo_map[id].velocity);
    return 0;
  }
};

int DriverFeetechServo::getSinglePresentCurrent(const int id)
{
  uint16_t data;
  // Read present current (length: 2 bytes)
  mCommResult = packetHandler->read2ByteTxRx(
    portHandler,
    (uint8_t) mServoData.servo_map[id].id,
    ADDR_PRESENT_CURRENT,
    &data,
    &mErrorCode
  );

  // Check and convert sign
  int signedValue = data & ~0x8000;
  if (data & 0x8000)
      signedValue = -signedValue;
  mServoData.servo_map[id].current = signedValue;

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get present current for ID %d. Error code %i", id, mErrorCode);
    return -1;
  } 
  else {
    RCLCPP_DEBUG(this->get_logger(), "Get [ID: %d] [Present current: %f mA]",
    mServoData.servo_map[id].id,
    mServoData.servo_map[id].current*6.5);
    return 0;
  }

};

int DriverFeetechServo::getSinglePresentVoltage(const int id)
{
  // Read present voltage (length: 1 byte)
  uint8_t data;
  mCommResult = packetHandler->read1ByteTxRx(
    portHandler,
    (uint8_t) mServoData.servo_map[id].id,
    ADDR_PRESENT_VOLTAGE,
    &data,
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get present voltage for ID %d. Error code %i", id, mErrorCode);
    return -1;
  } 
  else {
    RCLCPP_DEBUG(this->get_logger(), "Get [ID: %d] [Present voltage: %f V]",
    mServoData.servo_map[id].id,
    data/10.0);
    mServoData.servo_map[id].voltage = data;
    return 0;
  }
};

void DriverFeetechServo::getAllPresentPositions()
{
  int previous_position;
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
      previous_position = mServoData.servo_map[id].position; // save the previous position
      getSinglePresentPosition(id); // get the new position (homed)

      int change_ticks;
      if (mServoData.servo_map[id].position-previous_position>3500) 
        {change_ticks = mServoData.servo_map[id].position-previous_position-4096;}
      else if (mServoData.servo_map[id].position-previous_position<-3500)
        {change_ticks = mServoData.servo_map[id].position-previous_position+4096;}
      else {change_ticks = mServoData.servo_map[id].position-previous_position;}

      // Calculate change in output to absolute current position and add to absolute current position
      mServoData.servo_map[id].absolute_current_position += change_ticks*2*M_PI/4096/mServoData.servo_map[id].gear_ratio;
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

void DriverFeetechServo::getAllPresentVoltages()
{
  for (auto& [id, servo] : mServoData.servo_map) {
    getSinglePresentVoltage(id);
  }
};

int DriverFeetechServo::setSingleMode(const int id, const ControlMode &mode)
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
    RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control Mode for ID %d. Error code %i", id, mErrorCode);
    return -1;
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set control mode of ID %d to %d.", id, mode);
    mServoData.servo_map[id].control_mode = mode;
    return 0;
  }
}

/* Set servo control mode
 * @param servo_id: ID of the servo
 * @param mode: 0 for position mode, 1 for velocity mode
 */
void DriverFeetechServo::setAllMode(const ControlMode &mode)
{
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
    // If setting to velocity mode, make sure reference velocity is first set to zero
    if (mode==VELOCITY_MODE)
    {
      setVelocityReference(id, 0);
    }
    setSingleMode(id, mode);
  }
};

int DriverFeetechServo::setPositionReference(const int id, const int &reference)
{
  mCommResult = packetHandler->write2ByteTxRx(
    portHandler,
    id,
    ADDR_GOAL_POSITION,
    reference,
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set position reference for ID %d. Error code %i", id, mErrorCode);
    return -1;
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Succeeded to set position reference for ID %d to %d ticks.", id, reference);
    return 0;
  }
};

int DriverFeetechServo::setVelocityReference(const int id, const int &reference)
{
  // Handle sign
  uint16_t ref = abs(reference);
  if (reference < 0) {
    ref = 0x8000 | ref;
  }
  
  // Write to servo
  mCommResult = packetHandler->write2ByteTxRx(
    portHandler,
    id,
    ADDR_GOAL_SPEED,
    ref,
    &mErrorCode
  );
  
  // Error handling
  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set velocity reference for ID %d. Error code %i", id, mErrorCode);
    return -1;
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Succeeded to set velocity reference of ID %d to %d ticks/s.", id, reference);
    return 0;
  }
};
 

int DriverFeetechServo::setSingleEnable(const int id, const TorqueEnable &enable)
{
  mCommResult = packetHandler->write1ByteTxRx(
    portHandler,
    (uint8_t) id,
    ADDR_TORQUE_ENABLE,
    enable,
    &mErrorCode
  );

  if (mCommResult != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set torque enable for ID %d. Error code %i", id, mErrorCode);
    return -1;
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set torque enable of ID %d to %i.", id, enable);
    return 0;
  }
}

void DriverFeetechServo::setAllEnable(const TorqueEnable &enable)
{
  for (auto& [id, servo] : mServoData.servo_map) {  // Use non-const reference
    setSingleEnable(id, enable);
  }
};

/* Move the joint to a relative position
 * @param id: ID of the servo
 * @param rel_position_rad: Relative position in radians
*/
void DriverFeetechServo::moveToRelativePosition(const int id, const double rel_position_rad)
{
  // Get current position
  getSinglePresentPosition(id);
  int previous_position = mServoData.servo_map[id].position;
  int current_position = previous_position;
  int goal_rel_position_ticks = rel_position_rad * mServoData.servo_map[id].gear_ratio * 4096 / (2*M_PI); // Move by this many ticks
  int ticks_to_go = goal_rel_position_ticks;
  int ticks_moved = 0;
  int velocity = 0;

  RCLCPP_DEBUG(this->get_logger(), "Moving servo %d to relative position %.3f rad", id, rel_position_rad);
  RCLCPP_DEBUG(this->get_logger(), "Ticks to go: %d", ticks_to_go);
  RCLCPP_DEBUG(this->get_logger(), "abs ticks to go: %d", abs(ticks_to_go));

  while(abs(ticks_to_go)>mPositionThreshold)
  {
    RCLCPP_DEBUG(this->get_logger(), "Ticks to go: %d", ticks_to_go);
    getSinglePresentPosition(id);
    current_position = mServoData.servo_map[id].position;
    // Calculate number of ticks moved
    ticks_moved = current_position - previous_position;

    // Deal with under- and overflow (enables handling multiple servo rotations)
    if (ticks_moved>3500) {ticks_moved=4096-ticks_moved;}
    else if (ticks_moved<-3500) {ticks_moved=4096+ticks_moved;}
    previous_position = current_position;

    // Get number of ticks to go
    ticks_to_go = ticks_to_go - ticks_moved;
    // Calculate saturated velocity
    velocity = (int)std::max(std::min(mPositionPGain*ticks_to_go + mPositionDGain*ticks_moved, mMaxVelocity), -mMaxVelocity);
    // Write velocity
    setVelocityReference(id, velocity);
  }
  // Stop servo
  setVelocityReference(id, 0);
}

int main(int argc, char * argv[])
{
  // Set up the gpio using wiringPi for limit switches
  wiringPiSetup();
  pinMode(LIMIT_PIVOT_1, INPUT);
  pinMode(LIMIT_PIVOT_2, INPUT);
  pinMode(LIMIT_SHOULDER_1, INPUT);
  pinMode(LIMIT_SHOULDER_2, INPUT);

  // Initialize ROS node
  rclcpp::init(argc, argv);
  auto driverfeetechservo = std::make_shared<DriverFeetechServo>();
  rclcpp::spin(driverfeetechservo);
  rclcpp::shutdown();

  return 0;
}
