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

#ifndef FEETECH_ROS2_DRIVER_HPP_
#define FEETECH_ROS2_DRIVER_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

// Control table addresses for Feetech STS
#define ADDR_OPERATING_MODE 33 // 0: position, 1: velocity
#define ADDR_TORQUE_ENABLE 40 // 0: torque disable, 1: torque enable
#define ADDR_GOAL_POSITION 42
#define ADDR_PRESENT_POSITION 56
#define ADDR_GOAL_SPEED 46
#define ADDR_PRESENT_SPEED 58
#define ADDR_PRESENT_CURRENT 69

// Controller gains
#define ADDR_POSITION_P 21
#define ADDR_POSITION_D 22
#define ADDR_POSITION_I 23
#define ADDR_VELOCITY_P 37
#define ADDR_VELOCITY_I 39

// Protocol version
#define PROTOCOL_VERSION 1.0  // Feetech uses Dynamixel protocol v1.0

// Default setting
#define BAUDRATE 1000000  // Servo Baudrate
#define DEVICE_NAME "/dev/ttyUSB1"  // [Linux]: "/dev/ttyUSB*"

// Modes
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

enum ControlMode {
  POSITION_MODE = 0,
  VELOCITY_MODE = 1
};

enum HomingMode {
  LOAD_BASED = 0,   // Home based on load increase
  SWITCH_BASED = 1  // Home based on limit switch
};

enum TorqueEnable {
  ENABLED = 1,
  DISABLED = 0
};

class DriverFeetechServo : public rclcpp::Node
{
public:
  DriverFeetechServo();
  ~DriverFeetechServo();

  // Define struct for saving state of single servo
  struct ServoState {
    int id;
    int position;
    int velocity;
    int current;
    int limit_switch_pin;
    int continuous_position;
    int home_position;
    HomingMode homing_mode;
    ControlMode control_mode;

    ServoState(
      int id = 0, 
      int position = 0, 
      int velocity = 0, 
      int current = 0, 
      int limit_switch_pin = 0,
      int continuous_position = 0,
      int home_position = 0,
      HomingMode homing_mode = LOAD_BASED, 
      ControlMode control_mode = POSITION_MODE)
        : id(id), 
        position(position), 
        velocity(velocity), 
        current(current),
        limit_switch_pin(limit_switch_pin),
        continuous_position(continuous_position),
        home_position(home_position),
        homing_mode(homing_mode),
        control_mode(control_mode) {}
  };

  // Define struct for saving state of all servos
  struct ServoData {
    // unordered map allows accessing servos by ID: servo_data.servos[PIVOT_1_ID]
    std::unordered_map<int, ServoState> servo_map;

    ServoData() {}
    void AddServo(ServoState servo) {
      servo_map[servo.id] = servo;
    }
    
  } mServoData;

private:
  // functions
  void InitializeServos();
  void HomeSingleServo(const int id);
  void HomeAll();
  void timerCallback();

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_servo_position_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_servo_velocity_publisher_;

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr reference_servo_position_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr reference_servo_velocity_subscriber_;

  // callbacks
  void referenceServoPositionCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void referenceServoVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  // servo data getters
  void getSinglePresentPosition(const int id);
  void getSinglePresentVelocity(const int id);
  void getSinglePresentCurrent(const int id);
  void getAllPresentPositions();
  void getAllPresentVelocities();
  void getAllPresentCurrents();

  // general servo data setters
  void setSingleMode(const int id, const ControlMode &mode);
  void setAllMode(const ControlMode &mode);

  // individual servo data setters
  void setSingleEnable(const int id, const TorqueEnable &enable);
  void setAllEnable(const TorqueEnable &enable);
  void setPositionReference(const int id, const int &reference);
  void setVelocityReference(const int id, const int &reference);

  // publish and subscribe functions
  void PublishServoData();
  
  // Handlers
  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;

  // ??
  uint8_t mErrorCode;
  int mCommResult;
  const int mHomePositionIncrement;
  const int mCurrentThreshold;
  int mNodeFrequency;
  rclcpp::TimerBase::SharedPtr mTimer;
};

#endif  // FEETECH_ROS2_DRIVER_HPP_
