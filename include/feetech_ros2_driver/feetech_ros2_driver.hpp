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
#define ADDR_PRESENT_VOLTAGE 62 // [0.1V]

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

// Modes
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

enum ControlMode {
  POSITION_MODE = 0,
  VELOCITY_MODE = 1
};

enum HomingMode {
  LOAD_BASED = 0,   // Home based on load increase
  SWITCH_BASED = 1, // Home based on limit switch
  PRESET = 2        // Home based on taking initial position as home
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
    int voltage;
    int limit_switch_pin;
    int continuous_position;
    int home_position;
    double home_to_nominal;
    float gear_ratio;
    float rad_per_tick;
    float absolute_current_position; // rad
    float absolute_goal_position; // rad
    int ticks_to_go;
    HomingMode homing_mode;
    ControlMode control_mode;

    ServoState(
      int id = 0, 
      int position = 0, 
      int velocity = 0, 
      int current = 0, 
      int voltage = 0,
      int limit_switch_pin = 0,
      int continuous_position = 0,
      int home_position = 0,
      double home_to_nominal = 0,
      float gear_ratio = 1.0,
      float absolute_current_position = 0.f,
      float absolute_goal_position = 0.f,
      int ticks_to_go = 0,
      HomingMode homing_mode = LOAD_BASED, 
      ControlMode control_mode = POSITION_MODE)
        : id(id), 
        position(position), 
        velocity(velocity), 
        current(current),
        voltage(voltage),
        limit_switch_pin(limit_switch_pin),
        continuous_position(continuous_position),
        home_position(home_position),
        home_to_nominal(home_to_nominal),
        gear_ratio(gear_ratio),
        absolute_current_position(absolute_current_position),
        absolute_goal_position(absolute_goal_position),
        ticks_to_go(ticks_to_go),
        homing_mode(homing_mode),
        control_mode(control_mode) {
          rad_per_tick = 2*M_PI/4096/gear_ratio;
        }
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
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr current_servo_position_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr current_servo_velocity_publisher_;

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr reference_servo_position_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr reference_servo_velocity_subscriber_;

  // callbacks
  void referenceServoPositionCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void referenceServoVelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  // servo data getters
  int getSinglePresentPosition(const int id);
  int getSinglePresentVelocity(const int id);
  int getSinglePresentCurrent(const int id);
  int getSinglePresentVoltage(const int id);
  void getAllPresentPositions();
  void getAllPresentVelocities();
  void getAllPresentCurrents();
  void getAllPresentVoltages();

  // general servo data setters
  int setSingleMode(const int id, const ControlMode &mode);
  void setAllMode(const ControlMode &mode);

  // individual servo data setters
  int setSingleEnable(const int id, const TorqueEnable &enable);
  void setAllEnable(const TorqueEnable &enable);
  int setPositionReference(const int id, const int &reference);
  int setVelocityReference(const int id, const int &reference);

  // publish and subscribe functions
  void PublishServoData();

  // Joint functions
  void moveToRelativePosition(const int id, const double rel_position_rad);
  void tMoveToAbsolutePosition(); // command all servos to move the the set absolute position goal via velocity mode
  void moveToAbsolutePosition(const int id, const double abs_position_rad);
  
  // Handlers
  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;

  // ??
  uint8_t mErrorCode;
  int mCommResult;
  int mHomeVelocity;
  std::string mDeviceName;
  const double mMaxVelocity;
  const int mCurrentThreshold;
  int mNodeFrequency;
  double mPositionPGain;
  double mPositionDGain;
  int mPositionThreshold;
  std::string mNamespace;
  rclcpp::TimerBase::SharedPtr mTimer;
};

#endif  // FEETECH_ROS2_DRIVER_HPP_
