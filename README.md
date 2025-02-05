# feetech_ros2_driver
ROS2 wrapper for commanding and reading Feetech servos

This package assumes that you connect the servos using the URT board (a full duplex to half duplex adapter), which interfaces using USB with the companion computer.

## Dependencies
### Dynamixel SDK
It also requires you to clone [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) into your ROS2 workspace and build it.

Example folder structure:
ros2_ws
├── _config.yml
├── src
│   ├── feetech_ros2_driver <-- this repository
│   └── DynamixelSDK <-- DynamixelSDK repository
│       ├── dynamixel_sdk
│       ├── dynamixel_sdk_examples (not strictly necessary)
│       └── dynamixel_sdk_custom_interfaces (not strictly necessary)

### WiringPi (WiringOP)
To interface with the companion computer (Orange Pi 5) GPIO I'm using the [wiringOP](https://github.com/orangepi-xunlong/wiringOP) library, which you can just install according to instructions. After installing use `gpio readall` to find the pin allocation for your particular board. The pins in the code follow the wPi column of the output. 
The ROS2 node can be compiled against this library by including it in the CMakeLists.txt (see the CMakeLists.txt file in the repository). The ROS2 node needs to be run as root for the GPIO to be accessible, I have not yet found a workaround for this.
