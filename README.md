# Feetech ROS2 package
ROS2 wrapper for commanding and reading Feetech servos (STS/SCS models) using the [Feetech CPP Library](https://github.com/BioMorphic-Intelligence-Lab/feetech_cpp_lib). Uses the Feetech CPP Library in ROS2 nodes to command servos by publishing servo commands on ROS2 topics.

This package assumes that you connect the servos using the URT board (a full duplex to half duplex adapter), which interfaces using USB with the companion computer.

## How to use
You can create your own nodes by adapting from the `feetech_ros2_interface.cpp` definition. Configurations for the nodes are stored in the files under `config`, which are then passed using ROS2 launch files. The node, driver, and servos have the following parameters that can be modified. In case a parameter is not specified, it will take a default value.s

| **Type** | **Name**           | **Description**                                                                                                                                                                        |   |   |
|----------|--------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|---|
| Node     | frequency          | Frequency in Hz that the node runs at                                                                                                                                                  |   |   |
| Driver   | port_name          | Name of the interface port on the computer                                                                                                                                             |   |   |
|          | baud_rate          | Baud rate (bps) of communication with servos                                                                                                                                           |   |   |
|          | frequency          | Frequency in Hz that the internal driver loop runs at (default 100 Hz)                                                                                                                 |   |   |
| Servo    | ids                | List of servo IDs on the communication bus                                                                                                                                             |   |   |
|          | operating_modes    | List of operating modes for the servos, corresponding in order with ids. Operating mode 0 is position, 2 is velocity                                                                   |   |   |
|          | homing_modes       | List of homing modes for the servos. Homing mode 0 sets home at location at startup.                                                                                                   |   |   |
|          | directions         | List of servo directions. 1 for clockwise and -1 for counter-clockwise.                                                                                                                |   |   |
|          | max_speeds         | List of maximum output speeds in rad/s.                                                                                                                                                |   |   |
|          | max_currents       | List of maximum currents in mA.                                                                                                                                                        |   |   |
|          | gear_ratios        | List of gear ratios from servo horn to output (in case additional gearing is used). If the gearing reduces horn speed by 2 at the output, gear ratio is 2. In case of doubt, set to 1. |   |   |
|          | start_offsets        | List of offsets in radians from the zero position to where the servo is when initialized. Set to 0 if not applicable. |   |   |


## Credits
- Anton Bredenbeck ([@antbre](https://github.com/antbre), Delft University of Technology)
- Martijn Brummelhuis ([@mbrummelhuis](https://github.com/mbrummelhuis), Delft University of Technology)

## License
MIT license
