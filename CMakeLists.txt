cmake_minimum_required(VERSION 3.8)
project(feetech_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(wiringPi REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(feetech_ros2_driver src/feetech_ros2_driver.cpp)
ament_target_dependencies(feetech_ros2_driver 
  rclcpp 
  geometry_msgs
  dynamixel_sdk
  )

include_directories(
  include
)

# link wiringPi library
target_link_libraries(feetech_ros2_driver
    wiringPi
)

install(TARGETS
feetech_ros2_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
