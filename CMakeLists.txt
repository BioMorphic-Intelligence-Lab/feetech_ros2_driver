cmake_minimum_required(VERSION 3.8)
project(driver_feetech_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(wiringPi REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(driver_feetech_servo src/driver_feetech_servo.cpp)
ament_target_dependencies(driver_feetech_servo 
  rclcpp 
  geometry_msgs
  dynamixel_sdk
  )

include_directories(
  include
)

# link wiringPi library
target_link_libraries(driver_feetech_servo
    wiringPi
)

install(TARGETS
  driver_feetech_servo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
