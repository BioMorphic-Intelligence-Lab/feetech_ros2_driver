cmake_minimum_required(VERSION 3.8)
project(feetech_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(feetech_cpp_lib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMode.srv"
)

add_executable(feetech_ros2_interface src/feetech_ros2_interface.cpp)
target_include_directories(feetech_ros2_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/feetech_ros2_interface>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(feetech_ros2_interface 
  rclcpp 
  feetech_cpp_lib
  sensor_msgs
)

install(TARGETS
  feetech_ros2_interface
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(feetech_ros2_interface
  ${cpp_typesupport_target})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
