cmake_minimum_required(VERSION 3.5)
project(balance_controller)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

add_executable(balance_controller_node src/balance_controller_node.cpp)
ament_target_dependencies(balance_controller_node rclcpp std_msgs sensor_msgs dynamixel_sdk)

install(TARGETS
  balance_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
