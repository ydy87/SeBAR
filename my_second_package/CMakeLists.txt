cmake_minimum_required(VERSION 3.5)
project(my_second_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(prototype src/prototype.cpp)
ament_target_dependencies(prototype rclcpp std_msgs)
ament_target_dependencies(prototype rclcpp std_msgs dynamixel_sdk)  
ament_target_dependencies(prototype rclcpp std_msgs sensor_msgs)
install(TARGETS
  prototype
  DESTINATION lib/${PROJECT_NAME})

ament_package()
