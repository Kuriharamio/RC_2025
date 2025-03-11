cmake_minimum_required(VERSION 3.22)
project(sensor_bridge)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPLIE_COMMANDS ON)
set(SENSOR_BRIDGE_PACKAGE_NAME ${PROJECT_NAME})

include_directories(
  src/communicator/include
  src/sensor_driver/2d_lidar_driver
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs)
find_package(tf2)
find_package(tf2_ros)

# add_subdirectory(src/communicator)
add_subdirectory(src/sensor_driver/2d_lidar_driver)


# install(DIRECTORY

# )

ament_package()
