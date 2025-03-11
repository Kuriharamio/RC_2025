cmake_minimum_required(VERSION 3.22)
project(2d_lidar_driver)

set(DEPENDS 
  rclcpp
  sensor_msgs
)

include_directories(
  sdk/include/
)

add_library(2d_lidar_sdk SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/src/C3iroboticsLidar.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/src/CArcTime.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/src/CCountDown.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/src/CDeviceConnection.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/src/CLidarPacket.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/src/CLidarPacketReceiver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/src/CLidarUnpacket.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/src/CSerialConnection.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/src/CSimulateSerial.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/src/CTime.cpp
)

add_executable(2d_lidar_driver 
  ${CMAKE_CURRENT_SOURCE_DIR}/2d_lidar_driver.cpp
)

set(PROJECT_TARGETS 
  2d_lidar_driver 
)

foreach(target ${PROJECT_TARGETS})
  ament_target_dependencies(${target}
    ${DEPENDS}
  )
endforeach()

target_link_libraries(2d_lidar_driver 2d_lidar_sdk)

install(TARGETS 
  2d_lidar_sdk
  LIBRARY DESTINATION lib)

install(TARGETS 
  2d_lidar_driver
  DESTINATION lib/${SENSOR_BRIDGE_PACKAGE_NAME})