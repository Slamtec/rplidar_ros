cmake_minimum_required(VERSION 2.8.3)
project(rplidar_ros)

set(RPLIDAR_SDK_PATH "../../rplidar_sdk_v1.4.1/sdk/")
find_library(RPLIDAR_SDK rplidar_sdk ${RPLIDAR_SDK_PATH}/output/Linux/Release)
if(NOT RPLIDAR_SDK)
  message(FATAL_ERROR "Couldn't find librplidar_sdk.a")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
)

include_directories(
  ${RPLIDAR_SDK_PATH}/sdk/include
)

catkin_package()

add_executable(rplidarNode src/node.cpp)
target_link_libraries(rplidarNode ${RPLIDAR_SDK} ${catkin_LIBRARIES})

add_executable(rplidarNodeClient src/client.cpp)
target_link_libraries(rplidarNodeClient ${catkin_LIBRARIES})
