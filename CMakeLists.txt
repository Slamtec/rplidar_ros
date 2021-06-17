################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(rplidar_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

set(RPLIDAR_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/sdk)

set(RPLIDAR_SDK_SRC 
  ${RPLIDAR_SDK_PATH}/src/hal/abs_rxtx.h
  ${RPLIDAR_SDK_PATH}/src/hal/assert.h
  ${RPLIDAR_SDK_PATH}/src/hal/byteops.h
  ${RPLIDAR_SDK_PATH}/src/hal/event.h
  ${RPLIDAR_SDK_PATH}/src/hal/locker.h
  ${RPLIDAR_SDK_PATH}/src/hal/socket.h
  ${RPLIDAR_SDK_PATH}/src/hal/thread.cpp
  ${RPLIDAR_SDK_PATH}/src/hal/thread.h
  ${RPLIDAR_SDK_PATH}/src/hal/types.h
  ${RPLIDAR_SDK_PATH}/src/hal/util.h

  ${RPLIDAR_SDK_PATH}/src/rplidar_driver.cpp
  ${RPLIDAR_SDK_PATH}/src/rplidar_driver_impl.h
  ${RPLIDAR_SDK_PATH}/src/rplidar_driver_serial.h
  ${RPLIDAR_SDK_PATH}/src/rplidar_driver_TCP.h
  ${RPLIDAR_SDK_PATH}/src/sdkcommon.h
)

if (UNIX)
    LIST(APPEND RPLIDAR_SDK_SRC
        ${RPLIDAR_SDK_PATH}/src/arch/linux/arch_linux.h
        ${RPLIDAR_SDK_PATH}/src/arch/linux/net_serial.cpp
        ${RPLIDAR_SDK_PATH}/src/arch/linux/net_serial.h
        ${RPLIDAR_SDK_PATH}/src/arch/linux/net_socket.cpp
        ${RPLIDAR_SDK_PATH}/src/arch/linux/thread.hpp
        ${RPLIDAR_SDK_PATH}/src/arch/linux/timer.cpp
        ${RPLIDAR_SDK_PATH}/src/arch/linux/timer.h
        )
elseif(WIN32)
    LIST(APPEND RPLIDAR_SDK_SRC
        ${RPLIDAR_SDK_PATH}/src/arch/win32/arch_win32.h
        ${RPLIDAR_SDK_PATH}/src/arch/win32/net_serial.cpp
        ${RPLIDAR_SDK_PATH}/src/arch/win32/net_serial.h
        ${RPLIDAR_SDK_PATH}/src/arch/win32/net_socket.cpp
        ${RPLIDAR_SDK_PATH}/src/arch/win32/timer.cpp
        ${RPLIDAR_SDK_PATH}/src/arch/win32/timer.h
        ${RPLIDAR_SDK_PATH}/src/arch/win32/winthread.hpp
        )
elseif(APPLE)
    LIST(APPEND RPLIDAR_SDK_SRC 
        ${RPLIDAR_SDK_PATH}/src/arch/macOS/arch_macOS.h
        ${RPLIDAR_SDK_PATH}/src/arch/macOS/net_serial.cpp
        ${RPLIDAR_SDK_PATH}/src/arch/macOS/net_serial.h
        ${RPLIDAR_SDK_PATH}/src/arch/macOS/net_socket.cpp
        ${RPLIDAR_SDK_PATH}/src/arch/macOS/thread.hpp
        ${RPLIDAR_SDK_PATH}/src/arch/macOS/timer.cpp
        ${RPLIDAR_SDK_PATH}/src/arch/macOS/timer.h        
        )
endif()


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
)

add_executable(rplidar_scan_publisher	src/rplidar_scan_publisher.cpp ${RPLIDAR_SDK_SRC})
# target_link_libraries(rplidar_scan_publisher ${ament_cmake_LIBRARIES})
ament_target_dependencies(rplidar_scan_publisher
  rclcpp
  std_srvs
  sensor_msgs
)

################################################################################
# Install
################################################################################
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS rplidar_scan_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(include)
ament_package()
