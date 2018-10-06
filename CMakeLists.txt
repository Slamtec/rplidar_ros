cmake_minimum_required(VERSION 3.9.5)
project(rplidar_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

set(req_deps
  "rclcpp"
#  "rosconsole"
  "sensor_msgs"
)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

ament_auto_find_build_dependencies(REQUIRED ${req_deps})

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
)

ament_auto_add_executable(rplidarNode src/node.cpp ${RPLIDAR_SDK_SRC})
ament_target_dependencies(rplidarNode ${req_deps})

ament_auto_add_executable(rplidarNodeClient src/client.cpp)
ament_target_dependencies(rplidarNodeClient ${req_deps})

install(
  TARGETS rplidarNode rplidarNodeClient
  DESTINATION lib/${PROJECT_NAME}
)

# TODO(hallen): port this
# install(DIRECTORY launch rviz sdk
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   USE_SOURCE_PERMISSIONS
# )

ament_auto_package()
