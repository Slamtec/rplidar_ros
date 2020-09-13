cmake_minimum_required(VERSION 3.9.5)
project(rplidar_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -pedantic)
endif()

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

set(req_deps
  "rclcpp"
  "sensor_msgs"
  "std_srvs"
  "rclcpp_components"
)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

ament_auto_find_build_dependencies(REQUIRED ${req_deps})

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  include
)

# build composition node
add_library(rplidar_composition_node SHARED
  src/rplidar_node.cpp
  ${RPLIDAR_SDK_SRC}
)
ament_target_dependencies(rplidar_composition_node ${req_deps})
rclcpp_components_register_nodes(rplidar_composition_node "rplidar_ros::rplidar_node")

# build composition node
ament_auto_add_executable(rplidar_composition src/standalone_rplidar.cpp)
target_link_libraries(rplidar_composition rplidar_composition_node)
ament_target_dependencies(rplidar_composition ${req_deps})

install(
  TARGETS rplidar_composition
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install the shared library composition node
install(
  TARGETS rplidar_composition_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# TODO(hallen): port this
# install(DIRECTORY sdk
#   USE_SOURCE_PERMISSIONS
# )

ament_auto_package()
