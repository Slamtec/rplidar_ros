^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rplidar_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Update README
* Add launch file for rplidar C1
* Update SDK.
  * Re-implemented the data retrieving logic based on async fetching and decoding mechanism to improve performance
  * RPLIDAR C1 support
* Contributors: deyou wang

2.1.4 (2023-08-15)
------------------
* Update SDK.
  * UltraDense protocol support
  * support for stoppping A1 motor
  * Optimize lidar driver for switching S2E workingmode
* Bugfix:logical error
* Install udev rules via debian.
* Contributors: Tony Baltovski, deyou wang

2.1.3 (2023-07-20)
------------------
* Bugfix:auto standby mode not work
* Rename SLLidar* to RPLidar* in rplidar_node.cpp
* Renaming variable m_running to is_scanning
* Update README
* Use rplidar_ros.rviz instead of rplidar.rviz
* Update source files in src directory:
  Use the source files developed by Slamtec instead of the old ones, and add the functionality of the old code.
* Modify and add launch files
* Update rplidar-sdk to 2.0.0
* Update create_udev_rules.sh
* Update description in package.xml
* Update maintainer to Wang DeYou
* Contributors: Wang DeYou

2.1.0 (2022-09-06)
------------------
* Add auto standby mode (`#29 <https://github.com/allenh1/rplidar_ros/issues/29>`_)
  * Add auto standby mode
  Turn on/off motor based on topic subsribers
  * Set auto_standby off by default
* Fix building on Apple machines (`#30 <https://github.com/allenh1/rplidar_ros/issues/30>`_)
* Update README & fix launch files for Foxy and up (`#26 <https://github.com/allenh1/rplidar_ros/issues/26>`_)
  * Update README.md
  based on modifications from youngday
  * Update launch files for Foxy or later
* Contributors: Jesse Zhang, Vasily Kiniv

2.0.3 (2022-09-01)
------------------
* Fix build with later versions of GCC
* Contributors: Hunter L. Allen

2.0.2 (2021-05-27)
------------------
* Remove test_rplidar.launch.py, since relevant executables no longer exist (`#24 <https://github.com/allenh1/rplidar_ros/issues/24>`_)
* Contributors: Hunter L. Allen

2.0.1 (2020-09-13)
------------------
* Remove old driver (`#21 <https://github.com/allenh1/rplidar_ros/issues/21>`_)
  * Remove old rplidar driver in favor of the component version
  * Lint the source
* Fix incompatibilities with slam_toolbox (`#20 <https://github.com/allenh1/rplidar_ros/issues/20>`_)
  * Fix incompatibilities with slam_toolbox:
  - Fix angle compensate mode to publish angle compensated values
  - Fix angle_increment calculation
  - Add optional flip_x_axis option to deal with issue discussed here: https://github.com/SteveMacenski/slam_toolbox/issues/198.  Flip x-axis can be used when laser is mounted with motor behind it as rotated TF laser frame doesn't seem to work with slam_toolbox.
  * Fix whitespace
* Fix node count for component implementation (`#19 <https://github.com/allenh1/rplidar_ros/issues/19>`_)
* Slam Toolbox compatibility (`#18 <https://github.com/allenh1/rplidar_ros/issues/18>`_)
  (cherry picked from commit f21079fea8eca8946b5b4ae72f50b8d9f1ac46a2)
* Fix building with GCC 10 (`#17 <https://github.com/allenh1/rplidar_ros/issues/17>`_)
* Contributors: Christen Lofland, Hunter L. Allen, justinIRBT

2.0.0 (2020-07-15)
------------------
* Update SDK to Version 0.12.0 (`#14 <https://github.com/allenh1/rplidar_ros/issues/14>`_)
  * Register the rclcpp component
  * Update RPLIDAR SDK to version 1.12.0
* Update ROS 2 parameters and use node's clock instance (`#9 <https://github.com/allenh1/rplidar_ros/issues/9>`_)
  * Update ROS 2 parameters and use node's clock instance
  * Fix scan_mode listing output
  * Stop motors and exit when set_scan_mode() call fails
* Fix compilation with eloquent (`#6 <https://github.com/allenh1/rplidar_ros/issues/6>`_)
* Use Composition node with launch files (`#4 <https://github.com/allenh1/rplidar_ros/issues/4>`_)
* Composable nodes (`#3 <https://github.com/allenh1/rplidar_ros/issues/3>`_)
  * Begin implementation of composable rplidar_ros::rplidar_node
  * Declare composition node library in CMake, as well as continue the port
  * Get to a compiling state
  * Add start/stop motor callbacks + more driver setup
  * Add publish loop for scans
  * Add composition node
  * Lint
* Port rviz and launch files to ROS2 (`#2 <https://github.com/allenh1/rplidar_ros/issues/2>`_)
  * Port non-rviz launch files to ROS2
  * Compatibility with rviz2
  * revert whitespace changes
  * Port the remaining launch files to ROS2
  * Revert more whitespace changes
  * Fix luanch and rviz install path indent level
* Ros2 port (`#1 <https://github.com/allenh1/rplidar_ros/issues/1>`_)
  ROS 2 port
  * Port CMakeLists.txt
  * Port package.xml
  * Port client.cpp
  * Port node.cpp
  Fix compilation
* Support TCP
* upgrade sdk 1.10.0
* upgrade sdk 1.9.0
  [new feature] support baudrate 57600 and 1382400, support HQ scan response
  [bugfix] TCP channel doesn't work
  [improvement] Print warning messages when deprecated APIs are called; imporve angular accuracy for ultra capsuled scan points
* [bugfix]modify scan_mode at test_rplidar.launch and test_rplidar_a3.launch
* Contributors: Dan Rose, Hunter L. Allen, WubinXia, kint

1.10.0 (2019-02-22)
-------------------
* Update RPLIDAR SDK to 1.10.0
* [new feature] support Rplidar S1
* Contributors: tony,WubinXia

1.9.0 (2018-08-24)
------------------
* Update RPLIDAR SDK to 1.9.0
* [new feature] support baudrate 57600 and 1382400, support HQ scan response
* [bugfix] TCP channel doesn't work
* [improvement] Print warning messages when deprecated APIs are called; imporve angular accuracy for ultra capsuled scan points
* Contributors: tony,kint

1.7.0 (2018-07-19)
------------------
* Update RPLIDAR SDK to 1.7.0
* support scan points farther than 16.38m
* upport display and set scan mode
* Contributors: kint

1.6.0 (2018-05-21)
------------------
* Release 1.6.0.
* Update RPLIDAR SDK to 1.6.0
* Support new product RPLIDAR A3(default 16K model and max_distance 25m)
* Contributors: kint

1.5.7 (2016-12-15)
------------------
* Release 1.5.7.
* Update RPLIDAR SDK to 1.5.7
* Fixed the motor default speed at 10 HZ. Extend the measurement of max_distance from 6m to 8m.
* Contributors: kint

1.5.5 (2016-08-23)
------------------
* Release 1.5.5.
* Update RPLIDAR SDK to 1.5.5
* Add RPLIDAR information print, and fix the standard motor speed of RPLIDAR A2.
* Contributors: kint

1.5.4 (2016-06-02)
------------------
* Release 1.5.4.
* Update RPLIDAR SDK to 1.5.4
* Support RPLIDAR A2
* Contributors: kint

1.5.2 (2016-04-29)
------------------
* Release 1.5.2.
* Update RPLIDAR SDK to 1.5.2
* Support RPLIDAR A2
* Contributors: kint

1.0.1 (2014-06-03)
------------------
* Release 1.0.1.
* Add angle compensate mechanism to compatible with ROS scan message
* Add RPLIDAR sdk to the repo.
* First release of RPLIDAR ROS package (1.0.0)
* Initial commit
* Contributors: Ling, RoboPeak Public Repos
