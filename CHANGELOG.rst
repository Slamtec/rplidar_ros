^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rplidar_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.2 (2023-06-12)
------------------
* Support RPLIDAR S3
* Add maintainer members
* Contributors: Wang DeYou

2.1.1 (2023-05-22)
------------------
* change package.xml
* add sdk/src/hal/assert.h again
* delete sdk/src/hal/assert.h
* Update create_udev_rules.sh and README.md
* create_udev_rule.sh bug fix in ros2
* Change the SLLidar in the log to RPLidar
* Update readme, improve the usage of rplidar_ros package.
* add launch file for a2m*
* Fix sllidar_s1_launch.py:closing parenthesis ']' does not match opening parenthesis '{' on line 56 (sllidar_s1_launch.py, line 61)
* Eliminate a compilation warnings.Close `#113 <https://github.com/Slamtec/rplidar_ros/issues/113>`_
* Eliminate some compilation warnings
* Change the package name to rplidar_ros
* fix compile warnings
* updated README.md
* add ROS2 support
* bugfix:start/stop_motor(service) cause lidar to stop scanning
* upgrade sdk 1.12.0
* bugfix:angle_compensate_nodes will be out of range if angle >= 359.25
* Support TCP
* upgrade sdk 1.10.0
* upgrade sdk 1.9.0
  [new feature] support baudrate 57600 and 1382400, support HQ scan response
  [bugfix] TCP channel doesn't work
  [improvement] Print warning messages when deprecated APIs are called; imporve angular accuracy for ultra capsuled scan points
* [bugfix]modify scan_mode at test_rplidar.launch and test_rplidar_a3.launch
* Contributors: Tony Huang, Wang DeYou, WubinXia, deyou wang, haozhou.wong, kint, wubin.xia, yzx

1.7.0 (2018-07-19)
------------------
* [release] rplidar_ros release 1.7.0
* improvements: upgrade RPLIDAR SDK to 1.7.0
  new feature: support scan points farther than 16.38m
* new feature: lidar scan mode selection; change: use Stability as the default mode for RPLIDAR A3
* fixed angle_compensate_multiple is zero for A1@2K
  fixed angle_compensate_multiple is zero for A1 work at 2K mode
* [upgrade]upgrade the SDK to 1.6.0, and support RPLIDAR A3
* [`#51 <https://github.com/Slamtec/rplidar_ros/issues/51>`_]fixed scan_duration to standard [seconds]
* Contributors: Tony Huang, kint.zhao, kintzhao

1.5.7 (2016-12-15)
------------------
* update sdk to 1.5.7, fixed the motor default speed at 10 HZ.
* Extend the measurement of max_distance from 6m to 8m
* Contributors: kintzhao

1.5.5 (2016-08-24)
------------------
* add RPLIDAR information print,and fix the standard motor speed of RPLIDAR A2
* change rplidar_A1.png
* add new picture for install and fixed the default param of inverted and angle_compensate
* Update README.md
  add three website  about  rplidar: roswiki, homepage, tutorial.
* Contributors: kint.zhao, kintzhao

1.5.4 (2016-06-02)
------------------
* updated to SDK 1.5.4
* Contributors: kint.zhao

1.5.2 (2016-04-29)
------------------
* update CHANGELOG
* update to RPLIDAR SDK 1.5.2
  Add RPLIDAR A2 support in ROS node
  Add helper scripts and launch file for RPLIDAR
* Merge pull request `#16 <https://github.com/Slamtec/rplidar_ros/issues/16>`_ from yujinrobot/catkin_fix
  Include catkin directories
* Merge pull request `#19 <https://github.com/Slamtec/rplidar_ros/issues/19>`_ from yujinrobot/publish_reverse_scan
  Reverse the scan publisher if max < min.
* remove processing cost for reversing the published scan
* reverse the scan publisher if max < min.
* include catkin directories
  Without this, it won't build without sourcing setup.bash first.
  Note that this will mean it fails the first time you try to release
  it as a deb on the osrf build farm as it is a sandboxed environment.
* Merge pull request `#10 <https://github.com/Slamtec/rplidar_ros/issues/10>`_ from negre/master
  Start / Stop motor service
* Merge pull request `#9 <https://github.com/Slamtec/rplidar_ros/issues/9>`_ from afrancescon/master
  Fix RPLidar and GMapping incompatibility
* Merge pull request `#7 <https://github.com/Slamtec/rplidar_ros/issues/7>`_ from k-okada/add_install
  add install targets
* Merge pull request `#4 <https://github.com/Slamtec/rplidar_ros/issues/4>`_ from jlblancoc/master
  Fix stack corruption in Windows 64bit
* remove commented lines
* add motor start/stop service
* fixed rplidar-frame.png
* added documentation about rplidar frame
* fixed angle min and max on published laser scan message
* quality used as intensity in sensor_msgs/LaserScan
* add install targets
* Fix stack corruption in Windows 64bit
* Merge pull request `#1 <https://github.com/Slamtec/rplidar_ros/issues/1>`_ from pal-robotics/master
  Fix memory leak, unitialized vars, typos and follow REP 117
* Replace 0.0 (max range) readings with Inf
  This is required to follow REP 117
  http://www.ros.org/reps/rep-0117.html
* Fix typos
* Fixed cpp check warnings: memory leak and uninitialized vars
* Contributors: Alessandro Francescon, Amaury Nègre, Daniel Stonier, Jose-Luis Blanco-Claraco, Kei Okada, RoboPeak Public Repos, Sammy Pfeiffer, kint.zhao, negre

1.0.1 (2014-06-03)
------------------
* Add CHANGELOG
* Add angle compensate mechanism to compatible with ROS scan message
* use ascendScanData before publish scan.
  Some dos2unix convert
* Add RPLIDAR sdk to the repo.
* Merge branch 'master' of https://github.com/robopeak/rplidar_ros
* First release of RPLIDAR ROS package
* First release of RPLIDAR ROS package
* Initial commit
* Contributors: =, Ling, RoboPeak Public Repos
