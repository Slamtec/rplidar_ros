^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rplidar_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.5 (2023-11-20)
------------------
* Update RPLIDAR SDK to 2.1.0
  * Support RPLIDAR C1 (`#142 <https://github.com/Slamtec/rplidar_ros/issues/142>`_)
  * Re-implemented the data retrieving logic based on async fetching and decoding mechanism to improve performance
  * UltraDense protocol support
  * support for stoppping A1 motor
  * bugfix:start/stop_motor(service) cause lidar to stop scanning
* Fix build with C++14 && --march=native
* scan frequency configuration support
* Add launch file for RPLIDAR a2m*,C1
* Install udev rules via debian. (`#126 <https://github.com/Slamtec/rplidar_ros/issues/126>`_)
* Bugfix:create_udev_rules.sh dose not take effect immediately.
* When node starts to reset rplidar, if rplidar info is not obtained within 15 seconds, rplidar reset fails
* Add initial_reset option to reset rplidar on node start
* Compilation optimization
* Contributors: Babak-SSh, Tim Clephas, Tony Baltovski, Ubuntu248, Victor Belov, Wang DeYou, WubinXia, kint, yzx

2.0.0 (2021-10-8)
-----------------
* Update RPLIDAR SDK to 2.0.0
* [new feature] 1.redesign the skelton of the sdk. 2.support Rplidar S2
* Contributors: tony,WubinXia

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
