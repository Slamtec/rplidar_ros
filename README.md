RPLIDAR ROS package
=====================================================================

ROS node and test application for RPLIDAR

Visit RoboPeak Website for more details about RPLIDAR.

How to build rplidar ros package
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build rplidarNode and rplidarNodeClient

How to run rplidar ros package
=====================================================================
There're two ways to run rplidar ros package

I. Run rplidar node and view in the rviz
------------------------------------------------------------
roslaunch rplidar_ros view_rplidar.launch

You should see rplidar's scan result in the rviz.

II. Run rplidar node and view using test application
------------------------------------------------------------
roslaunch rplidar_ros rplidar.launch

rosrun rplidar_ros rplidarNodeClient

You should see rplidar's scan result in the console

RPLidar frame
=====================================================================
RPLidar frame must be broadcasted according to picture shown in
rplidar-frame.png