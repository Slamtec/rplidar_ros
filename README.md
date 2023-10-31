# RPLIDAR ROS package

ROS node and test application for RPLIDAR

Visit following Website for more details about RPLIDAR:

rplidar roswiki: <http://wiki.ros.org/rplidar>

rplidar HomePage: <http://www.slamtec.com/en/Lidar>

rplidar SDK: <https://github.com/Slamtec/rplidar_sdk>

rplidar Tutorial: <https://github.com/robopeak/rplidar_ros/wiki>

## How to build rplidar ros package

   1) Clone this project to your catkin's workspace src folder
   2) Running catkin_make to build rplidarNode and rplidarNodeClient

## How to run rplidar ros package

There're two ways to run rplidar ros package

### I. Run rplidar node and view in the rviz

The command for RPLIDAR A1 is :

```bash
roslaunch rplidar_ros view_rplidar_a1.launch
```

The command for RPLIDAR A2M7 is :

```bash
roslaunch rplidar_ros view_rplidar_a2m7.launch
```

The command for RPLIDAR A2M8 is :

```bash
roslaunch rplidar_ros view_rplidar_a2m8.launch
```

The command for RPLIDAR A2M12 is :

```bash
roslaunch rplidar_ros view_rplidar_a2m12.launch
```

The command for RPLIDAR A3 is :

```bash
roslaunch rplidar_ros view_rplidar_a3.launch
```

The command for RPLIDAR S1 is :

```bash
roslaunch rplidar_ros view_rplidar_s1.launch
```

The command for RPLIDAR S2 is :

```bash
roslaunch rplidar_ros view_rplidar_s2.launch
```

The command for RPLIDAR S3 is :

```bash
roslaunch rplidar_ros view_rplidar_s3.launch
```

The command for RPLIDAR S2E is :

```bash
roslaunch rplidar_ros view_rplidar_s2e.launch
```

The command for RPLIDAR T1 is :

```bash
roslaunch rplidar_ros view_rplidar_t1.launch
```

The command for RPLIDAR C1 is :

```bash
roslaunch rplidar_ros view_rplidar_c1.launch
```

You should see rplidar's scan result in the rviz.

### II. Run rplidar node and view using test application

The command for RPLIDAR A1 is :

```bash
roslaunch rplidar_ros rplidar_a1.launch
```

The command for RPLIDAR A2M7 is :

```bash
roslaunch rplidar_ros rplidar_a2m7.launch
```

The command for RPLIDAR A2M8 is :

```bash
roslaunch rplidar_ros rplidar_a2m8.launch
```

The command for RPLIDAR A2M12 is :

```bash
roslaunch rplidar_ros rplidar_a2m12.launch
```

The command for RPLIDAR A3 is :

```bash
roslaunch rplidar_ros rplidar_a3.launch
```

The command for RPLIDAR S1 is :

```bash
roslaunch rplidar_ros rplidar_s1.launch
```

The command for RPLIDAR S2 is :

```bash
roslaunch rplidar_ros rplidar_s2.launch
```

The command for RPLIDAR S3 is :

```bash
roslaunch rplidar_ros rplidar_s3.launch
```

The command for RPLIDAR S2E is :

```bash
roslaunch rplidar_ros rplidar_s2e.launch
```

The command for RPLIDAR T1 is :

```bash
roslaunch rplidar_ros rplidar_t1.launch
```

The command for RPLIDAR C1 is :

```bash
roslaunch rplidar_ros rplidar_c1.launch
```

and in another terminal, run the following command

```bash
rosrun rplidar_ros rplidarNodeClient
```

You should see rplidar's scan result in the console.

Notice: different lidar use different serial_baudrate.

## RPLidar frame

RPLidar frame must be broadcasted according to picture shown in rplidar-frame.png
