FROM ros:noetic-ros-base

RUN apt-get update && \
    apt-get install -y --no-install-recommends g++ \
    make \
    git \
    ros-noetic-geometry \
    ros-noetic-laser-filters \
    ros-noetic-ira-laser-tools \
    ros-noetic-pcl-ros && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists

WORKDIR /home/slamtec_lidar_ws/src

COPY . /home/slamtec_lidar_ws/src/

WORKDIR /home/slamtec_lidar_ws/

RUN /ros_entrypoint.sh catkin_make && sed -i '$isource "/home/slamtec_lidar_ws/devel/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD roslaunch rplidar_ros combined_lidar.launch --wait
