#!/usr/bin/env python3
import rospy
from adc_raw.msg import Mcp3208_data
from sensor_msgs.msg import ChannelFloat32
from sensor_msgs.msg import LaserScan
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped

import numpy as np
class DownSampler():
    def __init__(self,name):
        self.name=name
        self.rospy=rospy
        self.rospy.init_node(self.name,anonymous=True)
        self.rospy.loginfo("[%s] Starting Node ",self.name)
        self.initParams()
        self.initVariables()
        self.initSubscribers()
        self.initPublishers()
        self.main()
    
    def initParams(self):
        self.acquisition_rate = self.rospy.get_param("laser/acquisition_rate",10)
        return

    def initVariables(self):
        self.rate = self.rospy.Rate(self.acquisition_rate)
        self.laser_output_message = LaserScan()
        self.output_size = self.rospy.get_param("rplidar_params/output_samples",1195) 
        self.output_scan_array = []
        self.output_intensities_array=[]
        self.samples_spacing = (int(3585/self.output_size))
        # print(self.samples_spacing)
        self.flag = False
        return

    def initSubscribers(self):
        self.lidar_sub = self.rospy.Subscriber("/raw_scan", LaserScan, self.lidarCallback, queue_size=2)
        return

    def initPublishers(self):
        self.pubLaser = self.rospy.Publisher("/scan",LaserScan,queue_size=2)


        # Let's initialize the transform broadcaster and the message to broadcast
        self.lidar_broadcaster = tf2_ros.TransformBroadcaster()
        self.lidar_transform = TransformStamped()
        self.lidar_transform.header.frame_id = "rplidara2_sensor_link"
        self.lidar_transform.child_frame_id = "base_link"
        #self.pub_laserRaw2 = self.rospy.Publisher(self.publish_topic2,ChannelFloat32,queue_size=10)
        return
    
    def lidarCallback(self,msg):
        self.output_scan_array=[]
        self.output_intensities_array=[]
        # print(len(msg.ranges))
        # self.output_scan_array[0]=msg.ranges[0]
        for i in range(0,len(msg.ranges),3):
            self.output_scan_array.append(msg.ranges[i])
            self.output_intensities_array.append(msg.intensities[i])

        self.output_scan_array=np.array(self.output_scan_array)
        # print(self.output_scan_array.size)

        self.laser_output_message.header=msg.header
        self.laser_output_message.header.frame_id="rplidara2_sensor_link"
        self.laser_output_message.header.stamp = self.rospy.Time.now()
        self.laser_output_message.ranges=self.output_scan_array
        self.laser_output_message.intensities=self.output_intensities_array
        self.laser_output_message.angle_min=msg.angle_min
        self.laser_output_message.angle_max=msg.angle_max
        self.laser_output_message.range_min=msg.range_min
        self.laser_output_message.range_min=msg.range_max
        self.laser_output_message.angle_increment=3*msg.angle_increment
        self.laser_output_message.scan_time=3*msg.scan_time
        self.laser_output_message.time_increment=3*msg.time_increment




          # self.odom_transform.header.stamp = self.rospy.Time.now()
        # self.odom_transform.transform.translation.x = self.pos_x
        # self.odom_transform.transform.translation.y = self.pos_y
        # self.odom_transform.transform.translation.z = 0.0
        # self.odom_transform.transform.rotation.x = odom_quat[0]
        # self.odom_transform.transform.rotation.y = odom_quat[1]
        # self.odom_transform.transform.rotation.z = odom_quat[2]
        # self.odom_transform.transform.rotation.w = odom_quat[3]

        # self.lidar_broadcaster.sendTransform(self.lidar_transform)
        self.pubLaser.publish(self.laser_output_message)



    def main(self):
        while not self.rospy.is_shutdown():
            self.rospy.spin()
if __name__ == '__main__':
	try:
		DownSamplerNode = DownSampler('DownSampler_NODE')
	except rospy.ROSInterruptException:
		pass
