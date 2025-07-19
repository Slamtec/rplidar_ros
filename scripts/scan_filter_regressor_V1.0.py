#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from math import pi, radians
from sklearn.impute import KNNImputer

class scanFilter:
    def __init__(self):
        self.rospy = rospy
        rospy.init_node("scan_filter_node")
        self.initVariables()
        self.initSubscribers()
        self.initPublishers()
        self.main()
    
    def initVariables(self):
        self.last_scan = LaserScan()
        self.proportion = 0.5
        self.lidar_fov = 240
        self.regions = [(0,0),(0,0)]

    
    def initSubscribers(self):
        self.indices_sub = rospy.Subscriber("/scan_unfiltered", LaserScan, self.lidar_callback)

    def initPublishers(self):
        self.filtered_scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=50)

    def lidar_callback(self, msg):
        scan_data = np.array(msg.ranges)

        inf_count = np.sum(np.isinf(scan_data))
        total_count = len(scan_data)

        min_mask_index = round(total_count * (radians(self.lidar_fov / (4 * pi))))
        max_mask_index = round(total_count * (1 - radians(self.lidar_fov / (4 * pi))))

        self.regions = [(0,min_mask_index), (max_mask_index,total_count)]

        if ((inf_count/total_count) < self.proportion):
            self.last_scan = msg
            # print("Antes da regressão: ", scan_data[90:130])

            self.last_scan.ranges = self.lidar_regressor(scan_data)

            # print("Depois da regressão: ", self.last_scan[90:130])

            self.filtered_scan_pub.publish(self.last_scan)

        else:
            if self.last_scan is not None:
                self.filtered_scan_pub.publish(self.last_scan)

    def lidar_regressor(self, ranges):
        
        for region in self.regions:
            start_index, end_index = region

            region_ranges = ranges[start_index:end_index].reshape(-1,1)

            region_ranges[~np.isfinite(region_ranges)] = np.nan
            
            imputer = KNNImputer(n_neighbors=5)
            interpolated_region = imputer.fit_transform(region_ranges)

            ranges[start_index:end_index] = interpolated_region.ravel()

        return ranges
  
    def main(self):
        while not self.rospy.is_shutdown():
            
            rospy.spin()

if __name__ == '__main__':
    try:
        scanFilter()
    except rospy.ROSInterruptException:
        pass