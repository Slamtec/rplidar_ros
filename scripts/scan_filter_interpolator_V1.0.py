#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from scipy.interpolate import interp1d

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
        self.regions = [(0,658), (1315,1972)]
    
    def initSubscribers(self):
        self.indices_sub = rospy.Subscriber("/scan_unfiltered", LaserScan, self.lidar_callback)

    def initPublishers(self):
        self.filtered_scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=50)

    def lidar_callback(self, msg):
        scan_data = np.array(msg.ranges)
        print("resultado antes da interpolação: ", msg.ranges[95:125])
        inf_count = np.sum(np.isinf(scan_data))
        total_count = len(scan_data)

        if ((inf_count/total_count) < self.proportion):
            self.last_scan = msg
            
            self.last_scan.ranges = self.lidar_interpolator(scan_data)
            print("resultado depois da interpolação: ", self.last_scan.ranges[95:125])
            self.filtered_scan_pub.publish(self.last_scan)

        else:
            if self.last_scan is not None:
                self.filtered_scan_pub.publish(self.last_scan)

    def lidar_interpolator(self, ranges):
        
        for region in self.regions:
            start_index, end_index = region

            region_ranges = ranges[start_index:end_index]

            valid = np.isfinite(region_ranges)

            invalid = ~valid
            
            if np.sum(valid) == 0:
                continue
     
            interpolate_array = interp1d(np.where(valid)[0], region_ranges[valid], kind = 'linear', bounds_error=False, fill_value='extrapolate')

            region_ranges[invalid] = interpolate_array(np.where(invalid)[0])

            ranges[start_index:end_index] = region_ranges

        return ranges
  
    def main(self):
        while not self.rospy.is_shutdown():
            
            rospy.spin()

if __name__ == '__main__':
    try:
        scanFilter()
    except rospy.ROSInterruptException:
        pass