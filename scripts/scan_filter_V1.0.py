#!/usr/bin/env python3

#Esse script verifica se a proporção de amostras com valor inf em uma mensagem do lidar é maior que o aceitável, caso for maior, a mensagem anterior será republicada.
#Como utilizamos apenas 66% da visão do lidar, o valor mínimo de proporção é 34%, onde qualquer valor inf a mais além do pré-ajustado na mensagem fará com que a mesma seja ignorada.
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

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
        self.proportion = 0.50
        self.deleted_counter = 0
        self.general_counter = 0
        self.erro_percentil = 0
        
    def initSubscribers(self):
        self.indices_sub = rospy.Subscriber("/scan_unfiltered", LaserScan, self.lidar_callback)

    def initPublishers(self):
        self.filtered_scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=50)

    def lidar_callback(self, msg):
        scan_data = np.array(msg.ranges)


        inf_count = np.sum(np.isinf(scan_data))
        total_count = len(scan_data)


        if ((inf_count/total_count) < self.proportion):
            self.last_scan = msg
            self.filtered_scan_pub.publish(self.last_scan)
            self.general_counter = self.general_counter + 1

        else:
            if self.last_scan is not None:
                self.last_scan.header.stamp = self.rospy.Time.now()
                self.filtered_scan_pub.publish(self.last_scan)
                self.deleted_counter = self.deleted_counter + 1
                self.general_counter = self.general_counter + 1
                print("numero de mensagens excluidas: ", self.deleted_counter)
                self.erro_percentil = (self.deleted_counter/self.general_counter)*100
                print("percentual de erro: ",self.erro_percentil)

    
  
    def main(self):
        while not self.rospy.is_shutdown():
            

            rospy.spin()

if __name__ == '__main__':
    try:
        scanFilter()
    except rospy.ROSInterruptException:
        pass