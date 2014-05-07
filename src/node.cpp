/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */



#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace rp::standalone::rplidar;

void publish_scan(ros::Publisher *pub, 
                  rplidar_response_measurement_node_t *nodes, 
                  size_t node_count, ros::Time start,
                  double scan_time, bool inverted, 
                  float angle_min, float angle_max, 
                  size_t zero_pos, std::string frame_id)
{
    static int scan_count = 0;
    sensor_msgs::LaserScan scan_msg;

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    scan_count++;

    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;
    scan_msg.angle_increment = 
        (scan_msg.angle_max - scan_msg.angle_min) / (double)node_count;

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)node_count;

    scan_msg.range_min = 0.15;
    scan_msg.range_max = 6.;

    scan_msg.ranges.resize(node_count);
    if (!inverted) { // assumes scan window at the top
        // The full round of scan data may not always start from zero degree.
        // Needs re-order the range array from zero position
        for (size_t i = zero_pos; i < node_count; i++) {
            scan_msg.ranges[i-zero_pos] 
                = (float)nodes[i].distance_q2/4.0f/1000;
        }
        for (size_t i = 0; i < zero_pos; i++) {
            scan_msg.ranges[i+node_count-zero_pos] 
                = (float)nodes[i].distance_q2/4.0f/1000;
        }
    } else {
        for (size_t i = zero_pos; i < node_count; i++) {
            scan_msg.ranges[node_count-1-i+zero_pos] 
                = (float)nodes[i].distance_q2/4.0f/1000;
        }
        for (size_t i = 0; i < zero_pos; i++) {
            scan_msg.ranges[zero_pos-1-i] 
                = (float)nodes[i].distance_q2/4.0f/1000;
        }
    }

    scan_msg.intensities.resize(node_count);
    for (size_t i = 0; i < node_count; i++) {
        scan_msg.intensities[i] = (float)0;
    }

    pub->publish(scan_msg);
}

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { 
        printf("RPLidar health status : %d\n", healthinfo.status);
        
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected."
                            "Please reboot the device to retry.\n");
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve rplidar health code: %x\n", 
                        op_result);
        return false;
    }
}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "rplidar_node");

    std::string serial_port;
    int serial_baudrate;
    std::string frame_id;
    bool inverted;

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); 
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200); 
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("inverted", inverted, "false");

    u_result     op_result;

    // create the driver instance
    RPlidarDriver * drv = 
        RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    
    if (!drv) {
        fprintf(stderr, "Create Driver fail, exit\n");
        return -2;
    }

    // make connection...
    if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , serial_port.c_str());
        RPlidarDriver::DisposeDriver(drv);
        return -1;
    }

    // check health...
    if (!checkRPLIDARHealth(drv)) {
        RPlidarDriver::DisposeDriver(drv);
        return -1;
    }

    // start scan...
    drv->startScan();

    ros::Time start_scan_time;
    ros::Time end_scan_time;
    double scan_duration;
    while (ros::ok()) {

        rplidar_response_measurement_node_t nodes[360*2];
        size_t   count = _countof(nodes);

        start_scan_time = ros::Time::now();
        op_result = drv->grabScanData(nodes, count);
        end_scan_time = ros::Time::now();
        scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;

        // find zero_position in the full scan
        size_t zero_pos = 0;
        float pre_degree = (nodes[0].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
        if (IS_OK(op_result)) {
            for (size_t pos = 0; pos < count ; ++pos) {
                float degree = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
                if (zero_pos == 0 && (pre_degree - degree > 180)) {
                    zero_pos = pos;
                }
                pre_degree = degree;
            }
        }

        float angle_min = DEG2RAD(0.0);
        float angle_max = DEG2RAD(360.0);
        if(zero_pos) {
            angle_min = DEG2RAD((float)(nodes[zero_pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
            angle_max = DEG2RAD((float)(nodes[zero_pos-1].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
        } else {
            angle_min = DEG2RAD((float)(nodes[zero_pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
            angle_max = DEG2RAD((float)(nodes[count-1].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
        }


        publish_scan(&scan_pub, nodes, count, 
                     start_scan_time, scan_duration, inverted,  
                     angle_min, angle_max, 
                     zero_pos, frame_id);

        ros::spinOnce();
    }

    // done!
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}
