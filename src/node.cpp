/*
 *  RPLIDAR ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
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

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "sl_lidar.h" 

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace sl;

ILidarDriver * drv = NULL;

void publish_scan(ros::Publisher *pub,
                  sl_lidar_response_measurement_node_hq_t *nodes,
                  size_t node_count, ros::Time start,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  float max_distance,
                  std::string frame_id)
{
    static int scan_count = 0;
    sensor_msgs::LaserScan scan_msg;

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    scan_count++;
    
    bool reversed = (angle_max > angle_min);
    if ( reversed ) {
      scan_msg.angle_min =  M_PI - angle_max;
      scan_msg.angle_max =  M_PI - angle_min;
    } else {
      scan_msg.angle_min =  M_PI - angle_min;
      scan_msg.angle_max =  M_PI - angle_max;
    }
    scan_msg.angle_increment =
        (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count-1);

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(node_count-1);
    scan_msg.range_min = 0.15;
    scan_msg.range_max = max_distance;//8.0;

    scan_msg.intensities.resize(node_count);
    scan_msg.ranges.resize(node_count);
    bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
    if (!reverse_data) {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
            if (read_value == 0.0)
                scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[i] = read_value;
            scan_msg.intensities[i] = (float) (nodes[i].quality >> 2);
        }
    } else {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000;
            if (read_value == 0.0)
                scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[node_count-1-i] = read_value;
            scan_msg.intensities[node_count-1-i] = (float) (nodes[i].quality >> 2);
        }
    }

    pub->publish(scan_msg);
}

bool getRPLIDARDeviceInfo(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);
    if (SL_IS_FAIL(op_result)) {
        if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
            ROS_ERROR("Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
        } else {
            ROS_ERROR("Error, unexpected error, code: %x",op_result);
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    char sn_str[35] = {0}; 
    for (int pos = 0; pos < 16 ;++pos) {
        sprintf(sn_str + (pos * 2),"%02X", devinfo.serialnum[pos]);
    }
    ROS_INFO("RPLIDAR S/N: %s",sn_str);
    ROS_INFO("Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    ROS_INFO("Hardware Rev: %d",(int)devinfo.hardware_version);
    return true;
}

bool checkRPLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { 
        //ROS_INFO("RPLidar health status : %d", healthinfo.status);
        switch (healthinfo.status) {
			case SL_LIDAR_STATUS_OK:
                ROS_INFO("RPLidar health status : OK.");
				return true;
			case SL_LIDAR_STATUS_WARNING:
                ROS_INFO("RPLidar health status : Warning.");
				return true;
			case SL_LIDAR_STATUS_ERROR:
                ROS_ERROR("Error, rplidar internal error detected. Please reboot the device to retry.");
				return false;
        }
    } else {
        ROS_ERROR("Error, cannot retrieve rplidar health code: %x", op_result);
        return false;
    }
}

bool stop_motor(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res)
{
  if(!drv)
       return false;

  ROS_DEBUG("Stop motor");
  drv->setMotorSpeed(0);
  return true;
}

bool start_motor(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res)
{
  if(!drv)
       return false;
  if(drv->isConnected())
  {
      ROS_DEBUG("Start motor");
      sl_result ans=drv->setMotorSpeed();
  
      ans=drv->startScan(0,1);
   }
   else ROS_INFO("lost connection");
  return true;
}

static float getAngle(const sl_lidar_response_measurement_node_hq_t& node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "rplidar_node");
    
    std::string channel_type;
    std::string tcp_ip;
    int tcp_port = 20108;
    std::string udp_ip;
    int udp_port = 8089;
    std::string serial_port;    
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;
    bool angle_compensate = true;    
    float angle_compensate_multiple = 1.0;//min 360 ponits at per 1 degree
    int points_per_circle = 360;//min 360 ponits at per circle 
    std::string scan_mode;
    float max_distance;
    float scan_frequency;
    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("channel_type", channel_type, "serial");
    nh_private.param<std::string>("tcp_ip", tcp_ip, "192.168.0.7"); 
    nh_private.param<int>("tcp_port", tcp_port, 20108);
    nh_private.param<std::string>("udp_ip", udp_ip, "192.168.11.2"); 
    nh_private.param<int>("udp_port", udp_port, 8089);
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); 
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200/*256000*/);//ros run for A1 A2, change to 256000 if A3
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("inverted", inverted, false);
    nh_private.param<bool>("angle_compensate", angle_compensate, false);
    nh_private.param<std::string>("scan_mode", scan_mode, std::string());
    if(channel_type == "udp"){
        nh_private.param<float>("scan_frequency", scan_frequency, 20.0);
    }
    else{
        nh_private.param<float>("scan_frequency", scan_frequency, 10.0);
    }

    int ver_major = SL_LIDAR_SDK_VERSION_MAJOR;
    int ver_minor = SL_LIDAR_SDK_VERSION_MINOR;
    int ver_patch = SL_LIDAR_SDK_VERSION_PATCH;    
    ROS_INFO("RPLIDAR running on ROS package rplidar_ros, SDK Version:%d.%d.%d",ver_major,ver_minor,ver_patch);

    sl_result  op_result;

    // create the driver instance
    drv = *createLidarDriver();
    IChannel* _channel;
    if(channel_type == "tcp"){
        _channel = *createTcpChannel(tcp_ip, tcp_port);
    }
    else if(channel_type == "udp"){
        _channel = *createUdpChannel(udp_ip, udp_port);
    }
    else{
        _channel = *createSerialPortChannel(serial_port, serial_baudrate);
    }
    if (SL_IS_FAIL((drv)->connect(_channel))) {
		if(channel_type == "tcp"){
            ROS_ERROR("Error, cannot connect to the ip addr  %s with the tcp port %s.",tcp_ip.c_str(),std::to_string(tcp_port).c_str());
        }
        else if(channel_type == "udp"){
            ROS_ERROR("Error, cannot connect to the ip addr  %s with the udp port %s.",udp_ip.c_str(),std::to_string(udp_port).c_str());
        }
        else{
            ROS_ERROR("Error, cannot bind to the specified serial port %s.",serial_port.c_str());            
        }
        delete drv;
        return -1;
    }
    // get rplidar device info
    if(!getRPLIDARDeviceInfo(drv)){
       delete drv;
       return -1;
    }
    if (!checkRPLIDARHealth(drv)) {
        delete drv;
        return -1;
    }
    
    //two service for start/stop lidar rotate
    ros::ServiceServer stop_motor_service = nh.advertiseService("stop_motor", stop_motor);
    ros::ServiceServer start_motor_service = nh.advertiseService("start_motor", start_motor);

    //start lidar rotate
    drv->setMotorSpeed();

    LidarScanMode current_scan_mode;
    if (scan_mode.empty()) {
        op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
    } else {
        std::vector<LidarScanMode> allSupportedScanModes;
        op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

        if (SL_IS_OK(op_result)) {
            sl_u16 selectedScanMode = sl_u16(-1);
            for (std::vector<LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                if (iter->scan_mode == scan_mode) {
                    selectedScanMode = iter->id;
                    break;
                }
            }

            if (selectedScanMode == sl_u16(-1)) {
                ROS_ERROR("scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
                for (std::vector<LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    ROS_ERROR("\t%s: max_distance: %.1f m, Point number: %.1fK",  iter->scan_mode,
                            iter->max_distance, (1000/iter->us_per_sample));
                }
                op_result = SL_RESULT_OPERATION_FAIL;
            } else {
                op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
            }
        }
    }

    if(SL_IS_OK(op_result))
    {
        //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us        
        points_per_circle = (int)(1000*1000/current_scan_mode.us_per_sample/scan_frequency);
        angle_compensate_multiple = points_per_circle/360.0  + 1;
        if(angle_compensate_multiple < 1) 
          angle_compensate_multiple = 1.0;
        max_distance = (float)current_scan_mode.max_distance;
        ROS_INFO("current scan mode: %s, sample rate: %d Khz, max_distance: %.1f m, scan frequency:%.1f Hz, ", current_scan_mode.scan_mode,(int)(1000/current_scan_mode.us_per_sample+0.5),max_distance, scan_frequency); 
    }
    else
    {
        ROS_ERROR("Can not start scan: %08x!", op_result);
    }

    ros::Time start_scan_time;
    ros::Time end_scan_time;
    double scan_duration;
    while (ros::ok()) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        start_scan_time = ros::Time::now();
        op_result = drv->grabScanDataHq(nodes, count);
        end_scan_time = ros::Time::now();
        scan_duration = (end_scan_time - start_scan_time).toSec();

        if (op_result == SL_RESULT_OK) {
            op_result = drv->ascendScanData(nodes, count);
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(360.0f);
            if (op_result == SL_RESULT_OK) {
                if (angle_compensate) {
                                      const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
                    int angle_compensate_offset = 0;
                    sl_lidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
                    memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(sl_lidar_response_measurement_node_hq_t));

                    int i = 0, j = 0;
                    for( ; i < count; i++ ) {
                        if (nodes[i].dist_mm_q2 != 0) {
                            float angle = getAngle(nodes[i]);
                            int angle_value = (int)(angle * angle_compensate_multiple);
                            if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                            for (j = 0; j < angle_compensate_multiple; j++) {

                                int angle_compensate_nodes_index = angle_value-angle_compensate_offset+j;
                                if(angle_compensate_nodes_index >= angle_compensate_nodes_count)
                                    angle_compensate_nodes_index = angle_compensate_nodes_count-1;
                                angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                            }
                        }
                    }
  
                    publish_scan(&scan_pub, angle_compensate_nodes, angle_compensate_nodes_count,
                             start_scan_time, scan_duration, inverted,
                             angle_min, angle_max, max_distance,
                             frame_id);
                } else {
                    int start_node = 0, end_node = 0;
                    int i = 0;
                    // find the first valid node and last valid node
                    while (nodes[i++].dist_mm_q2 == 0);
                    start_node = i-1;
                    i = count -1;
                    while (nodes[i--].dist_mm_q2 == 0);
                    end_node = i+1;

                    angle_min = DEG2RAD(getAngle(nodes[start_node]));
                    angle_max = DEG2RAD(getAngle(nodes[end_node]));

                    publish_scan(&scan_pub, &nodes[start_node], end_node-start_node +1,
                             start_scan_time, scan_duration, inverted,
                             angle_min, angle_max, max_distance,
                             frame_id);
               }
            } else if (op_result == SL_RESULT_OPERATION_FAIL) {
                // All the data is invalid, just publish them
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);
                publish_scan(&scan_pub, nodes, count,
                             start_scan_time, scan_duration, inverted,
                             angle_min, angle_max, max_distance,
                             frame_id);
            }
        }

        ros::spinOnce();
    }

    // done!
    drv->setMotorSpeed(0);
    drv->stop();
    delete drv;
    return 0;
}
