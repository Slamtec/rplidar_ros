/*
 *  RPLIDAR ROS2 NODE
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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include "rplidar.h"

#include <signal.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

#define ROS2VERSION "1.0.1"

using namespace rp::standalone::rplidar;

bool need_exit = false;

class RPLidarScanPublisher : public rclcpp::Node
{
  public:
    RPLidarScanPublisher()
    : Node("rplidar_scan_publisher")
    {
      scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::SensorDataQoS()));
      
    }

  private:    
    void init_param()
    {
        this->declare_parameter("channel_type");
        this->declare_parameter("tcp_ip");
        this->declare_parameter("tcp_port");
        this->declare_parameter("serial_port");
        this->declare_parameter("serial_baudrate");
        this->declare_parameter("frame_id");
        this->declare_parameter("inverted");
        this->declare_parameter("angle_compensate");
        this->declare_parameter("scan_mode");

        this->get_parameter_or<std::string>("channel_type", channel_type, "serial");
        this->get_parameter_or<std::string>("tcp_ip", tcp_ip, "192.168.0.7"); 
        this->get_parameter_or<int>("tcp_port", tcp_port, 20108);
        this->get_parameter_or<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); 
        this->get_parameter_or<int>("serial_baudrate", serial_baudrate, 115200/*256000*/);//ros run for A1 A2, change to 256000 if A3
        this->get_parameter_or<std::string>("frame_id", frame_id, "laser_frame");
        this->get_parameter_or<bool>("inverted", inverted, false);
        this->get_parameter_or<bool>("angle_compensate", angle_compensate, false);
        this->get_parameter_or<std::string>("scan_mode", scan_mode, std::string());
    }

    bool getRPLIDARDeviceInfo(RPlidarDriver * drv)
    {
        u_result     op_result;
        rplidar_response_device_info_t devinfo;

        op_result = drv->getDeviceInfo(devinfo);
        if (IS_FAIL(op_result)) {
            if (op_result == RESULT_OPERATION_TIMEOUT) {
                RCLCPP_ERROR(this->get_logger(),"Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
            } else {
                RCLCPP_ERROR(this->get_logger(),"Error, unexpected error, code: %x",op_result);
            }
            return false;
        }

        // print out the device serial number, firmware and hardware version number..
        std::string sn_str;
        for (int pos = 0; pos < 16 ;++pos) {
            char sn[3]={};
            sprintf(sn,"%02X", devinfo.serialnum[pos]);
            sn_str += std::string(sn,sn+2);
        }
        RCLCPP_INFO(this->get_logger(),"RPLIDAR S/N: %s",sn_str.c_str());
        RCLCPP_INFO(this->get_logger(),"Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
        RCLCPP_INFO(this->get_logger(),"Hardware Rev: %d",(int)devinfo.hardware_version);
        return true;
    }

    bool checkRPLIDARHealth(RPlidarDriver * drv)
    {
        u_result     op_result;
        rplidar_response_device_health_t healthinfo;
        op_result = drv->getHealth(healthinfo);
        if (IS_OK(op_result)) { 
            RCLCPP_INFO(this->get_logger(),"RPLidar health status : %d", healthinfo.status);
            if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
                RCLCPP_ERROR(this->get_logger(),"Error, rplidar internal error detected. Please reboot the device to retry.");
                return false;
            } else {
                return true;
            }

        } else {
            RCLCPP_ERROR(this->get_logger(),"Error, cannot retrieve rplidar health code: %x", op_result);
            return false;
        }
    }

    bool stop_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;

        if(!drv)
            return false;

        RCLCPP_DEBUG(this->get_logger(),"Stop motor");
        drv->stopMotor();
        return true;
    }

    bool start_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;

        if(!drv)
           return false;
        if(drv->isConnected())
        {
            RCLCPP_DEBUG(this->get_logger(),"Start motor");
            u_result ans=drv->startMotor();
            if (IS_FAIL(ans)) {
                RCLCPP_WARN(this->get_logger(), "Failed to start motor: %08x", ans);
                return false;
            }
        
            ans=drv->startScan(0,1);
            if (IS_FAIL(ans)) {
                RCLCPP_WARN(this->get_logger(), "Failed to start scan: %08x", ans);
            }
        } else {
            RCLCPP_INFO(this->get_logger(),"lost connection");
            return false;
        }

        return true;
    }

    static float getAngle(const rplidar_response_measurement_node_hq_t& node)
    {
        return node.angle_z_q14 * 90.f / 16384.f;
    }

    void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub,
                  rplidar_response_measurement_node_hq_t *nodes,
                  size_t node_count, rclcpp::Time start,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  float max_distance,
                  std::string frame_id)
    {
        static int scan_count = 0;
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

        scan_msg->header.stamp = start;
        scan_msg->header.frame_id = frame_id;
        scan_count++;

        bool reversed = (angle_max > angle_min);
        if ( reversed ) {
        scan_msg->angle_min =  M_PI - angle_max;
        scan_msg->angle_max =  M_PI - angle_min;
        } else {
        scan_msg->angle_min =  M_PI - angle_min;
        scan_msg->angle_max =  M_PI - angle_max;
        }
        scan_msg->angle_increment =
            (scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count-1);

        scan_msg->scan_time = scan_time;
        scan_msg->time_increment = scan_time / (double)(node_count-1);
        scan_msg->range_min = 0.15;
        scan_msg->range_max = max_distance;//8.0;

        scan_msg->intensities.resize(node_count);
        scan_msg->ranges.resize(node_count);
        bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
        if (!reverse_data) {
            for (size_t i = 0; i < node_count; i++) {
                float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
                if (read_value == 0.0)
                    scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
                else
                    scan_msg->ranges[i] = read_value;
                scan_msg->intensities[i] = (float) (nodes[i].quality >> 2);
            }
        } else {
            for (size_t i = 0; i < node_count; i++) {
                float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000;
                if (read_value == 0.0)
                    scan_msg->ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
                else
                    scan_msg->ranges[node_count-1-i] = read_value;
                scan_msg->intensities[node_count-1-i] = (float) (nodes[i].quality >> 2);
            }
        }

        pub->publish(*scan_msg);
    }
public:    
    int work_loop()
    {
        
        init_param();
        RCLCPP_INFO(this->get_logger(),"RPLIDAR running on ROS2 package rplidar_ros2. ROS2 SDK Version:" ROS2VERSION ", RPLIDAR SDK Version:" RPLIDAR_SDK_VERSION "");

        u_result     op_result;

        // create the driver instance
        if(channel_type == "tcp"){
            drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP);
        }
        else{
            drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
        }

        
        if (!drv) {
            RCLCPP_ERROR(this->get_logger(),"Create Driver fail, exit");
            return -2;
        }

        if(channel_type == "tcp"){
            // make connection...
            if (IS_FAIL(drv->connect(tcp_ip.c_str(), (_u32)tcp_port))) {
                RCLCPP_ERROR(this->get_logger(),"Error, cannot bind to the specified serial port %s.",serial_port.c_str());
                RPlidarDriver::DisposeDriver(drv);
                return -1;
            }

        }
        else{
        // make connection...
            if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
                RCLCPP_ERROR(this->get_logger(),"Error, cannot bind to the specified serial port %s.",serial_port.c_str());
                RPlidarDriver::DisposeDriver(drv);
                return -1;
            }

        }
        
        // get rplidar device info
        if (!getRPLIDARDeviceInfo(drv)) {
            return -1;
        }

        // check health...
        if (!checkRPLIDARHealth(drv)) {
            RPlidarDriver::DisposeDriver(drv);
            return -1;
        }

        stop_motor_service = this->create_service<std_srvs::srv::Empty>("stop_motor",  
                                std::bind(&RPLidarScanPublisher::stop_motor,this,std::placeholders::_1,std::placeholders::_2));
        start_motor_service = this->create_service<std_srvs::srv::Empty>("start_motor", 
                                std::bind(&RPLidarScanPublisher::start_motor,this,std::placeholders::_1,std::placeholders::_2));

        drv->startMotor();

        RplidarScanMode current_scan_mode;
        if (scan_mode.empty()) {
            op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
        } else {
            std::vector<RplidarScanMode> allSupportedScanModes;
            op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

            if (IS_OK(op_result)) {
                _u16 selectedScanMode = _u16(-1);
                for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    if (iter->scan_mode == scan_mode) {
                        selectedScanMode = iter->id;
                        break;
                    }
                }

                if (selectedScanMode == _u16(-1)) {
                    RCLCPP_ERROR(this->get_logger(),"scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
                    for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                        RCLCPP_ERROR(this->get_logger(),"\t%s: max_distance: %.1f m, Point number: %.1fK",  iter->scan_mode,
                                iter->max_distance, (1000/iter->us_per_sample));
                    }
                    op_result = RESULT_OPERATION_FAIL;
                } else {
                    op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
                }
            }
        }

        if(IS_OK(op_result))
        {
            //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
            angle_compensate_multiple = (int)(1000*1000/current_scan_mode.us_per_sample/10.0/360.0);
            if(angle_compensate_multiple < 1) 
            angle_compensate_multiple = 1;
            max_distance = current_scan_mode.max_distance;
            RCLCPP_INFO(this->get_logger(),"current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d",current_scan_mode.scan_mode,
                    current_scan_mode.max_distance, (1000/current_scan_mode.us_per_sample), angle_compensate_multiple);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),"Can not start scan: %08x!", op_result);
        }

        rclcpp::Time start_scan_time;
        rclcpp::Time end_scan_time;
        double scan_duration;
        while (rclcpp::ok() && !need_exit) {
            rplidar_response_measurement_node_hq_t nodes[360*8];
            size_t   count = _countof(nodes);

            start_scan_time = this->now();
            op_result = drv->grabScanDataHq(nodes, count);
            end_scan_time = this->now();
            scan_duration = (end_scan_time - start_scan_time).seconds();

            if (op_result == RESULT_OK) {
                op_result = drv->ascendScanData(nodes, count);
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);
                if (op_result == RESULT_OK) {
                    if (angle_compensate) {
                        //const int angle_compensate_multiple = 1;
                        const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
                        int angle_compensate_offset = 0;
                        auto angle_compensate_nodes = new rplidar_response_measurement_node_hq_t[angle_compensate_nodes_count];
                        memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_hq_t));

                        size_t i = 0, j = 0;
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
    
                        publish_scan(scan_pub, angle_compensate_nodes, angle_compensate_nodes_count,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance,
                                frame_id);

                        if (angle_compensate_nodes) {
                            delete[] angle_compensate_nodes;
                            angle_compensate_nodes = nullptr;
                        }
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

                        publish_scan(scan_pub, &nodes[start_node], end_node-start_node +1,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance,
                                frame_id);
                    }
                } else if (op_result == RESULT_OPERATION_FAIL) {
                    // All the data is invalid, just publish them
                    float angle_min = DEG2RAD(0.0f);
                    float angle_max = DEG2RAD(359.0f);
                    publish_scan(scan_pub, nodes, count,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance,
                                frame_id);
                }
            }

            rclcpp::spin_some(shared_from_this());
        }

        // done!
        drv->stopMotor();
        drv->stop();
        RCLCPP_INFO(this->get_logger(),"Stop motor");
        RPlidarDriver::DisposeDriver(drv);

        return 0;
    }


  private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service;

    std::string channel_type;
    std::string tcp_ip;
    std::string serial_port;
    int tcp_port = 20108;
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;
    bool angle_compensate = true;
    float max_distance = 8.0;
    size_t angle_compensate_multiple = 1;//it stand of angle compensate at per 1 degree
    std::string scan_mode;

    RPlidarDriver * drv;    
};

void ExitHandler(int sig)
{
    (void)sig;
    need_exit = true;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  
  auto rplidar_scan_publisher = std::make_shared<RPLidarScanPublisher>();
  signal(SIGINT,ExitHandler);
  int ret = rplidar_scan_publisher->work_loop();
  rclcpp::shutdown();
  return ret;
}

