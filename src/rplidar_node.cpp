/*
 *  RPLIDAR ROS2 NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
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
#include "sl_lidar.h"
#include "math.h"

#include <signal.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

#define ROS2VERSION "1.0.1"

enum {
    LIDAR_A_SERIES_MINUM_MAJOR_ID   = 0,
    LIDAR_S_SERIES_MINUM_MAJOR_ID   = 5,
    LIDAR_T_SERIES_MINUM_MAJOR_ID   = 8,
};

using namespace sl;

bool need_exit = false;

class RPlidarNode : public rclcpp::Node
{
  public:
    RPlidarNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("rplidar_node", options)
    {

      
    }

  private:    
    void init_param()
    {
        this->declare_parameter<std::string>("channel_type","serial");
        this->declare_parameter<std::string>("tcp_ip", "192.168.0.7");
        this->declare_parameter<int>("tcp_port", 20108);
        this->declare_parameter<std::string>("udp_ip","192.168.11.2");
        this->declare_parameter<int>("udp_port",8089);
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("serial_baudrate",1000000);
        this->declare_parameter<std::string>("frame_id","laser_frame");
        this->declare_parameter<bool>("inverted", false);
        this->declare_parameter<bool>("angle_compensate", false);
        this->declare_parameter<bool>("flip_x_axis", false);
        this->declare_parameter<bool>("auto_standby", false);
        this->declare_parameter<std::string>("topic_name",std::string("scan"));
        this->declare_parameter<std::string>("scan_mode",std::string());
        this->declare_parameter<float>("scan_frequency",10);
        
        this->get_parameter_or<std::string>("channel_type", channel_type, "serial");
        this->get_parameter_or<std::string>("tcp_ip", tcp_ip, "192.168.0.7"); 
        this->get_parameter_or<int>("tcp_port", tcp_port, 20108);
        this->get_parameter_or<std::string>("udp_ip", udp_ip, "192.168.11.2"); 
        this->get_parameter_or<int>("udp_port", udp_port, 8089);
        this->get_parameter_or<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); 
        this->get_parameter_or<int>("serial_baudrate", serial_baudrate, 1000000/*256000*/);//ros run for A1 A2, change to 256000 if A3
        this->get_parameter_or<std::string>("frame_id", frame_id, "laser_frame");
        this->get_parameter_or<bool>("inverted", inverted, false);
        this->get_parameter_or<bool>("angle_compensate", angle_compensate, false);
        this->get_parameter_or<bool>("flip_x_axis", flip_x_axis, false);
        this->get_parameter_or<bool>("auto_standby", auto_standby, false);
        this->get_parameter_or<std::string>("topic_name", topic_name, "scan");
        this->get_parameter_or<std::string>("scan_mode", scan_mode, std::string());
        if(channel_type == "udp")
            this->get_parameter_or<float>("scan_frequency", scan_frequency, 20.0);
        else
            this->get_parameter_or<float>("scan_frequency", scan_frequency, 10.0);
    }

    bool getRPLIDARDeviceInfo(ILidarDriver * drv)
    {
        sl_result     op_result;
        sl_lidar_response_device_info_t devinfo;

        op_result = drv->getDeviceInfo(devinfo);
        if (SL_IS_FAIL(op_result)) {
            if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
                RCLCPP_ERROR(this->get_logger(),"Error, operation time out. SL_RESULT_OPERATION_TIMEOUT! ");
            } else {
                RCLCPP_ERROR(this->get_logger(),"Error, unexpected error, code: %x",op_result);
            }
            return false;
        }

        // print out the device serial number, firmware and hardware version number..
        char sn_str[37] = {'\0'}; 
        for (int pos = 0; pos < 16 ;++pos) {
            sprintf(sn_str + (pos * 2),"%02X", devinfo.serialnum[pos]);
        }
        RCLCPP_INFO(this->get_logger(),"RPLidar S/N: %s",sn_str);
        RCLCPP_INFO(this->get_logger(),"Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
        RCLCPP_INFO(this->get_logger(),"Hardware Rev: %d",(int)devinfo.hardware_version);
        return true;
    }

    bool checkRPLIDARHealth(ILidarDriver * drv)
    {
        sl_result     op_result;
        sl_lidar_response_device_health_t healthinfo;
        op_result = drv->getHealth(healthinfo);
        if (SL_IS_OK(op_result)) { 
            RCLCPP_INFO(this->get_logger(),"RPLidar health status : %d", healthinfo.status);
            switch (healthinfo.status) {
                case SL_LIDAR_STATUS_OK:
                    RCLCPP_INFO(this->get_logger(),"RPLidar health status : OK.");
                    return true;
                case SL_LIDAR_STATUS_WARNING:
                    RCLCPP_INFO(this->get_logger(),"RPLidar health status : Warning.");
                    return true;
                case SL_LIDAR_STATUS_ERROR:
                    RCLCPP_ERROR(this->get_logger(),"Error, RPLidar internal error detected. Please reboot the device to retry.");
                    return false;
                default:
                    RCLCPP_ERROR(this->get_logger(),"Error, Unknown internal error detected. Please reboot the device to retry.");
                    return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(),"Error, cannot retrieve RPLidar health code: %x", op_result);
            return false;
        }
    }

    bool stop_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;

        if (auto_standby) {
            RCLCPP_INFO(
                this->get_logger(),
                "Ingnoring stop_motor request because rplidar_node is in 'auto standby' mode");
            return false;
        }

        RCLCPP_DEBUG(this->get_logger(), "Call to '%s'", __FUNCTION__);
        
        //RCLCPP_DEBUG(this->get_logger(),"Stop motor");
        this->stop();
        //drv->setMotorSpeed(0);
        return true;
    }

    bool start_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;

        if (auto_standby) {
            RCLCPP_INFO(
                this->get_logger(),
                "Ingnoring start_motor request because rplidar_node is in 'auto standby' mode");
            return false;
        }
        RCLCPP_DEBUG(this->get_logger(), "Call to '%s'", __FUNCTION__);
        return this->start();

#if 0
        if(!drv)
           return false;
        if(drv->isConnected())
        {
            RCLCPP_DEBUG(this->get_logger(),"Start motor");
            sl_result ans=drv->setMotorSpeed();
            if (SL_IS_FAIL(ans)) {
                RCLCPP_WARN(this->get_logger(), "Failed to start motor: %08x", ans);
                return false;
            }
        
            ans=drv->startScan(0,1);
            if (SL_IS_FAIL(ans)) {
                RCLCPP_WARN(this->get_logger(), "Failed to start scan: %08x", ans);
            }
        } else {
            RCLCPP_INFO(this->get_logger(),"lost connection");
            return false;
        }

        return true;
#endif

    }

    static float getAngle(const sl_lidar_response_measurement_node_hq_t& node)
    {
        return node.angle_z_q14 * 90.f / 16384.f;
    }

    void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub,
                  sl_lidar_response_measurement_node_hq_t *nodes,
                  size_t node_count, rclcpp::Time start,
                  double scan_time, bool inverted, bool flip_X_axis,
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
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count-1);

        scan_msg->scan_time = scan_time;
        scan_msg->time_increment = scan_time / (double)(node_count-1);
        scan_msg->range_min = 0.15;
        scan_msg->range_max = max_distance;//8.0;

        scan_msg->intensities.resize(node_count);
        scan_msg->ranges.resize(node_count);
        bool reverse_data = (!inverted && reversed) || (inverted && !reversed);

        size_t scan_midpoint = node_count / 2;
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
            size_t apply_index = i;
            if (reverse_data) {
                apply_index = node_count - 1 - i;
            }
            if (flip_X_axis) {
                if (apply_index >= scan_midpoint)
                    apply_index = apply_index - scan_midpoint;
                else
                    apply_index = apply_index + scan_midpoint;
            }

            if (read_value == 0.0)
                scan_msg->ranges[apply_index] = std::numeric_limits<float>::infinity();
            else
                scan_msg->ranges[apply_index] = read_value;
            scan_msg->intensities[apply_index] = (float)(nodes[apply_index].quality >> 2);
        }

        pub->publish(*scan_msg);
    }

    bool set_scan_mode() {
        sl_result     op_result;
        LidarScanMode current_scan_mode;
        if (scan_mode.empty()) {
            op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
        }
        else {
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
                    RCLCPP_ERROR(this->get_logger(), "scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
                    for (std::vector<LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                        RCLCPP_ERROR(this->get_logger(), "\t%s: max_distance: %.1f m, Point number: %.1fK", iter->scan_mode,
                            iter->max_distance, (1000 / iter->us_per_sample));
                    }
                    op_result = SL_RESULT_OPERATION_FAIL;
                }
                else {
                    op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
                }
            }
        }

        if (SL_IS_OK(op_result))
        {
            //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
            int points_per_circle = (int)(1000 * 1000 / current_scan_mode.us_per_sample / scan_frequency);
            angle_compensate_multiple = points_per_circle / 360.0 + 1;
            if (angle_compensate_multiple < 1)
                angle_compensate_multiple = 1.0;
            max_distance = (float)current_scan_mode.max_distance;
            RCLCPP_INFO(this->get_logger(), "current scan mode: %s, sample rate: %d Khz, max_distance: %.1f m, scan frequency:%.1f Hz, ",
                current_scan_mode.scan_mode, (int)(1000 / current_scan_mode.us_per_sample + 0.5), max_distance, scan_frequency);
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Can not start scan: %08x!", op_result);
            return false;
        }
    }
    bool start()
    {
        if (nullptr == drv) {
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Start");
        drv->setMotorSpeed();
        if (!set_scan_mode()) {
            this->stop();
            RCLCPP_ERROR(this->get_logger(), "Failed to set scan mode");
            return false;
        }
        is_scanning = true;
        return true;
    }

    void stop()
    {
        if (nullptr == drv) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Stop");
        drv->stop();
        drv->setMotorSpeed(0);
        is_scanning = false;
    }

public:    
    int work_loop()
    {        
        init_param();
        int ver_major = SL_LIDAR_SDK_VERSION_MAJOR;
        int ver_minor = SL_LIDAR_SDK_VERSION_MINOR;
        int ver_patch = SL_LIDAR_SDK_VERSION_PATCH;
        RCLCPP_INFO(this->get_logger(),"RPLidar running on ROS2 package rplidar_ros. RPLIDAR SDK Version:%d.%d.%d",ver_major,ver_minor,ver_patch);
    
        sl_result     op_result;
        // create the driver instance
        drv = *createLidarDriver();
        if (nullptr == drv) {
            /* don't start spinning without a driver object */
            RCLCPP_ERROR(this->get_logger(), "Failed to construct driver");
            return -1;
        }
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
                RCLCPP_ERROR(this->get_logger(),"Error, cannot connect to the ip addr  %s with the tcp port %s.",tcp_ip.c_str(),std::to_string(tcp_port).c_str());
            }
            else if(channel_type == "udp"){
                RCLCPP_ERROR(this->get_logger(),"Error, cannot connect to the ip addr  %s with the udp port %s.",udp_ip.c_str(),std::to_string(udp_port).c_str());
            }
            else{
                RCLCPP_ERROR(this->get_logger(),"Error, cannot bind to the specified serial port %s.",serial_port.c_str());            
            }
            delete drv; drv = nullptr;
            return -1;
        }
        
        // get rplidar device info
        if (!getRPLIDARDeviceInfo(drv)) {
            delete drv; drv = nullptr;
            return -1;
        }

        // check health...
        if (!checkRPLIDARHealth(drv)) {
            delete drv; drv = nullptr;
            return -1;
        }

        sl_lidar_response_device_info_t devinfo;
        op_result = drv->getDeviceInfo(devinfo);
        bool scan_frequency_tunning_after_scan = false;

        if( (devinfo.model>>4) > LIDAR_S_SERIES_MINUM_MAJOR_ID){
            scan_frequency_tunning_after_scan = true;
        }

        if(!scan_frequency_tunning_after_scan){ //for RPLIDAR A serials
            //start RPLIDAR A serials  rotate by pwm
            drv->setMotorSpeed(600);
        }

        /* start motor and scanning */
        if (!auto_standby && !this->start()) {
            delete drv; drv = nullptr;
            return -1;
        }

        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, rclcpp::QoS(rclcpp::KeepLast(10)));

        stop_motor_service = this->create_service<std_srvs::srv::Empty>("stop_motor",  
                                std::bind(&RPlidarNode::stop_motor,this,std::placeholders::_1,std::placeholders::_2));
        start_motor_service = this->create_service<std_srvs::srv::Empty>("start_motor", 
                                std::bind(&RPlidarNode::start_motor,this,std::placeholders::_1,std::placeholders::_2));

        //drv->setMotorSpeed();

        rclcpp::Time start_scan_time;
        rclcpp::Time end_scan_time;
        double scan_duration;
        while (rclcpp::ok() && !need_exit) {
            sl_lidar_response_measurement_node_hq_t nodes[8192];
            size_t   count = _countof(nodes);

            if (auto_standby) {
                if (scan_pub->get_subscription_count() > 0 && !is_scanning) {
                    this->start();
                }
                else if (scan_pub->get_subscription_count() == 0) {
                    if (is_scanning) {
                        this->stop();
                    }
                }
            }

            start_scan_time = this->now();
            op_result = drv->grabScanDataHq(nodes, count);
            end_scan_time = this->now();
            scan_duration = (end_scan_time - start_scan_time).seconds();

            if (op_result == SL_RESULT_OK) {
                if(scan_frequency_tunning_after_scan) { //Set scan frequency(For Slamtec Tof lidar)
                    RCLCPP_ERROR(this->get_logger(), "set lidar scan frequency to %.1f Hz(%.1f Rpm) ",scan_frequency,scan_frequency*60);
                    drv->setMotorSpeed(scan_frequency*60); //rpm 
                    scan_frequency_tunning_after_scan = false;
                    continue;
                }
                op_result = drv->ascendScanData(nodes, count);
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);
                if (op_result == SL_RESULT_OK) {
                    if (angle_compensate) {
                        //const int angle_compensate_multiple = 1;
                        const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
                        int angle_compensate_offset = 0;
                        auto angle_compensate_nodes = new sl_lidar_response_measurement_node_hq_t[angle_compensate_nodes_count];
                        memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(sl_lidar_response_measurement_node_hq_t));

                        size_t i = 0, j = 0;
                        for( ; i < count; i++ ) {
                            if (nodes[i].dist_mm_q2 != 0) {
                                float angle = getAngle(nodes[i]);
                                int angle_value = (int)(angle * angle_compensate_multiple);
                                if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                                for (j = 0; j < angle_compensate_multiple; j++) {
                                    int angle_compensate_nodes_index = angle_value-angle_compensate_offset + j;
                                    if(angle_compensate_nodes_index >= angle_compensate_nodes_count)
                                        angle_compensate_nodes_index = angle_compensate_nodes_count - 1;
                                    angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                                }
                            }
                        }
    
                        publish_scan(scan_pub, angle_compensate_nodes, angle_compensate_nodes_count,
                                start_scan_time, scan_duration, inverted, flip_x_axis,
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
                                start_scan_time, scan_duration, inverted, flip_x_axis, 
                                angle_min, angle_max, max_distance,
                                frame_id);
                    }
                } else if (op_result == SL_RESULT_OPERATION_FAIL) {
                    // All the data is invalid, just publish them
                    float angle_min = DEG2RAD(0.0f);
                    float angle_max = DEG2RAD(359.0f);
                    publish_scan(scan_pub, nodes, count,
                                start_scan_time, scan_duration, inverted, flip_x_axis,
                                angle_min, angle_max, max_distance,
                                frame_id);
                }
            }

            rclcpp::spin_some(shared_from_this());
        }

        // done!
        drv->setMotorSpeed(0);
        drv->stop();
        RCLCPP_INFO(this->get_logger(),"Stop motor");
        if (drv) { delete drv;  drv = nullptr; }
        return 0;
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service;

    std::string channel_type;
    std::string tcp_ip;
    std::string udp_ip;
    std::string serial_port;
    std::string topic_name;
    int tcp_port = 20108;
    int udp_port = 8089;
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;
    bool angle_compensate = true;
    bool flip_x_axis = false;
    bool auto_standby = false;
    float max_distance = 8.0;
    size_t angle_compensate_multiple = 1;//it stand of angle compensate at per 1 degree
    std::string scan_mode;
    float scan_frequency;
    /* State */
    bool is_scanning = false;

    ILidarDriver *drv = nullptr;
};

void ExitHandler(int sig)
{
    (void)sig;
    need_exit = true;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  
  auto rplidar_node = std::make_shared<RPlidarNode>(rclcpp::NodeOptions());
  signal(SIGINT,ExitHandler);
  int ret = rplidar_node->work_loop();
  rclcpp::shutdown();
  return ret;
}

