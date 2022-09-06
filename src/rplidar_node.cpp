/*
 *  Rplidar ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 *  Copyright (c) 2019 Hunter L. Allen
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

#include <rplidar_node.hpp>

namespace rplidar_ros
{

rplidar_node::rplidar_node(const rclcpp::NodeOptions & options)
: rclcpp::Node("rplidar_node", options)
{
  /* set parameters */
  channel_type_ = this->declare_parameter("channel_type", "serial");
  tcp_ip_ = this->declare_parameter("tcp_ip", "192.168.0.7");
  tcp_port_ = this->declare_parameter("tcp_port", 20108);
  serial_port_ = this->declare_parameter("serial_port", "/dev/ttyUSB0");
  serial_baudrate_ = this->declare_parameter("serial_baudrate", 115200);
  frame_id_ = this->declare_parameter("frame_id", std::string("laser_frame"));
  inverted_ = this->declare_parameter("inverted", false);
  angle_compensate_ = this->declare_parameter("angle_compensate", false);
  flip_x_axis_ = this->declare_parameter("flip_x_axis", false);
  scan_mode_ = this->declare_parameter("scan_mode", std::string());
  topic_name_ = this->declare_parameter("topic_name", std::string("scan"));
  auto_standby_ = this->declare_parameter("auto_standby", false);

  RCLCPP_INFO(
    this->get_logger(),
    "RPLIDAR running on ROS 2 package rplidar_ros. SDK Version: '%s'", RPLIDAR_SDK_VERSION);

  /* initialize SDK */
  m_drv = (channel_type_ == "tcp") ?
    RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP) :
    RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);

  if (nullptr == m_drv) {
    /* don't start spinning without a driver object */
    RCLCPP_ERROR(this->get_logger(), "Failed to construct driver");
    return;
  }

  if (channel_type_ == "tcp") {
    // make connection...
    if (IS_FAIL(m_drv->connect(tcp_ip_.c_str(), (_u32)tcp_port_))) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Error, cannot bind to the specified TCP host '%s:%ud'",
        tcp_ip_.c_str(), static_cast<unsigned int>(tcp_port_));
      RPlidarDriver::DisposeDriver(m_drv);
      return;
    }
  } else {
    // make connection...
    if (IS_FAIL(m_drv->connect(serial_port_.c_str(), (_u32)serial_baudrate_))) {
      RCLCPP_ERROR(
        this->get_logger(), "Error, cannot bind to the specified serial port '%s'.",
        serial_port_.c_str());
      RPlidarDriver::DisposeDriver(m_drv);
      return;
    }
  }

  // get rplidar device info
  if (!getRPLIDARDeviceInfo()) {
    /* don't continue */
    RPlidarDriver::DisposeDriver(m_drv);
    return;
  }

  // check health...
  if (!checkRPLIDARHealth()) {
    RPlidarDriver::DisposeDriver(m_drv);
    return;
  }

  /* start motor and scanning */
  if (!auto_standby_) {
    this->start();
  }

  /* done setting up RPLIDAR stuff, now set up ROS 2 stuff */

  /* create the publisher for "/scan" */
  m_publisher = this->create_publisher<LaserScan>(topic_name_, 10);

  /* create stop motor service */
  m_stop_motor_service = this->create_service<std_srvs::srv::Empty>(
    "stop_motor",
    std::bind(&rplidar_node::stop_motor, this, std::placeholders::_1, std::placeholders::_2));

  /* create start motor service */
  m_start_motor_service = this->create_service<std_srvs::srv::Empty>(
    "start_motor",
    std::bind(&rplidar_node::start_motor, this, std::placeholders::_1, std::placeholders::_2));
  /* start timer */
  m_timer = this->create_wall_timer(1ms, std::bind(&rplidar_node::publish_loop, this));
}

rplidar_node::~rplidar_node()
{
  m_drv->stop();
  m_drv->stopMotor();
  RPlidarDriver::DisposeDriver(m_drv);
}

void rplidar_node::publish_scan(
  const double scan_time, ResponseNodeArray nodes, size_t node_count)
{
  static size_t scan_count = 0;
  sensor_msgs::msg::LaserScan scan_msg;

  /* NOTE(allenh1): time was passed in as a parameter before */
  scan_msg.header.stamp = this->now();
  scan_msg.header.frame_id = frame_id_;
  scan_count++;

  bool reversed = (angle_max > angle_min);
  if (reversed) {
    /* NOTE(allenh1): the other case seems impossible? */
    scan_msg.angle_min = M_PI - angle_max;
    scan_msg.angle_max = M_PI - angle_min;
  } else {
    scan_msg.angle_min = M_PI - angle_min;
    scan_msg.angle_max = M_PI - angle_max;
  }
  scan_msg.angle_increment =
    (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count - 1);

  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / (double)(node_count - 1);
  scan_msg.range_min = min_distance;
  scan_msg.range_max = max_distance;

  scan_msg.intensities.resize(node_count);
  scan_msg.ranges.resize(node_count);
  bool reverse_data = (!inverted_ && reversed) || (inverted_ && !reversed);
  size_t scan_midpoint = node_count / 2;
  for (size_t i = 0; i < node_count; ++i) {
    float read_value = (float) nodes[i].dist_mm_q2 / 4.0f / 1000;
    size_t apply_index = i;
    if (reverse_data) {
      apply_index = node_count - 1 - i;
    }
    if (flip_x_axis_) {
      if (apply_index >= scan_midpoint) {
        apply_index = apply_index - scan_midpoint;
      } else {
        apply_index = apply_index + scan_midpoint;
      }
    }
    if (read_value == 0.0) {
      scan_msg.ranges[apply_index] = std::numeric_limits<float>::infinity();
    } else {
      scan_msg.ranges[apply_index] = read_value;
    }
    scan_msg.intensities[apply_index] = (float) (nodes[i].quality >> 2);
  }

  m_publisher->publish(scan_msg);
}


bool rplidar_node::getRPLIDARDeviceInfo() const
{
  u_result op_result;
  rplidar_response_device_info_t devinfo;

  op_result = m_drv->getDeviceInfo(devinfo);
  if (IS_FAIL(op_result)) {
    if (op_result == RESULT_OPERATION_TIMEOUT) {
      RCLCPP_ERROR(this->get_logger(), "Error, operation time out. RESULT_OPERATION_TIMEOUT!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error, unexpected error, code: '%x'", op_result);
    }
    return false;
  }

  // print out the device serial number, firmware and hardware version number..
  std::string serial_no{"RPLIDAR S/N: "};
  for (int pos = 0; pos < 16; ++pos) {
    char buff[3];
    snprintf(buff, sizeof(buff), "%02X", devinfo.serialnum[pos]);
    serial_no += buff;
  }
  RCLCPP_INFO(this->get_logger(), "%s", serial_no.c_str());
  RCLCPP_INFO(
    this->get_logger(), "Firmware Ver: %d.%02d", devinfo.firmware_version >> 8,
    devinfo.firmware_version & 0xFF);
  RCLCPP_INFO(this->get_logger(), "Hardware Rev: %d", static_cast<int>(devinfo.hardware_version));
  return true;
}

bool rplidar_node::checkRPLIDARHealth() const
{
  rplidar_response_device_health_t healthinfo;
  u_result op_result = m_drv->getHealth(healthinfo);

  if (IS_OK(op_result)) {
    RCLCPP_INFO(this->get_logger(), "RPLidar health status : '%d'", healthinfo.status);
    if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Error, rplidar internal error detected. Please reboot the device to retry");
      return false;
    }
    return true;
  }
  RCLCPP_ERROR(this->get_logger(), "Error, cannot retrieve rplidar health code: '%x'", op_result);
  return false;
}

void rplidar_node::stop_motor(const EmptyRequest req, EmptyResponse res)
{
  if (auto_standby_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Ingnoring stop_motor request because rplidar_node is in 'auto standby' mode");
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Call to '%s'", __FUNCTION__);

  this->stop();
}

void rplidar_node::start_motor(const EmptyRequest req, EmptyResponse res)
{
  if (auto_standby_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Ingnoring start_motor request because rplidar_node is in 'auto standby' mode");
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Call to '%s'", __FUNCTION__);

  this->start();
}

bool rplidar_node::set_scan_mode()
{
  u_result op_result;
  RplidarScanMode current_scan_mode;
  if (scan_mode_.empty()) {
    op_result = m_drv->startScan(
      false /* not force scan */, true /* use typical scan mode */, 0,
      &current_scan_mode);
  } else {
    std::vector<RplidarScanMode> allSupportedScanModes;
    op_result = m_drv->getAllSupportedScanModes(allSupportedScanModes);
    if (IS_OK(op_result)) {
      auto iter = std::find_if(
        allSupportedScanModes.begin(), allSupportedScanModes.end(),
        [this](auto s1) {
          return std::string(s1.scan_mode) == scan_mode_;
        });
      if (iter == allSupportedScanModes.end()) {
        RCLCPP_ERROR(
          this->get_logger(), "scan mode `%s' is not supported by lidar, supported modes ('%zd'):",
          scan_mode_.c_str(), allSupportedScanModes.size());
        for (const auto & it : allSupportedScanModes) {
          RCLCPP_ERROR(
            this->get_logger(), "%s: max_distance: %.1f m, Point number: %.1fK",
            it.scan_mode, it.max_distance, (1000 / it.us_per_sample));
        }
        op_result = RESULT_OPERATION_FAIL;
        return false;
      } else {
        op_result = m_drv->startScanExpress(
          false /* not force scan */, iter->id, 0,
          &current_scan_mode);
      }
    }
  }

  /* verify we set the scan mode */
  if (!IS_OK(op_result)) {
    RCLCPP_ERROR(this->get_logger(), "Cannot start scan: '%08x'", op_result);
    return false;
  }

  // default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
  m_angle_compensate_multiple =
    static_cast<int>(1000 * 1000 / current_scan_mode.us_per_sample / 10.0 / 360.0);
  if (m_angle_compensate_multiple < 1) {
    m_angle_compensate_multiple = 1;
  }
  max_distance = current_scan_mode.max_distance;
  RCLCPP_INFO(
    this->get_logger(),
    "current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d, flip_x_axis %d", current_scan_mode.scan_mode,
    current_scan_mode.max_distance, (1000 / current_scan_mode.us_per_sample),
    m_angle_compensate_multiple, flip_x_axis_);
  return true;
}

void rplidar_node::publish_loop()
{
  rclcpp::Time start_scan_time;
  rclcpp::Time end_scan_time;
  u_result op_result;
  size_t count = 360 * 8;
  auto nodes = std::make_unique<rplidar_response_measurement_node_hq_t[]>(count);

  if (auto_standby_) {
    if (m_publisher->get_subscription_count() > 0 && !m_running) {
      this->start();
    } else if (m_publisher->get_subscription_count() == 0) {
      if (m_running) {
        this->stop();
      }
      return;
    }
  }

  start_scan_time = this->now();
  op_result = m_drv->grabScanDataHq(nodes.get(), count);
  end_scan_time = this->now();
  double scan_duration = (end_scan_time - start_scan_time).nanoseconds() * 1E-9;

  if (op_result != RESULT_OK) {
    return;
  }
  op_result = m_drv->ascendScanData(nodes.get(), count);
  angle_min = deg_2_rad(0.0f);
  angle_max = deg_2_rad(359.0f);
  if (op_result == RESULT_OK) {
    if (angle_compensate_) {
      const int angle_compensate_nodes_count = 360 * m_angle_compensate_multiple;
      int angle_compensate_offset = 0;
      auto angle_compensate_nodes = std::make_unique<rplidar_response_measurement_node_hq_t[]>(
        angle_compensate_nodes_count);
      memset(
        angle_compensate_nodes.get(), 0,
        angle_compensate_nodes_count * sizeof(rplidar_response_measurement_node_hq_t));

      size_t i = 0, j = 0;
      for (; i < count; i++) {
        if (nodes[i].dist_mm_q2 != 0) {
          float angle = getAngle(nodes[i]);
          int angle_value = (int)(angle * m_angle_compensate_multiple);
          if ((angle_value - angle_compensate_offset) < 0) {angle_compensate_offset = angle_value;}
          for (j = 0; j < m_angle_compensate_multiple; j++) {
            angle_compensate_nodes[angle_value - angle_compensate_offset + j] = nodes[i];
          }
        }
      }

      publish_scan(scan_duration, std::move(angle_compensate_nodes), angle_compensate_nodes_count);
    } else {
      int start_node = 0, end_node = 0;
      int i = 0;
      // find the first valid node and last valid node
      while (nodes[i++].dist_mm_q2 == 0) {}
      start_node = i - 1;
      i = count - 1;
      while (nodes[i--].dist_mm_q2 == 0) {}
      end_node = i + 1;

      angle_min = deg_2_rad(getAngle(nodes[start_node]));
      angle_max = deg_2_rad(getAngle(nodes[end_node]));
      auto valid = std::make_unique<rplidar_response_measurement_node_hq_t[]>(
        end_node - start_node + 1);
      for (size_t x = start_node, y = 0; x < end_node; ++x, ++y) {
        valid[y] = nodes[x];
      }
      publish_scan(scan_duration, std::move(valid), end_node - start_node + 1);
    }
  } else if (op_result == RESULT_OPERATION_FAIL) {
    // All the data is invalid, just publish them
    float angle_min = deg_2_rad(0.0f);
    float angle_max = deg_2_rad(359.0f);

    publish_scan(scan_duration, std::move(nodes), count);
  }
}

void rplidar_node::start()
{
  if (nullptr == m_drv) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Start");
  m_drv->startMotor();
  if (!set_scan_mode()) {
    this->stop();
    RCLCPP_ERROR(this->get_logger(), "Failed to set scan mode");
    RPlidarDriver::DisposeDriver(m_drv);
    exit(1);
  }
  m_running = true;
}

void rplidar_node::stop()
{
  if (nullptr == m_drv) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Stop");
  m_drv->stop();
  m_drv->stopMotor();
  m_running = false;
}

}  // namespace rplidar_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rplidar_ros::rplidar_node)
