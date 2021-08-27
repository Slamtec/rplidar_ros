/*
 *  RPLIDAR SDK
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014-2020 Shanghai Slamtec Co., Ltd.
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

#pragma once
#include "sl_lidar_cmd.h"
#include "rplidar_protocol.h"

// Commands
//-----------------------------------------

#define RPLIDAR_AUTOBAUD_MAGICBYTE     SL_LIDAR_AUTOBAUD_MAGICBYTE

// Commands without payload and response
#define RPLIDAR_CMD_STOP               SL_LIDAR_CMD_STOP
#define RPLIDAR_CMD_SCAN               SL_LIDAR_CMD_SCAN
#define RPLIDAR_CMD_FORCE_SCAN         SL_LIDAR_CMD_FORCE_SCAN
#define RPLIDAR_CMD_RESET              SL_LIDAR_CMD_RESET


// Commands without payload but have response
#define RPLIDAR_CMD_GET_DEVICE_INFO          SL_LIDAR_CMD_GET_DEVICE_INFO
#define RPLIDAR_CMD_GET_DEVICE_HEALTH        SL_LIDAR_CMD_GET_DEVICE_HEALTH

#define RPLIDAR_CMD_GET_SAMPLERATE           SL_LIDAR_CMD_GET_SAMPLERATE //added in fw 1.17

#define RPLIDAR_CMD_HQ_MOTOR_SPEED_CTRL      SL_LIDAR_CMD_HQ_MOTOR_SPEED_CTRL

// Commands with payload but no response
#define RPLIDAR_CMD_NEW_BAUDRATE_CONFIRM     SL_LIDAR_CMD_NEW_BAUDRATE_CONFIRM //added in fw 1.30

// Commands with payload and have response
#define RPLIDAR_CMD_EXPRESS_SCAN            SL_LIDAR_CMD_EXPRESS_SCAN //added in fw 1.17
#define RPLIDAR_CMD_HQ_SCAN                 SL_LIDAR_CMD_HQ_SCAN //added in fw 1.24
#define RPLIDAR_CMD_GET_LIDAR_CONF          SL_LIDAR_CMD_GET_LIDAR_CONF //added in fw 1.24
#define RPLIDAR_CMD_SET_LIDAR_CONF          SL_LIDAR_CMD_SET_LIDAR_CONF //added in fw 1.24
//add for A2 to set RPLIDAR motor pwm when using accessory board
#define RPLIDAR_CMD_SET_MOTOR_PWM           SL_LIDAR_CMD_SET_MOTOR_PWM
#define RPLIDAR_CMD_GET_ACC_BOARD_FLAG      SL_LIDAR_CMD_GET_ACC_BOARD_FLAG

#if defined(_WIN32)
#pragma pack(1)
#endif


// Payloads
// ------------------------------------------
#define RPLIDAR_EXPRESS_SCAN_MODE_NORMAL      SL_LIDAR_EXPRESS_SCAN_MODE_NORMAL 
#define RPLIDAR_EXPRESS_SCAN_MODE_FIXANGLE    SL_LIDAR_EXPRESS_SCAN_MODE_FIXANGLE  // won't been supported but keep to prevent build fail
//for express working flag(extending express scan protocol)
#define RPLIDAR_EXPRESS_SCAN_FLAG_BOOST                 SL_LIDAR_EXPRESS_SCAN_FLAG_BOOST 
#define RPLIDAR_EXPRESS_SCAN_FLAG_SUNLIGHT_REJECTION    SL_LIDAR_EXPRESS_SCAN_FLAG_SUNLIGHT_REJECTION

//for ultra express working flag
#define RPLIDAR_ULTRAEXPRESS_SCAN_FLAG_STD                 SL_LIDAR_ULTRAEXPRESS_SCAN_FLAG_STD 
#define RPLIDAR_ULTRAEXPRESS_SCAN_FLAG_HIGH_SENSITIVITY    SL_LIDAR_ULTRAEXPRESS_SCAN_FLAG_HIGH_SENSITIVITY

#define RPLIDAR_HQ_SCAN_FLAG_CCW            (0x1<<0)
#define RPLIDAR_HQ_SCAN_FLAG_RAW_ENCODER    (0x1<<1)
#define RPLIDAR_HQ_SCAN_FLAG_RAW_DISTANCE   (0x1<<2)

typedef sl_lidar_payload_express_scan_t   rplidar_payload_express_scan_t;
typedef sl_lidar_payload_hq_scan_t        rplidar_payload_hq_scan_t;
typedef sl_lidar_payload_get_scan_conf_t  rplidar_payload_get_scan_conf_t;
typedef sl_lidar_payload_motor_pwm_t      rplidar_payload_motor_pwm_t;
typedef sl_lidar_payload_acc_board_flag_t rplidar_payload_acc_board_flag_t;
typedef sl_lidar_payload_set_scan_conf_t  rplidar_payload_set_scan_conf_t;
typedef sl_lidar_payload_new_bps_confirmation_t  rplidar_payload_new_bps_confirmation_t;

// Response
// ------------------------------------------
#define RPLIDAR_ANS_TYPE_DEVINFO                                 SL_LIDAR_ANS_TYPE_DEVINFO
#define RPLIDAR_ANS_TYPE_DEVHEALTH                               SL_LIDAR_ANS_TYPE_DEVHEALTH
#define RPLIDAR_ANS_TYPE_MEASUREMENT                             SL_LIDAR_ANS_TYPE_MEASUREMENT
// Added in FW ver 1.17
#define RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED                    SL_LIDAR_ANS_TYPE_MEASUREMENT_CAPSULED
#define RPLIDAR_ANS_TYPE_MEASUREMENT_HQ                          SL_LIDAR_ANS_TYPE_MEASUREMENT_HQ
// Added in FW ver 1.17
#define RPLIDAR_ANS_TYPE_SAMPLE_RATE                             SL_LIDAR_ANS_TYPE_SAMPLE_RATE
//added in FW ver 1.23alpha
#define RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA              SL_LIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA
//added in FW ver 1.24
#define RPLIDAR_ANS_TYPE_GET_LIDAR_CONF                          SL_LIDAR_ANS_TYPE_GET_LIDAR_CONF
#define RPLIDAR_ANS_TYPE_SET_LIDAR_CONF                          SL_LIDAR_ANS_TYPE_SET_LIDAR_CONF
#define RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED              SL_LIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED
#define RPLIDAR_ANS_TYPE_ACC_BOARD_FLAG                          SL_LIDAR_ANS_TYPE_ACC_BOARD_FLAG

#define RPLIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK      SL_LIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK

typedef sl_lidar_response_acc_board_flag_t rplidar_response_acc_board_flag_t;


#define RPLIDAR_STATUS_OK                 SL_LIDAR_STATUS_OK
#define RPLIDAR_STATUS_WARNING            SL_LIDAR_STATUS_WARNING
#define RPLIDAR_STATUS_ERROR              SL_LIDAR_STATUS_ERROR

#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        SL_LIDAR_RESP_MEASUREMENT_SYNCBIT
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT
#define RPLIDAR_RESP_HQ_FLAG_SYNCBIT            SL_LIDAR_RESP_HQ_FLAG_SYNCBIT
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT       SL_LIDAR_RESP_MEASUREMENT_CHECKBIT
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT

typedef sl_lidar_response_sample_rate_t rplidar_response_sample_rate_t;
typedef sl_lidar_response_measurement_node_t rplidar_response_measurement_node_t;

//[distance_sync flags]
#define RPLIDAR_RESP_MEASUREMENT_EXP_ANGLE_MASK           SL_LIDAR_RESP_MEASUREMENT_EXP_ANGLE_MASK
#define RPLIDAR_RESP_MEASUREMENT_EXP_DISTANCE_MASK        SL_LIDAR_RESP_MEASUREMENT_EXP_DISTANCE_MASK

typedef sl_lidar_response_cabin_nodes_t rplidar_response_cabin_nodes_t;


#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1               SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_1
#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2               SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_2
#define RPLIDAR_RESP_MEASUREMENT_HQ_SYNC                  SL_LIDAR_RESP_MEASUREMENT_HQ_SYNC
#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT              SL_LIDAR_RESP_MEASUREMENT_EXP_SYNCBIT


typedef sl_lidar_response_capsule_measurement_nodes_t         rplidar_response_capsule_measurement_nodes_t;
typedef sl_lidar_response_dense_cabin_nodes_t                 rplidar_response_dense_cabin_nodes_t;
typedef sl_lidar_response_dense_capsule_measurement_nodes_t   rplidar_response_dense_capsule_measurement_nodes_t;

// ext1 : x2 boost mode

#define RPLIDAR_RESP_MEASUREMENT_EXP_ULTRA_MAJOR_BITS     SL_LIDAR_RESP_MEASUREMENT_EXP_ULTRA_MAJOR_BITS
#define RPLIDAR_RESP_MEASUREMENT_EXP_ULTRA_PREDICT_BITS   SL_LIDAR_RESP_MEASUREMENT_EXP_ULTRA_PREDICT_BITS

typedef sl_lidar_response_ultra_cabin_nodes_t                      rplidar_response_ultra_cabin_nodes_t;
typedef sl_lidar_response_ultra_capsule_measurement_nodes_t        rplidar_response_ultra_capsule_measurement_nodes_t;
typedef sl_lidar_response_measurement_node_hq_t                    rplidar_response_measurement_node_hq_t;
typedef sl_lidar_response_hq_capsule_measurement_nodes_t           rplidar_response_hq_capsule_measurement_nodes_t;


#   define RPLIDAR_CONF_SCAN_COMMAND_STD            SL_LIDAR_CONF_SCAN_COMMAND_STD
#   define RPLIDAR_CONF_SCAN_COMMAND_EXPRESS        SL_LIDAR_CONF_SCAN_COMMAND_EXPRESS
#   define RPLIDAR_CONF_SCAN_COMMAND_HQ             SL_LIDAR_CONF_SCAN_COMMAND_HQ
#   define RPLIDAR_CONF_SCAN_COMMAND_BOOST          SL_LIDAR_CONF_SCAN_COMMAND_BOOST
#   define RPLIDAR_CONF_SCAN_COMMAND_STABILITY      SL_LIDAR_CONF_SCAN_COMMAND_STABILITY
#   define RPLIDAR_CONF_SCAN_COMMAND_SENSITIVITY    SL_LIDAR_CONF_SCAN_COMMAND_SENSITIVITY

#define RPLIDAR_CONF_ANGLE_RANGE                    SL_LIDAR_CONF_ANGLE_RANGE
#define RPLIDAR_CONF_DESIRED_ROT_FREQ               SL_LIDAR_CONF_DESIRED_ROT_FREQ
#define RPLIDAR_CONF_SCAN_COMMAND_BITMAP            SL_LIDAR_CONF_SCAN_COMMAND_BITMAP
#define RPLIDAR_CONF_MIN_ROT_FREQ                   SL_LIDAR_CONF_MIN_ROT_FREQ
#define RPLIDAR_CONF_MAX_ROT_FREQ                   SL_LIDAR_CONF_MAX_ROT_FREQ
#define RPLIDAR_CONF_MAX_DISTANCE                   SL_LIDAR_CONF_MAX_DISTANCE
        
#define RPLIDAR_CONF_SCAN_MODE_COUNT                SL_LIDAR_CONF_SCAN_MODE_COUNT
#define RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE        SL_LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE
#define RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE         SL_LIDAR_CONF_SCAN_MODE_MAX_DISTANCE
#define RPLIDAR_CONF_SCAN_MODE_ANS_TYPE             SL_LIDAR_CONF_SCAN_MODE_ANS_TYPE
#define RPLIDAR_CONF_SCAN_MODE_TYPICAL              SL_LIDAR_CONF_SCAN_MODE_TYPICAL
#define RPLIDAR_CONF_SCAN_MODE_NAME                 SL_LIDAR_CONF_SCAN_MODE_NAME
#define RPLIDAR_EXPRESS_SCAN_STABILITY_BITMAP                 SL_LIDAR_EXPRESS_SCAN_STABILITY_BITMAP
#define RPLIDAR_EXPRESS_SCAN_SENSITIVITY_BITMAP               SL_LIDAR_EXPRESS_SCAN_SENSITIVITY_BITMAP
#define RPLIDAR_CONF_LIDAR_STATIC_IP_ADDR           SL_LIDAR_CONF_LIDAR_STATIC_IP_ADDR
#define RPLIDAR_CONF_LIDAR_MAC_ADDR                 SL_LIDAR_CONF_LIDAR_MAC_ADDR

#define RPLIDAR_CONF_DETECTED_SERIAL_BPS            SL_LIDAR_CONF_DETECTED_SERIAL_BPS

typedef sl_lidar_response_get_lidar_conf_t       rplidar_response_get_lidar_conf_t;
typedef sl_lidar_response_set_lidar_conf_t       rplidar_response_set_lidar_conf_t;
typedef sl_lidar_response_device_info_t          rplidar_response_device_info_t;
typedef sl_lidar_response_device_health_t        rplidar_response_device_health_t;
typedef sl_lidar_ip_conf_t                       rplidar_ip_conf_t;
typedef sl_lidar_response_device_macaddr_info_t  rplidar_response_device_macaddr_info_t;

// Definition of the variable bit scale encoding mechanism
#define RPLIDAR_VARBITSCALE_X2_SRC_BIT  SL_LIDAR_VARBITSCALE_X2_SRC_BIT
#define RPLIDAR_VARBITSCALE_X4_SRC_BIT  SL_LIDAR_VARBITSCALE_X4_SRC_BIT
#define RPLIDAR_VARBITSCALE_X8_SRC_BIT  SL_LIDAR_VARBITSCALE_X8_SRC_BIT
#define RPLIDAR_VARBITSCALE_X16_SRC_BIT SL_LIDAR_VARBITSCALE_X16_SRC_BIT

#define RPLIDAR_VARBITSCALE_X2_DEST_VAL SL_LIDAR_VARBITSCALE_X2_DEST_VAL
#define RPLIDAR_VARBITSCALE_X4_DEST_VAL SL_LIDAR_VARBITSCALE_X4_DEST_VAL
#define RPLIDAR_VARBITSCALE_X8_DEST_VAL SL_LIDAR_VARBITSCALE_X8_DEST_VAL
#define RPLIDAR_VARBITSCALE_X16_DEST_VAL SL_LIDAR_VARBITSCALE_X16_DEST_VAL

#define RPLIDAR_VARBITSCALE_GET_SRC_MAX_VAL_BY_BITS(_BITS_)   SL_LIDAR_VARBITSCALE_GET_SRC_MAX_VAL_BY_BITS(_BITS_)

#if defined(_WIN32)
#pragma pack()
#endif
