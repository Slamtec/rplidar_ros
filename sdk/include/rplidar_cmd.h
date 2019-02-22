/*
 *  RPLIDAR SDK
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
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

#include "rplidar_protocol.h"

// Commands
//-----------------------------------------

// Commands without payload and response
#define RPLIDAR_CMD_STOP               0x25
#define RPLIDAR_CMD_SCAN               0x20
#define RPLIDAR_CMD_FORCE_SCAN         0x21
#define RPLIDAR_CMD_RESET              0x40


// Commands without payload but have response
#define RPLIDAR_CMD_GET_DEVICE_INFO    0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALTH  0x52

#define RPLIDAR_CMD_GET_SAMPLERATE     0x59 //added in fw 1.17

#define RPLIDAR_CMD_HQ_MOTOR_SPEED_CTRL      0xA8

// Commands with payload and have response
#define RPLIDAR_CMD_EXPRESS_SCAN       0x82 //added in fw 1.17
#define RPLIDAR_CMD_HQ_SCAN                  0x83 //added in fw 1.24
#define RPLIDAR_CMD_GET_LIDAR_CONF           0x84 //added in fw 1.24
#define RPLIDAR_CMD_SET_LIDAR_CONF           0x85 //added in fw 1.24
//add for A2 to set RPLIDAR motor pwm when using accessory board
#define RPLIDAR_CMD_SET_MOTOR_PWM      0xF0
#define RPLIDAR_CMD_GET_ACC_BOARD_FLAG 0xFF

#if defined(_WIN32)
#pragma pack(1)
#endif


// Payloads
// ------------------------------------------
#define RPLIDAR_EXPRESS_SCAN_MODE_NORMAL      0 
#define RPLIDAR_EXPRESS_SCAN_MODE_FIXANGLE    0  // won't been supported but keep to prevent build fail
//for express working flag(extending express scan protocol)
#define RPLIDAR_EXPRESS_SCAN_FLAG_BOOST                 0x0001 
#define RPLIDAR_EXPRESS_SCAN_FLAG_SUNLIGHT_REJECTION    0x0002

//for ultra express working flag
#define RPLIDAR_ULTRAEXPRESS_SCAN_FLAG_STD                 0x0001 
#define RPLIDAR_ULTRAEXPRESS_SCAN_FLAG_HIGH_SENSITIVITY    0x0002

#define RPLIDAR_HQ_SCAN_FLAG_CCW            (0x1<<0)
#define RPLIDAR_HQ_SCAN_FLAG_RAW_ENCODER    (0x1<<1)
#define RPLIDAR_HQ_SCAN_FLAG_RAW_DISTANCE   (0x1<<2)

typedef struct _rplidar_payload_express_scan_t {
    _u8   working_mode;
    _u16  working_flags;
    _u16  param;
} __attribute__((packed)) rplidar_payload_express_scan_t;

typedef struct _rplidar_payload_hq_scan_t {
    _u8  flag;
    _u8   reserved[32];
} __attribute__((packed)) rplidar_payload_hq_scan_t;

typedef struct _rplidar_payload_get_scan_conf_t {
    _u32  type;
    _u8   reserved[32];
} __attribute__((packed)) rplidar_payload_get_scan_conf_t;
#define MAX_MOTOR_PWM               1023
#define DEFAULT_MOTOR_PWM           660
typedef struct _rplidar_payload_motor_pwm_t {
    _u16 pwm_value;
} __attribute__((packed)) rplidar_payload_motor_pwm_t;

typedef struct _rplidar_payload_acc_board_flag_t {
    _u32 reserved;
} __attribute__((packed)) rplidar_payload_acc_board_flag_t;

// Response
// ------------------------------------------
#define RPLIDAR_ANS_TYPE_DEVINFO          0x4
#define RPLIDAR_ANS_TYPE_DEVHEALTH        0x6

#define RPLIDAR_ANS_TYPE_MEASUREMENT                0x81
// Added in FW ver 1.17
#define RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED       0x82
#define RPLIDAR_ANS_TYPE_MEASUREMENT_HQ            0x83


// Added in FW ver 1.17
#define RPLIDAR_ANS_TYPE_SAMPLE_RATE      0x15
//added in FW ver 1.23alpha
#define RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA  0x84
//added in FW ver 1.24
#define RPLIDAR_ANS_TYPE_GET_LIDAR_CONF     0x20
#define RPLIDAR_ANS_TYPE_SET_LIDAR_CONF     0x21
#define RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED        0x85
#define RPLIDAR_ANS_TYPE_ACC_BOARD_FLAG   0xFF

#define RPLIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK      (0x1)
typedef struct _rplidar_response_acc_board_flag_t {
    _u32 support_flag;
} __attribute__((packed)) rplidar_response_acc_board_flag_t;


#define RPLIDAR_STATUS_OK                 0x0
#define RPLIDAR_STATUS_WARNING            0x1
#define RPLIDAR_STATUS_ERROR              0x2

#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2

#define RPLIDAR_RESP_HQ_FLAG_SYNCBIT               (0x1<<0)

#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

typedef struct _rplidar_response_sample_rate_t {
    _u16  std_sample_duration_us;
    _u16  express_sample_duration_us;
} __attribute__((packed)) rplidar_response_sample_rate_t;

typedef struct _rplidar_response_measurement_node_t {
    _u8    sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
    _u16   angle_q6_checkbit; // check_bit:1;angle_q6:15;
    _u16   distance_q2;
} __attribute__((packed)) rplidar_response_measurement_node_t;

//[distance_sync flags]
#define RPLIDAR_RESP_MEASUREMENT_EXP_ANGLE_MASK           (0x3)
#define RPLIDAR_RESP_MEASUREMENT_EXP_DISTANCE_MASK        (0xFC)

typedef struct _rplidar_response_cabin_nodes_t {
    _u16   distance_angle_1; // see [distance_sync flags]
    _u16   distance_angle_2; // see [distance_sync flags]
    _u8    offset_angles_q3;  
} __attribute__((packed)) rplidar_response_cabin_nodes_t;   


#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1               0xA
#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2               0x5

#define RPLIDAR_RESP_MEASUREMENT_HQ_SYNC                  0xA5

#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT              (0x1<<15)

typedef struct _rplidar_response_capsule_measurement_nodes_t {
    _u8                             s_checksum_1; // see [s_checksum_1]
    _u8                             s_checksum_2; // see [s_checksum_1]
    _u16                            start_angle_sync_q6;
    rplidar_response_cabin_nodes_t  cabins[16];
} __attribute__((packed)) rplidar_response_capsule_measurement_nodes_t;

typedef struct _rplidar_response_dense_cabin_nodes_t {
    _u16   distance; 
} __attribute__((packed)) rplidar_response_dense_cabin_nodes_t;

typedef struct _rplidar_response_dense_capsule_measurement_nodes_t {
    _u8                             s_checksum_1; // see [s_checksum_1]
    _u8                             s_checksum_2; // see [s_checksum_1]
    _u16                            start_angle_sync_q6;
    rplidar_response_dense_cabin_nodes_t  cabins[40];
} __attribute__((packed)) rplidar_response_dense_capsule_measurement_nodes_t;

// ext1 : x2 boost mode

#define RPLIDAR_RESP_MEASUREMENT_EXP_ULTRA_MAJOR_BITS     12
#define RPLIDAR_RESP_MEASUREMENT_EXP_ULTRA_PREDICT_BITS   10

typedef struct _rplidar_response_ultra_cabin_nodes_t {
    // 31                                              0
    // | predict2 10bit | predict1 10bit | major 12bit |
    _u32 combined_x3;
} __attribute__((packed)) rplidar_response_ultra_cabin_nodes_t;  

typedef struct _rplidar_response_ultra_capsule_measurement_nodes_t {
    _u8                             s_checksum_1; // see [s_checksum_1]
    _u8                             s_checksum_2; // see [s_checksum_1]
    _u16                            start_angle_sync_q6;
    rplidar_response_ultra_cabin_nodes_t  ultra_cabins[32];
} __attribute__((packed)) rplidar_response_ultra_capsule_measurement_nodes_t;

typedef struct rplidar_response_measurement_node_hq_t {
    _u16   angle_z_q14; 
    _u32   dist_mm_q2; 
    _u8    quality;  
    _u8    flag;
} __attribute__((packed)) rplidar_response_measurement_node_hq_t;

typedef struct _rplidar_response_hq_capsule_measurement_nodes_t{
    _u8 sync_byte;
    _u64 time_stamp;
    rplidar_response_measurement_node_hq_t node_hq[16];
    _u32  crc32;
}__attribute__((packed)) rplidar_response_hq_capsule_measurement_nodes_t;


#   define RPLIDAR_CONF_SCAN_COMMAND_STD            0
#   define RPLIDAR_CONF_SCAN_COMMAND_EXPRESS        1
#   define RPLIDAR_CONF_SCAN_COMMAND_HQ             2
#   define RPLIDAR_CONF_SCAN_COMMAND_BOOST          3
#   define RPLIDAR_CONF_SCAN_COMMAND_STABILITY      4
#   define RPLIDAR_CONF_SCAN_COMMAND_SENSITIVITY    5

#define RPLIDAR_CONF_ANGLE_RANGE                    0x00000000
#define RPLIDAR_CONF_DESIRED_ROT_FREQ               0x00000001
#define RPLIDAR_CONF_SCAN_COMMAND_BITMAP            0x00000002
#define RPLIDAR_CONF_MIN_ROT_FREQ                   0x00000004
#define RPLIDAR_CONF_MAX_ROT_FREQ                   0x00000005
#define RPLIDAR_CONF_MAX_DISTANCE                   0x00000060
        
#define RPLIDAR_CONF_SCAN_MODE_COUNT                0x00000070
#define RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE        0x00000071
#define RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE         0x00000074
#define RPLIDAR_CONF_SCAN_MODE_ANS_TYPE             0x00000075
#define RPLIDAR_CONF_SCAN_MODE_TYPICAL              0x0000007C
#define RPLIDAR_CONF_SCAN_MODE_NAME                 0x0000007F
#define RPLIDAR_EXPRESS_SCAN_STABILITY_BITMAP                 4
#define RPLIDAR_EXPRESS_SCAN_SENSITIVITY_BITMAP               5

typedef struct _rplidar_response_get_lidar_conf{
    _u32 type;
    _u8  payload[0];
}__attribute__((packed)) rplidar_response_get_lidar_conf_t;

typedef struct _rplidar_response_set_lidar_conf{
    _u32 result;
}__attribute__((packed)) rplidar_response_set_lidar_conf_t;


typedef struct _rplidar_response_device_info_t {
    _u8   model;
    _u16  firmware_version;
    _u8   hardware_version;
    _u8   serialnum[16];
} __attribute__((packed)) rplidar_response_device_info_t;

typedef struct _rplidar_response_device_health_t {
    _u8   status;
    _u16  error_code;
} __attribute__((packed)) rplidar_response_device_health_t;

// Definition of the variable bit scale encoding mechanism
#define RPLIDAR_VARBITSCALE_X2_SRC_BIT  9
#define RPLIDAR_VARBITSCALE_X4_SRC_BIT  11
#define RPLIDAR_VARBITSCALE_X8_SRC_BIT  12
#define RPLIDAR_VARBITSCALE_X16_SRC_BIT 14

#define RPLIDAR_VARBITSCALE_X2_DEST_VAL 512
#define RPLIDAR_VARBITSCALE_X4_DEST_VAL 1280
#define RPLIDAR_VARBITSCALE_X8_DEST_VAL 1792
#define RPLIDAR_VARBITSCALE_X16_DEST_VAL 3328

#define RPLIDAR_VARBITSCALE_GET_SRC_MAX_VAL_BY_BITS(_BITS_) \
    (  (((0x1<<(_BITS_)) - RPLIDAR_VARBITSCALE_X16_DEST_VAL)<<4) + \
       ((RPLIDAR_VARBITSCALE_X16_DEST_VAL - RPLIDAR_VARBITSCALE_X8_DEST_VAL)<<3) + \
       ((RPLIDAR_VARBITSCALE_X8_DEST_VAL - RPLIDAR_VARBITSCALE_X4_DEST_VAL)<<2) + \
       ((RPLIDAR_VARBITSCALE_X4_DEST_VAL - RPLIDAR_VARBITSCALE_X2_DEST_VAL)<<1) + \
       RPLIDAR_VARBITSCALE_X2_DEST_VAL - 1)


#if defined(_WIN32)
#pragma pack()
#endif
