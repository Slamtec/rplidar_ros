/*
* Slamtec LIDAR SDK
*
* sl_lidar_cmd.h
*
* Copyright (c) 2020 Shanghai Slamtec Co., Ltd.
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

#include "sl_lidar_protocol.h"

 // Commands
 //-----------------------------------------


#define SL_LIDAR_AUTOBAUD_MAGICBYTE         0x41

 // Commands without payload and response
#define SL_LIDAR_CMD_STOP                   0x25
#define SL_LIDAR_CMD_SCAN                   0x20
#define SL_LIDAR_CMD_FORCE_SCAN             0x21
#define SL_LIDAR_CMD_RESET                  0x40

// Commands with payload but no response
#define SL_LIDAR_CMD_NEW_BAUDRATE_CONFIRM   0x90 //added in fw 1.30

// Commands without payload but have response
#define SL_LIDAR_CMD_GET_DEVICE_INFO        0x50
#define SL_LIDAR_CMD_GET_DEVICE_HEALTH      0x52

#define SL_LIDAR_CMD_GET_SAMPLERATE         0x59 //added in fw 1.17

#define SL_LIDAR_CMD_HQ_MOTOR_SPEED_CTRL    0xA8


// Commands with payload and have response
#define SL_LIDAR_CMD_EXPRESS_SCAN           0x82 //added in fw 1.17
#define SL_LIDAR_CMD_HQ_SCAN                0x83 //added in fw 1.24
#define SL_LIDAR_CMD_GET_LIDAR_CONF         0x84 //added in fw 1.24
#define SL_LIDAR_CMD_SET_LIDAR_CONF         0x85 //added in fw 1.24
//add for A2 to set RPLIDAR motor pwm when using accessory board
#define SL_LIDAR_CMD_SET_MOTOR_PWM          0xF0
#define SL_LIDAR_CMD_GET_ACC_BOARD_FLAG     0xFF

#if defined(_WIN32)
#pragma pack(1)
#endif


// Payloads
// ------------------------------------------
#define SL_LIDAR_EXPRESS_SCAN_MODE_NORMAL      0 
#define SL_LIDAR_EXPRESS_SCAN_MODE_FIXANGLE    0  // won't been supported but keep to prevent build fail
//for express working flag(extending express scan protocol)
#define SL_LIDAR_EXPRESS_SCAN_FLAG_BOOST                 0x0001 
#define SL_LIDAR_EXPRESS_SCAN_FLAG_SUNLIGHT_REJECTION    0x0002

//for ultra express working flag
#define SL_LIDAR_ULTRAEXPRESS_SCAN_FLAG_STD                 0x0001 
#define SL_LIDAR_ULTRAEXPRESS_SCAN_FLAG_HIGH_SENSITIVITY    0x0002

typedef struct _sl_lidar_payload_express_scan_t
{
    sl_u8   working_mode;
    sl_u16  working_flags;
    sl_u16  param;
} __attribute__((packed)) sl_lidar_payload_express_scan_t;

typedef struct _sl_lidar_payload_hq_scan_t
{
    sl_u8  flag;
    sl_u8   reserved[32];
} __attribute__((packed)) sl_lidar_payload_hq_scan_t;

typedef struct _sl_lidar_payload_get_scan_conf_t
{
    sl_u32  type;
    sl_u8   reserved[32];
} __attribute__((packed)) sl_lidar_payload_get_scan_conf_t;

typedef struct _sl_payload_set_scan_conf_t {
    sl_u32  type;
} __attribute__((packed)) sl_lidar_payload_set_scan_conf_t;


#define DEFAULT_MOTOR_SPEED         (0xFFFFu)

typedef struct _sl_lidar_payload_motor_pwm_t
{
    sl_u16 pwm_value;
} __attribute__((packed)) sl_lidar_payload_motor_pwm_t;

typedef struct _sl_lidar_payload_acc_board_flag_t
{
    sl_u32 reserved;
} __attribute__((packed)) sl_lidar_payload_acc_board_flag_t;

typedef struct _sl_lidar_payload_hq_spd_ctrl_t {
    sl_u16  rpm;
} __attribute__((packed))sl_lidar_payload_hq_spd_ctrl_t;


typedef struct _sl_lidar_payload_new_bps_confirmation_t {
    sl_u16   flag; // reserved, must be 0x5F5F
    sl_u32  required_bps;
    sl_u16  param;
} __attribute__((packed)) sl_lidar_payload_new_bps_confirmation_t;

// Response
// ------------------------------------------
#define SL_LIDAR_ANS_TYPE_DEVINFO          0x4
#define SL_LIDAR_ANS_TYPE_DEVHEALTH        0x6

#define SL_LIDAR_ANS_TYPE_MEASUREMENT                0x81
// Added in FW ver 1.17
#define SL_LIDAR_ANS_TYPE_MEASUREMENT_CAPSULED       0x82
#define SL_LIDAR_ANS_TYPE_MEASUREMENT_HQ            0x83
#define SL_LIDAR_ANS_TYPE_MEASUREMENTT_ULTRA_DENSE_CAPSULED 0x86


// Added in FW ver 1.17
#define SL_LIDAR_ANS_TYPE_SAMPLE_RATE      0x15
//added in FW ver 1.23alpha
#define SL_LIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA  0x84
//added in FW ver 1.24
#define SL_LIDAR_ANS_TYPE_GET_LIDAR_CONF     0x20
#define SL_LIDAR_ANS_TYPE_SET_LIDAR_CONF     0x21
#define SL_LIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED        0x85
#define SL_LIDAR_ANS_TYPE_ACC_BOARD_FLAG   0xFF

#define SL_LIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK      (0x1)
typedef struct _sl_lidar_response_acc_board_flag_t
{
    sl_u32 support_flag;
} __attribute__((packed)) sl_lidar_response_acc_board_flag_t;


#define SL_LIDAR_STATUS_OK                 0x0
#define SL_LIDAR_STATUS_WARNING            0x1
#define SL_LIDAR_STATUS_ERROR              0x2

#define SL_LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2

#define SL_LIDAR_RESP_HQ_FLAG_SYNCBIT               (0x1<<0)

#define SL_LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

typedef struct _sl_lidar_response_sample_rate_t
{
    sl_u16  std_sample_duration_us;
    sl_u16  express_sample_duration_us;
} __attribute__((packed)) sl_lidar_response_sample_rate_t;

typedef struct _sl_lidar_response_measurement_node_t
{
    sl_u8    sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
    sl_u16   angle_q6_checkbit; // check_bit:1;angle_q6:15;
    sl_u16   distance_q2;
} __attribute__((packed)) sl_lidar_response_measurement_node_t;

//[distance_sync flags]
#define SL_LIDAR_RESP_MEASUREMENT_EXP_ANGLE_MASK           (0x3)
#define SL_LIDAR_RESP_MEASUREMENT_EXP_DISTANCE_MASK        (0xFC)

typedef struct _sl_lidar_response_cabin_nodes_t
{
    sl_u16   distance_angle_1; // see [distance_sync flags]
    sl_u16   distance_angle_2; // see [distance_sync flags]
    sl_u8    offset_angles_q3;
} __attribute__((packed)) sl_lidar_response_cabin_nodes_t;


#define SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_1               0xA
#define SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_2               0x5

#define SL_LIDAR_RESP_MEASUREMENT_HQ_SYNC                  0xA5

#define SL_LIDAR_RESP_MEASUREMENT_EXP_SYNCBIT              (0x1<<15)

typedef struct _sl_lidar_response_capsule_measurement_nodes_t
{
    sl_u8                             s_checksum_1; // see [s_checksum_1]
    sl_u8                             s_checksum_2; // see [s_checksum_1]
    sl_u16                            start_angle_sync_q6;
    sl_lidar_response_cabin_nodes_t  cabins[16];
} __attribute__((packed)) sl_lidar_response_capsule_measurement_nodes_t;

typedef struct _sl_lidar_response_dense_cabin_nodes_t
{
    sl_u16   distance;
} __attribute__((packed)) sl_lidar_response_dense_cabin_nodes_t;

typedef struct _sl_lidar_response_dense_capsule_measurement_nodes_t
{
    sl_u8                             s_checksum_1; // see [s_checksum_1]
    sl_u8                             s_checksum_2; // see [s_checksum_1]
    sl_u16                            start_angle_sync_q6;
    sl_lidar_response_dense_cabin_nodes_t  cabins[40];
} __attribute__((packed)) sl_lidar_response_dense_capsule_measurement_nodes_t;


typedef struct _sl_lidar_response_ultra_dense_cabin_nodes_t {
    sl_u16  qualityl_distance_scale[2];
    sl_u8   qualityh_array;
} __attribute__((packed)) sl_lidar_response_ultra_dense_cabin_nodes_t;

typedef struct _sl_lidar_response_ultra_dense_capsule_measurement_nodes_t {
    sl_u8                             s_checksum_1; // see [s_checksum_1]
    sl_u8                             s_checksum_2; // see [s_checksum_1]
    sl_u32                            time_stamp;
    sl_u16                            dev_status;
    sl_u16                            start_angle_sync_q6;
    sl_lidar_response_ultra_dense_cabin_nodes_t  cabins[32];
} __attribute__((packed)) sl_lidar_response_ultra_dense_capsule_measurement_nodes_t;


// ext1 : x2 boost mode

#define SL_LIDAR_RESP_MEASUREMENT_EXP_ULTRA_MAJOR_BITS     12
#define SL_LIDAR_RESP_MEASUREMENT_EXP_ULTRA_PREDICT_BITS   10

typedef struct _sl_lidar_response_ultra_cabin_nodes_t
{
    // 31                                              0
    // | predict2 10bit | predict1 10bit | major 12bit |
    sl_u32 combined_x3;
} __attribute__((packed)) sl_lidar_response_ultra_cabin_nodes_t;

typedef struct _sl_lidar_response_ultra_capsule_measurement_nodes_t
{
    sl_u8                             s_checksum_1; // see [s_checksum_1]
    sl_u8                             s_checksum_2; // see [s_checksum_1]
    sl_u16                            start_angle_sync_q6;
    sl_lidar_response_ultra_cabin_nodes_t  ultra_cabins[32];
} __attribute__((packed)) sl_lidar_response_ultra_capsule_measurement_nodes_t;

typedef struct sl_lidar_response_measurement_node_hq_t
{
    sl_u16   angle_z_q14;
    sl_u32   dist_mm_q2;
    sl_u8    quality;
    sl_u8    flag;
} __attribute__((packed)) sl_lidar_response_measurement_node_hq_t;

typedef struct _sl_lidar_response_hq_capsule_measurement_nodes_t
{
    sl_u8 sync_byte;
    sl_u64 time_stamp;
    sl_lidar_response_measurement_node_hq_t node_hq[96];
    sl_u32  crc32;
}__attribute__((packed)) sl_lidar_response_hq_capsule_measurement_nodes_t;


#   define SL_LIDAR_CONF_SCAN_COMMAND_STD            0
#   define SL_LIDAR_CONF_SCAN_COMMAND_EXPRESS        1
#   define SL_LIDAR_CONF_SCAN_COMMAND_HQ             2
#   define SL_LIDAR_CONF_SCAN_COMMAND_BOOST          3
#   define SL_LIDAR_CONF_SCAN_COMMAND_STABILITY      4
#   define SL_LIDAR_CONF_SCAN_COMMAND_SENSITIVITY    5

#define SL_LIDAR_CONF_ANGLE_RANGE                    0x00000000
#define SL_LIDAR_CONF_DESIRED_ROT_FREQ               0x00000001
#define SL_LIDAR_CONF_SCAN_COMMAND_BITMAP            0x00000002
#define SL_LIDAR_CONF_MIN_ROT_FREQ                   0x00000004
#define SL_LIDAR_CONF_MAX_ROT_FREQ                   0x00000005
#define SL_LIDAR_CONF_MAX_DISTANCE                   0x00000060

#define SL_LIDAR_CONF_SCAN_MODE_COUNT                0x00000070
#define SL_LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE        0x00000071
#define SL_LIDAR_CONF_SCAN_MODE_MAX_DISTANCE         0x00000074
#define SL_LIDAR_CONF_SCAN_MODE_ANS_TYPE             0x00000075
#define SL_LIDAR_CONF_LIDAR_MAC_ADDR                 0x00000079
#define SL_LIDAR_CONF_SCAN_MODE_TYPICAL              0x0000007C
#define SL_LIDAR_CONF_SCAN_MODE_NAME                 0x0000007F
#define SL_LIDAR_CONF_DETECTED_SERIAL_BPS            0x000000A1

#define SL_LIDAR_CONF_LIDAR_STATIC_IP_ADDR           0x0001CCC0
#define SL_LIDAR_EXPRESS_SCAN_STABILITY_BITMAP                 4
#define SL_LIDAR_EXPRESS_SCAN_SENSITIVITY_BITMAP               5

typedef struct _sl_lidar_response_get_lidar_conf
{
    sl_u32 type;
    sl_u8  payload[0];
}__attribute__((packed)) sl_lidar_response_get_lidar_conf_t;

typedef struct _sl_lidar_response_set_lidar_conf
{
    sl_u32 result;
}__attribute__((packed)) sl_lidar_response_set_lidar_conf_t;


typedef struct _sl_lidar_response_device_info_t
{
    sl_u8   model;
    sl_u16  firmware_version;
    sl_u8   hardware_version;
    sl_u8   serialnum[16];
} __attribute__((packed)) sl_lidar_response_device_info_t;

typedef struct _sl_lidar_response_device_health_t
{
    sl_u8   status;
    sl_u16  error_code;
} __attribute__((packed)) sl_lidar_response_device_health_t;

typedef struct _sl_lidar_ip_conf_t {
    sl_u8 ip_addr[4];
    sl_u8 net_mask[4];
    sl_u8 gw[4];
}__attribute__((packed)) sl_lidar_ip_conf_t;

typedef struct _sl_lidar_response_device_macaddr_info_t {
    sl_u8   macaddr[6];
} __attribute__((packed)) sl_lidar_response_device_macaddr_info_t;

typedef struct  _sl_lidar_response_desired_rot_speed_t{
    sl_u16 rpm;
    sl_u16 pwm_ref;
}__attribute__((packed)) sl_lidar_response_desired_rot_speed_t;

// Definition of the variable bit scale encoding mechanism
#define SL_LIDAR_VARBITSCALE_X2_SRC_BIT  9
#define SL_LIDAR_VARBITSCALE_X4_SRC_BIT  11
#define SL_LIDAR_VARBITSCALE_X8_SRC_BIT  12
#define SL_LIDAR_VARBITSCALE_X16_SRC_BIT 14

#define SL_LIDAR_VARBITSCALE_X2_DEST_VAL 512
#define SL_LIDAR_VARBITSCALE_X4_DEST_VAL 1280
#define SL_LIDAR_VARBITSCALE_X8_DEST_VAL 1792
#define SL_LIDAR_VARBITSCALE_X16_DEST_VAL 3328

#define SL_LIDAR_VARBITSCALE_GET_SRC_MAX_VAL_BY_BITS(_BITS_) \
    (  (((0x1<<(_BITS_)) - SL_LIDAR_VARBITSCALE_X16_DEST_VAL)<<4) + \
       ((SL_LIDAR_VARBITSCALE_X16_DEST_VAL - SL_LIDAR_VARBITSCALE_X8_DEST_VAL)<<3) + \
       ((SL_LIDAR_VARBITSCALE_X8_DEST_VAL - SL_LIDAR_VARBITSCALE_X4_DEST_VAL)<<2) + \
       ((SL_LIDAR_VARBITSCALE_X4_DEST_VAL - SL_LIDAR_VARBITSCALE_X2_DEST_VAL)<<1) + \
       SL_LIDAR_VARBITSCALE_X2_DEST_VAL - 1)


#if defined(_WIN32)
#pragma pack()
#endif
