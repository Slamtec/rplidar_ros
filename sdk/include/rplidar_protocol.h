/*
 *  RPLIDAR SDK
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
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
#include "sl_lidar_protocol.h"
// RP-Lidar Input Packets

#define RPLIDAR_CMD_SYNC_BYTE        SL_LIDAR_CMD_SYNC_BYTE
#define RPLIDAR_CMDFLAG_HAS_PAYLOAD  SL_LIDAR_CMDFLAG_HAS_PAYLOAD


#define RPLIDAR_ANS_SYNC_BYTE1       SL_LIDAR_ANS_SYNC_BYTE1
#define RPLIDAR_ANS_SYNC_BYTE2       SL_LIDAR_ANS_SYNC_BYTE2

#define RPLIDAR_ANS_PKTFLAG_LOOP     SL_LIDAR_ANS_PKTFLAG_LOOP

#define RPLIDAR_ANS_HEADER_SIZE_MASK        SL_LIDAR_ANS_HEADER_SIZE_MASK
#define RPLIDAR_ANS_HEADER_SUBTYPE_SHIFT    SL_LIDAR_ANS_HEADER_SUBTYPE_SHIFT

#if defined(_WIN32)
#pragma pack(1)
#endif

typedef sl_lidar_cmd_packet_t rplidar_cmd_packet_t;
typedef sl_lidar_ans_header_t rplidar_ans_header_t;

#if defined(_WIN32)
#pragma pack()
#endif
