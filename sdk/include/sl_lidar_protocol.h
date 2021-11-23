/*
* Slamtec LIDAR SDK
*
* sl_lidar_protocol.h
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

#include "sl_types.h"

#define SL_LIDAR_CMD_SYNC_BYTE              0xA5
#define SL_LIDAR_CMDFLAG_HAS_PAYLOAD        0x80

#define SL_LIDAR_ANS_SYNC_BYTE1             0xA5
#define SL_LIDAR_ANS_SYNC_BYTE2             0x5A

#define SL_LIDAR_ANS_PKTFLAG_LOOP           0x1

#define SL_LIDAR_ANS_HEADER_SIZE_MASK       0x3FFFFFFF
#define SL_LIDAR_ANS_HEADER_SUBTYPE_SHIFT   (30)

#if defined(_WIN32)
#pragma pack(1)
#endif

typedef struct sl_lidar_cmd_packet_t
{
    sl_u8 syncByte; //must be SL_LIDAR_CMD_SYNC_BYTE
    sl_u8 cmd_flag;
    sl_u8 size;
    sl_u8 data[0];
} __attribute__((packed)) sl_lidar_cmd_packet_t;


typedef struct sl_lidar_ans_header_t
{
    sl_u8  syncByte1; // must be SL_LIDAR_ANS_SYNC_BYTE1
    sl_u8  syncByte2; // must be SL_LIDAR_ANS_SYNC_BYTE2
    sl_u32 size_q30_subtype; // see _u32 size:30; _u32 subType:2;
    sl_u8  type;
} __attribute__((packed)) sl_lidar_ans_header_t;

#if defined(_WIN32)
#pragma pack()
#endif
