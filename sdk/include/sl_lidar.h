/*
* Slamtec LIDAR SDK
*
* sl_lidar.h
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

#include "sl_lidar_driver.h"

#define SL_LIDAR_SDK_VERSION_MAJOR  2
#define SL_LIDAR_SDK_VERSION_MINOR  1
#define SL_LIDAR_SDK_VERSION_PATCH  0
#define SL_LIDAR_SDK_VERSION_SEQ    ((SL_LIDAR_SDK_VERSION_MAJOR << 16) | (SL_LIDAR_SDK_VERSION_MINOR << 8) | SL_LIDAR_SDK_VERSION_PATCH)


#define SL_LIDAR_SDK_VERSION_MK_STR_INDIR(x)  #x
#define SL_LIDAR_SDK_VERSION_MK_STR(x)        SL_LIDAR_SDK_VERSION_MK_STR_INDIR(x)

#define SL_LIDAR_SDK_VERSION        (SL_LIDAR_SDK_VERSION_MK_STR(SL_LIDAR_SDK_VERSION_MAJOR) "." SL_LIDAR_SDK_VERSION_MK_STR(SL_LIDAR_SDK_VERSION_MINOR) "." SL_LIDAR_SDK_VERSION_MK_STR(SL_LIDAR_SDK_VERSION_PATCH))
