/*
 *  RPLIDAR SDK
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2018 Shanghai Slamtec Co., Ltd.
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

#include "sdkcommon.h"

#include "hal/abs_rxtx.h"
#include "hal/thread.h"
#include "hal/types.h"
#include "hal/assert.h"
#include "hal/locker.h"
#include "hal/socket.h"
#include "hal/event.h"
#include "rplidar_driver_impl.h"
#include "rplidar_driver_serial.h"
#include "rplidar_driver_TCP.h"

#include <algorithm>

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

namespace rp { namespace standalone{ namespace rplidar {

#define DEPRECATED_WARN(fn, replacement) do { \
        static bool __shown__ = false; \
        if (!__shown__) { \
            printDeprecationWarn(fn, replacement); \
            __shown__ = true; \
        } \
    } while (0)

    static void printDeprecationWarn(const char* fn, const char* replacement)
    {
        fprintf(stderr, "*WARN* YOU ARE USING DEPRECATED API: %s, PLEASE MOVE TO %s\n", fn, replacement);
    }

static void convert(const rplidar_response_measurement_node_t& from, rplidar_response_measurement_node_hq_t& to)
{
    to.angle_z_q14 = (((from.angle_q6_checkbit) >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) << 8) / 90;  //transfer to q14 Z-angle
    to.dist_mm_q2 = from.distance_q2;
    to.flag = (from.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);  // trasfer syncbit to HQ flag field
    to.quality = (from.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;  //remove the last two bits and then make quality from 0-63 to 0-255
}

static void convert(const rplidar_response_measurement_node_hq_t& from, rplidar_response_measurement_node_t& to)
{
    to.sync_quality = (from.flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) | ((from.quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
    to.angle_q6_checkbit = 1 | (((from.angle_z_q14 * 90) >> 8) << RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT);
    to.distance_q2 = from.dist_mm_q2 > _u16(-1) ? _u16(0) : _u16(from.dist_mm_q2);
}

// Factory Impl
RPlidarDriver * RPlidarDriver::CreateDriver(_u32 drivertype)
{
    switch (drivertype) {
    case DRIVER_TYPE_SERIALPORT:
        return new RPlidarDriverSerial();
    case DRIVER_TYPE_TCP:
         return new RPlidarDriverTCP();
    default:
        return NULL;
    }
}


void RPlidarDriver::DisposeDriver(RPlidarDriver * drv)
{
    delete drv;
}


RPlidarDriverImplCommon::RPlidarDriverImplCommon()
    : _isConnected(false)
    , _isScanning(false)
    , _isSupportingMotorCtrl(false)
{
    _cached_scan_node_hq_count = 0;
    _cached_scan_node_hq_count_for_interval_retrieve = 0;
    _cached_sampleduration_std = LEGACY_SAMPLE_DURATION;
    _cached_sampleduration_express = LEGACY_SAMPLE_DURATION;
}

bool RPlidarDriverImplCommon::isConnected()
{
    return _isConnected;
}


u_result RPlidarDriverImplCommon::reset(_u32 timeout)
{
    u_result ans;

    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_RESET))) {
            return ans;
        }
    }
    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::_waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout)
{
    int  recvPos = 0;
    _u32 startTs = getms();
    _u8  recvBuffer[sizeof(rplidar_ans_header_t)];
    _u8  *headerBuffer = reinterpret_cast<_u8 *>(header);
    _u32 waitTime;

    while ((waitTime=getms() - startTs) <= timeout) {
        size_t remainSize = sizeof(rplidar_ans_header_t) - recvPos;
        size_t recvSize;
        
        bool ans = _chanDev->waitfordata(remainSize, timeout - waitTime, &recvSize);
        if(!ans) return RESULT_OPERATION_TIMEOUT;
        
        if(recvSize > remainSize) recvSize = remainSize;
        
        recvSize = _chanDev->recvdata(recvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos) {
            _u8 currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0:
                if (currentByte != RPLIDAR_ANS_SYNC_BYTE1) {
                   continue;
                }
                
                break;
            case 1:
                if (currentByte != RPLIDAR_ANS_SYNC_BYTE2) {
                    recvPos = 0;
                    continue;
                }
                break;
            }
            headerBuffer[recvPos++] = currentByte;

            if (recvPos == sizeof(rplidar_ans_header_t)) {
                return RESULT_OK;
            }
        }
    }

    return RESULT_OPERATION_TIMEOUT;
}



u_result RPlidarDriverImplCommon::getHealth(rplidar_response_device_health_t & healthinfo, _u32 timeout)
{
    u_result  ans;
    
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    
    _disableDataGrabbing();

    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVHEALTH) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if ( header_size < sizeof(rplidar_response_device_health_t)) {
            return RESULT_INVALID_DATA;
        }

         if (!_chanDev->waitfordata(header_size, timeout)) {
            return RESULT_OPERATION_TIMEOUT;
        }
        _chanDev->recvdata(reinterpret_cast<_u8 *>(&healthinfo), sizeof(healthinfo));
    }
    return RESULT_OK;
}



u_result RPlidarDriverImplCommon::getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout)
{
    u_result  ans;
    
    if (!isConnected()) return RESULT_OPERATION_FAIL;

    _disableDataGrabbing();

    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVINFO) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(rplidar_response_device_info_t)) {
            return RESULT_INVALID_DATA;
        }

        if (!_chanDev->waitfordata(header_size, timeout)) {
            return RESULT_OPERATION_TIMEOUT;
        }
        _chanDev->recvdata(reinterpret_cast<_u8 *>(&info), sizeof(info));
    }
    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::getFrequency(bool inExpressMode, size_t count, float & frequency, bool & is4kmode)
{
    DEPRECATED_WARN("getFrequency(bool,size_t,float&,bool&)", "getFrequency(const RplidarScanMode&,size_t,float&)");

    _u16 sample_duration = inExpressMode?_cached_sampleduration_express:_cached_sampleduration_std;
    frequency = 1000000.0f/(count * sample_duration);

    if (sample_duration <= 277) {
        is4kmode = true;
    } else {
        is4kmode = false;
    }

    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::getFrequency(const RplidarScanMode& scanMode, size_t count, float & frequency)
{
    float sample_duration = scanMode.us_per_sample;
    frequency = 1000000.0f / (count * sample_duration);
    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::_waitNode(rplidar_response_measurement_node_t * node, _u32 timeout)
{
    int  recvPos = 0;
    _u32 startTs = getms();
    _u8  recvBuffer[sizeof(rplidar_response_measurement_node_t)];
    _u8 *nodeBuffer = (_u8*)node;
    _u32 waitTime;

   while ((waitTime=getms() - startTs) <= timeout) {
        size_t remainSize = sizeof(rplidar_response_measurement_node_t) - recvPos;
        size_t recvSize;

        bool ans = _chanDev->waitfordata(remainSize, timeout-waitTime, &recvSize);
        if(!ans) return RESULT_OPERATION_FAIL;

        if (recvSize > remainSize) recvSize = remainSize;
        
        recvSize = _chanDev->recvdata(recvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos) {
            _u8 currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0: // expect the sync bit and its reverse in this byte
                {
                    _u8 tmp = (currentByte>>1);
                    if ( (tmp ^ currentByte) & 0x1 ) {
                        // pass
                    } else {
                        continue;
                    }

                }
                break;
            case 1: // expect the highest bit to be 1
                {
                    if (currentByte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
                        // pass
                    } else {
                        recvPos = 0;
                        continue;
                    }
                }
                break;
            }
            nodeBuffer[recvPos++] = currentByte;

            if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
                return RESULT_OK;
            }
        }
    }

    return RESULT_OPERATION_TIMEOUT;
}

u_result RPlidarDriverImplCommon::_waitScanData(rplidar_response_measurement_node_t * nodebuffer, size_t & count, _u32 timeout)
{
    if (!_isConnected) {
        count = 0;
        return RESULT_OPERATION_FAIL;
    }

    size_t   recvNodeCount =  0;
    _u32     startTs = getms();
    _u32     waitTime;
    u_result ans;

    while ((waitTime = getms() - startTs) <= timeout && recvNodeCount < count) {
        rplidar_response_measurement_node_t node;
        if (IS_FAIL(ans = _waitNode(&node, timeout - waitTime))) {
            return ans;
        }
        
        nodebuffer[recvNodeCount++] = node;

        if (recvNodeCount == count) return RESULT_OK;
    }
    count = recvNodeCount;
    return RESULT_OPERATION_TIMEOUT;
}


u_result RPlidarDriverImplCommon::_waitCapsuledNode(rplidar_response_capsule_measurement_nodes_t & node, _u32 timeout)
{
    int  recvPos = 0;
    _u32 startTs = getms();
    _u8  recvBuffer[sizeof(rplidar_response_capsule_measurement_nodes_t)];
    _u8 *nodeBuffer = (_u8*)&node;
    _u32 waitTime;


   while ((waitTime=getms() - startTs) <= timeout) {
        size_t remainSize = sizeof(rplidar_response_capsule_measurement_nodes_t) - recvPos;
        size_t recvSize;

        bool ans = _chanDev->waitfordata(remainSize, timeout-waitTime, &recvSize);
        if(!ans)
        {
            return RESULT_OPERATION_TIMEOUT;
        }
        if (recvSize > remainSize) recvSize = remainSize;
        
        recvSize = _chanDev->recvdata(recvBuffer, recvSize);
        
        for (size_t pos = 0; pos < recvSize; ++pos) {
            _u8 currentByte = recvBuffer[pos];

            switch (recvPos) {
            case 0: // expect the sync bit 1
                {
                    _u8 tmp = (currentByte>>4);
                    if ( tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1 ) {
                        // pass
                    } else {
                        _is_previous_capsuledataRdy = false;
                        continue;
                    }

                }
                break;
            case 1: // expect the sync bit 2
                {
                    _u8 tmp = (currentByte>>4);
                    if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                        // pass
                    } else {
                        recvPos = 0;
                        _is_previous_capsuledataRdy = false;
                        continue;
                    }
                }
                break;
            }
            nodeBuffer[recvPos++] = currentByte;
            if (recvPos == sizeof(rplidar_response_capsule_measurement_nodes_t)) {
                // calc the checksum ...
                _u8 checksum = 0;
                _u8 recvChecksum = ((node.s_checksum_1 & 0xF) | (node.s_checksum_2<<4));
                for (size_t cpos = offsetof(rplidar_response_capsule_measurement_nodes_t, start_angle_sync_q6);
                    cpos < sizeof(rplidar_response_capsule_measurement_nodes_t); ++cpos)
                {
                    checksum ^= nodeBuffer[cpos];
                }
                if (recvChecksum == checksum)
                {
                    // only consider vaild if the checksum matches...
                    if (node.start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT) 
                    {
                        // this is the first capsule frame in logic, discard the previous cached data...
                        _is_previous_capsuledataRdy = false;
                        return RESULT_OK;
                    }
                    return RESULT_OK;
                }
                _is_previous_capsuledataRdy = false;
                return RESULT_INVALID_DATA;
            }
        }
    }
    _is_previous_capsuledataRdy = false;
    return RESULT_OPERATION_TIMEOUT;
}

u_result RPlidarDriverImplCommon::_waitUltraCapsuledNode(rplidar_response_ultra_capsule_measurement_nodes_t & node, _u32 timeout)
{
    if (!_isConnected) {
        return RESULT_OPERATION_FAIL;
    }
    
    int  recvPos = 0;
    _u32 startTs = getms();
    _u8  recvBuffer[sizeof(rplidar_response_ultra_capsule_measurement_nodes_t)];
    _u8 *nodeBuffer = (_u8*)&node;
    _u32 waitTime;
    
    while ((waitTime=getms() - startTs) <= timeout) {
        size_t remainSize = sizeof(rplidar_response_ultra_capsule_measurement_nodes_t) - recvPos;
        size_t recvSize;

        bool ans = _chanDev->waitfordata(remainSize, timeout-waitTime, &recvSize);
        if(!ans)
        {
            return RESULT_OPERATION_TIMEOUT;
        }
        if (recvSize > remainSize) recvSize = remainSize;
        
        recvSize = _chanDev->recvdata(recvBuffer, recvSize);
        
        for (size_t pos = 0; pos < recvSize; ++pos) {
            _u8 currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0: // expect the sync bit 1
                {
                    _u8 tmp = (currentByte>>4);
                    if ( tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1 ) {
                    // pass
                    }
                    else {
                        _is_previous_capsuledataRdy = false;
                        continue;
                    }
                }    
            break;
            case 1: // expect the sync bit 2
            {
                _u8 tmp = (currentByte>>4);
                if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                    // pass
                }
                else {
                    recvPos = 0;
                    _is_previous_capsuledataRdy = false;
                    continue;
                }
            }
            break;
            }
            nodeBuffer[recvPos++] = currentByte;
            if (recvPos == sizeof(rplidar_response_ultra_capsule_measurement_nodes_t)) {
                // calc the checksum ...
                _u8 checksum = 0;
                _u8 recvChecksum = ((node.s_checksum_1 & 0xF) | (node.s_checksum_2 << 4));
                
                for (size_t cpos = offsetof(rplidar_response_ultra_capsule_measurement_nodes_t, start_angle_sync_q6);
                cpos < sizeof(rplidar_response_ultra_capsule_measurement_nodes_t); ++cpos)
                {
                    checksum ^= nodeBuffer[cpos];
                }
                
                if (recvChecksum == checksum)
                {
                    // only consider vaild if the checksum matches...
                    if (node.start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT) 
                    {
                        // this is the first capsule frame in logic, discard the previous cached data...
                        _is_previous_capsuledataRdy = false;
                        return RESULT_OK;
                    }
                    return RESULT_OK;
                }
                _is_previous_capsuledataRdy = false;
                return RESULT_INVALID_DATA;
            }
        }
    }
    _is_previous_capsuledataRdy = false;
    return RESULT_OPERATION_TIMEOUT;
}

u_result RPlidarDriverImplCommon::_cacheScanData()
{
    rplidar_response_measurement_node_t      local_buf[128];
    size_t                                   count = 128;
    rplidar_response_measurement_node_hq_t   local_scan[MAX_SCAN_NODES];
    size_t                                   scan_count = 0;
    u_result                                 ans;
    memset(local_scan, 0, sizeof(local_scan));

    _waitScanData(local_buf, count); // // always discard the first data since it may be incomplete

    while(_isScanning)
    {
        if (IS_FAIL(ans=_waitScanData(local_buf, count))) {
            if (ans != RESULT_OPERATION_TIMEOUT) {
                _isScanning = false;
                return RESULT_OPERATION_FAIL;
            }
        }
        
        for (size_t pos = 0; pos < count; ++pos)
        {
            if (local_buf[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)
            {
                // only publish the data when it contains a full 360 degree scan 
                
                if ((local_scan[0].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                    _lock.lock();
                    memcpy(_cached_scan_node_hq_buf, local_scan, scan_count*sizeof(rplidar_response_measurement_node_hq_t));
                    _cached_scan_node_hq_count = scan_count;
                    _dataEvt.set();
                    _lock.unlock();
                }
                scan_count = 0;
            }

            rplidar_response_measurement_node_hq_t nodeHq;
            convert(local_buf[pos], nodeHq);
            local_scan[scan_count++] = nodeHq;
            if (scan_count == _countof(local_scan)) scan_count-=1; // prevent overflow

            //for interval retrieve
            {
                rp::hal::AutoLocker l(_lock);
                _cached_scan_node_hq_buf_for_interval_retrieve[_cached_scan_node_hq_count_for_interval_retrieve++] = nodeHq;
                if(_cached_scan_node_hq_count_for_interval_retrieve == _countof(_cached_scan_node_hq_buf_for_interval_retrieve)) _cached_scan_node_hq_count_for_interval_retrieve-=1; // prevent overflow
            }
        }
    }
    _isScanning = false;
    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::startScanNormal(bool force,  _u32 timeout)
{
    u_result ans;
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    if (_isScanning) return RESULT_ALREADY_DONE;

    stop(); //force the previous operation to stop

    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(force?RPLIDAR_CMD_FORCE_SCAN:RPLIDAR_CMD_SCAN))) {
            return ans;
        }

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_MEASUREMENT) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(rplidar_response_measurement_node_t)) {
            return RESULT_INVALID_DATA;
        }

        _isScanning = true;
        _cachethread = CLASS_THREAD(RPlidarDriverImplCommon, _cacheScanData);
        if (_cachethread.getHandle() == 0) {
            return RESULT_OPERATION_FAIL;
        }
    }
    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::checkExpressScanSupported(bool & support, _u32 timeout)
{
    DEPRECATED_WARN("checkExpressScanSupported(bool&,_u32)", "getAllSupportedScanModes()");

    rplidar_response_device_info_t devinfo;

    support = false;
    u_result ans = getDeviceInfo(devinfo, timeout);

    if (IS_FAIL(ans)) return ans;

    if (devinfo.firmware_version >= ((0x1<<8) | 17)) {
        support = true;
        rplidar_response_sample_rate_t sample_rate;
        getSampleDuration_uS(sample_rate);
        _cached_sampleduration_express = sample_rate.express_sample_duration_us;
        _cached_sampleduration_std = sample_rate.std_sample_duration_us;
    }

    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::_cacheCapsuledScanData()
{
    rplidar_response_capsule_measurement_nodes_t    capsule_node;
    rplidar_response_measurement_node_hq_t   local_buf[128];
    size_t                                   count = 128;
    rplidar_response_measurement_node_hq_t   local_scan[MAX_SCAN_NODES];
    size_t                                   scan_count = 0;
    u_result                                 ans;
    memset(local_scan, 0, sizeof(local_scan));

    _waitCapsuledNode(capsule_node); // // always discard the first data since it may be incomplete

    
    

    while(_isScanning)
    {
        if (IS_FAIL(ans=_waitCapsuledNode(capsule_node))) {
            if (ans != RESULT_OPERATION_TIMEOUT && ans != RESULT_INVALID_DATA) {
                _isScanning = false;
                return RESULT_OPERATION_FAIL;
            } else {
                // current data is invalid, do not use it.
                continue;
            }
        }
        
        _capsuleToNormal(capsule_node, local_buf, count);

        for (size_t pos = 0; pos < count; ++pos)
        {
            if (local_buf[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)
            {
                // only publish the data when it contains a full 360 degree scan 
                
                if ((local_scan[0].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                    _lock.lock();
                    memcpy(_cached_scan_node_hq_buf, local_scan, scan_count*sizeof(rplidar_response_measurement_node_hq_t));
                    _cached_scan_node_hq_count = scan_count;
                    _dataEvt.set();
                    _lock.unlock();
                }
                scan_count = 0;
            }
            local_scan[scan_count++] = local_buf[pos];
            if (scan_count == _countof(local_scan)) scan_count-=1; // prevent overflow

            //for interval retrieve
            {
                rp::hal::AutoLocker l(_lock);
                _cached_scan_node_hq_buf_for_interval_retrieve[_cached_scan_node_hq_count_for_interval_retrieve++] = local_buf[pos];
                if(_cached_scan_node_hq_count_for_interval_retrieve == _countof(_cached_scan_node_hq_buf_for_interval_retrieve)) _cached_scan_node_hq_count_for_interval_retrieve-=1; // prevent overflow
            }
        }
    }
    _isScanning = false;

    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::_cacheUltraCapsuledScanData()
{
    rplidar_response_ultra_capsule_measurement_nodes_t    ultra_capsule_node;
    rplidar_response_measurement_node_hq_t   local_buf[128];
    size_t                                   count = 128;
    rplidar_response_measurement_node_hq_t   local_scan[MAX_SCAN_NODES];
    size_t                                   scan_count = 0;
    u_result                                 ans;
    memset(local_scan, 0, sizeof(local_scan));

    _waitUltraCapsuledNode(ultra_capsule_node);
    
    while(_isScanning)
    {
        if (IS_FAIL(ans=_waitUltraCapsuledNode(ultra_capsule_node))) {
            if (ans != RESULT_OPERATION_TIMEOUT && ans != RESULT_INVALID_DATA) {
                _isScanning = false;
                return RESULT_OPERATION_FAIL;
            } else {
                // current data is invalid, do not use it.
                continue;
            }
        }
        
        _ultraCapsuleToNormal(ultra_capsule_node, local_buf, count);
        
        for (size_t pos = 0; pos < count; ++pos)
        {
            if (local_buf[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)
            {
                // only publish the data when it contains a full 360 degree scan 
                
                if ((local_scan[0].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                    _lock.lock();
                    memcpy(_cached_scan_node_hq_buf, local_scan, scan_count*sizeof(rplidar_response_measurement_node_hq_t));
                    _cached_scan_node_hq_count = scan_count;
                    _dataEvt.set();
                    _lock.unlock();
                }
                scan_count = 0;
            }
            local_scan[scan_count++] = local_buf[pos];
            if (scan_count == _countof(local_scan)) scan_count-=1; // prevent overflow

            //for interval retrieve
            {
                rp::hal::AutoLocker l(_lock);
                _cached_scan_node_hq_buf_for_interval_retrieve[_cached_scan_node_hq_count_for_interval_retrieve++] = local_buf[pos];
                if(_cached_scan_node_hq_count_for_interval_retrieve == _countof(_cached_scan_node_hq_buf_for_interval_retrieve)) _cached_scan_node_hq_count_for_interval_retrieve-=1; // prevent overflow
            }
        }
    }
    
    _isScanning = false;

    return RESULT_OK;
}

void     RPlidarDriverImplCommon::_capsuleToNormal(const rplidar_response_capsule_measurement_nodes_t & capsule, rplidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
{
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF)<< 2);
        int prevStartAngle_q8 = ((_cached_previous_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8) - (prevStartAngle_q8);
        if (prevStartAngle_q8 >  currentStartAngle_q8) {
            diffAngle_q8 += (360<<8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3);
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_capsuledata.cabins); ++pos)
        {
            int dist_q2[2];
            int angle_q6[2];
            int syncBit[2];

            dist_q2[0] = (_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0xFFFC);
            dist_q2[1] = (_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0xFFFC);

            int angle_offset1_q3 = ( (_cached_previous_capsuledata.cabins[pos].offset_angles_q3 & 0xF) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0x3)<<4));
            int angle_offset2_q3 = ( (_cached_previous_capsuledata.cabins[pos].offset_angles_q3 >> 4) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0x3)<<4));

            angle_q6[0] = ((currentAngle_raw_q16 - (angle_offset1_q3<<13))>>10);
            syncBit[0] =  (( (currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 )?1:0;
            currentAngle_raw_q16 += angleInc_q16;


            angle_q6[1] = ((currentAngle_raw_q16 - (angle_offset2_q3<<13))>>10);
            syncBit[1] =  (( (currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 )?1:0;
            currentAngle_raw_q16 += angleInc_q16;

            for (int cpos = 0; cpos < 2; ++cpos) {

                if (angle_q6[cpos] < 0) angle_q6[cpos] += (360<<6);
                if (angle_q6[cpos] >= (360<<6)) angle_q6[cpos] -= (360<<6);

                rplidar_response_measurement_node_hq_t node;

                node.angle_z_q14 = _u16((angle_q6[cpos] << 8) / 90);
                node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                node.quality = dist_q2[cpos] ? (0x2f << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                node.dist_mm_q2 = dist_q2[cpos];

                nodebuffer[nodeCount++] = node;
             }

        }
    }

    _cached_previous_capsuledata = capsule;
    _is_previous_capsuledataRdy = true;
}

//*******************************************HQ support********************************
u_result RPlidarDriverImplCommon::_cacheHqScanData()
{
    rplidar_response_hq_capsule_measurement_nodes_t    hq_node;
    rplidar_response_measurement_node_hq_t   local_buf[128];
    size_t                                   count = 128;
    rplidar_response_measurement_node_hq_t   local_scan[MAX_SCAN_NODES];
    size_t                                   scan_count = 0;
    u_result                                 ans;
    memset(local_scan, 0, sizeof(local_scan));
    _waitHqNode(hq_node);
    while (_isScanning) {
        if (IS_FAIL(ans = _waitHqNode(hq_node))) {
            if (ans != RESULT_OPERATION_TIMEOUT && ans != RESULT_INVALID_DATA) {
                _isScanning = false;
                return RESULT_OPERATION_FAIL;
            }
            else {
				// current data is invalid, do not use it.
				continue;
            }
        }

        _HqToNormal(hq_node, local_buf, count);
        for (size_t pos = 0; pos < count; ++pos)
        {
            if (local_buf[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)
            {
				// only publish the data when it contains a full 360 degree scan 
                if ((local_scan[0].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                    _lock.lock();
                    memcpy(_cached_scan_node_hq_buf, local_scan, scan_count * sizeof(rplidar_response_measurement_node_hq_t));
                    _cached_scan_node_hq_count = scan_count;
                    _dataEvt.set();
                    _lock.unlock();
                }
                scan_count = 0;
            }
            local_scan[scan_count++] = local_buf[pos];
            if (scan_count == _countof(local_scan)) scan_count -= 1; // prevent overflow
																	 //for interval retrieve
            {
                rp::hal::AutoLocker l(_lock);
                _cached_scan_node_hq_buf_for_interval_retrieve[_cached_scan_node_hq_count_for_interval_retrieve++] = local_buf[pos];
                if (_cached_scan_node_hq_count_for_interval_retrieve == _countof(_cached_scan_node_hq_buf_for_interval_retrieve)) _cached_scan_node_hq_count_for_interval_retrieve -= 1; // prevent overflow
            }
        }

    }
    return RESULT_OK;
}

//CRC calculate
static _u32 table[256];//crc32_table

//reflect
static _u32 _bitrev(_u32 input, _u16 bw)
{
    _u16 i;
    _u32 var;
    var = 0;
    for (i = 0; i<bw; i++){
        if (input & 0x01)
        {
            var |= 1 << (bw - 1 - i);
        }
        input >>= 1;
    }
    return var;
}

// crc32_table init
static void _crc32_init(_u32 poly)
{
    _u16 i;
    _u16 j;
    _u32 c;
    
    poly = _bitrev(poly, 32);
    for (i = 0; i<256; i++){
        c = i;
        for (j = 0; j<8; j++){
            if (c & 1)
                c = poly ^ (c >> 1);
            else
                c = c >> 1;
        }
        table[i] = c;
    }
}

static _u32 _crc32cal(_u32 crc, void* input, _u16 len)
{
    _u16 i;
    _u8 index;
    _u8* pch;
    pch = (unsigned char*)input;
    _u8 leftBytes = 4 - len & 0x3;

    for (i = 0; i<len; i++){
        index = (unsigned char)(crc^*pch);
        crc = (crc >> 8) ^ table[index];
        pch++;
    }

    for (i = 0; i < leftBytes; i++) {//zero padding
        index = (unsigned char)(crc^0);
        crc = (crc >> 8) ^ table[index];
    }
    return crc^0xffffffff;
}

//crc32cal
static u_result _crc32(_u8 *ptr, _u32 len) {
	static _u8 tmp;
	if (tmp != 1) {
		_crc32_init(0x4C11DB7);
		tmp = 1;
	}
	
	return _crc32cal(0xFFFFFFFF, ptr,len);
}

u_result RPlidarDriverImplCommon::_waitHqNode(rplidar_response_hq_capsule_measurement_nodes_t & node, _u32 timeout)
{
    if (!_isConnected) {
        return RESULT_OPERATION_FAIL;
    }

    int  recvPos = 0;
    _u32 startTs = getms();
    _u8  recvBuffer[sizeof(rplidar_response_hq_capsule_measurement_nodes_t)];
    _u8 *nodeBuffer = (_u8*)&node;
    _u32 waitTime;
    
    while ((waitTime=getms() - startTs) <= timeout) {
        size_t remainSize = sizeof(rplidar_response_hq_capsule_measurement_nodes_t) - recvPos;
        size_t recvSize;
        
        bool ans = _chanDev->waitfordata(remainSize, timeout-waitTime, &recvSize);
        if(!ans)
        {
            return RESULT_OPERATION_TIMEOUT;
        }
        if (recvSize > remainSize) recvSize = remainSize;
        
        recvSize = _chanDev->recvdata(recvBuffer, recvSize);
    
        for (size_t pos = 0; pos < recvSize; ++pos) {
            _u8 currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0: // expect the sync byte
                {
                    _u8 tmp = (currentByte);
                    if ( tmp == RPLIDAR_RESP_MEASUREMENT_HQ_SYNC ) {
                    // pass
                    }
                    else {
                        recvPos = 0;
                        _is_previous_HqdataRdy = false;
                        continue;
                    }
                }
           break;
           case sizeof(rplidar_response_hq_capsule_measurement_nodes_t) - 1 - 4: 
            {

            }
           break;
           case sizeof(rplidar_response_hq_capsule_measurement_nodes_t) - 1: 
            {				

            }
           break;
           }
           nodeBuffer[recvPos++] = currentByte;
           if (recvPos == sizeof(rplidar_response_hq_capsule_measurement_nodes_t)) {
                _u32 crcCalc2 = _crc32(nodeBuffer, sizeof(rplidar_response_hq_capsule_measurement_nodes_t) - 4);

                if(crcCalc2 == node.crc32){
                    _is_previous_HqdataRdy = true;
                    return RESULT_OK;
                }
                else {
                    _is_previous_HqdataRdy = false;
                    return RESULT_INVALID_DATA;
                }

            }
        }
    }
    _is_previous_HqdataRdy = false;
    return RESULT_OPERATION_TIMEOUT;
}

void RPlidarDriverImplCommon::_HqToNormal(const rplidar_response_hq_capsule_measurement_nodes_t & node_hq, rplidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount) 
{
    nodeCount = 0;
    if (_is_previous_HqdataRdy) {
        for (size_t pos = 0; pos < _countof(_cached_previous_Hqdata.node_hq); ++pos)
        {
            nodebuffer[nodeCount++] = node_hq.node_hq[pos];
        }	
    }
    _cached_previous_Hqdata = node_hq;
    _is_previous_HqdataRdy = true;

}
//*******************************************HQ support********************************//

static _u32 _varbitscale_decode(_u32 scaled, _u32 & scaleLevel)
{
    static const _u32 VBS_SCALED_BASE[] = {
        RPLIDAR_VARBITSCALE_X16_DEST_VAL,
        RPLIDAR_VARBITSCALE_X8_DEST_VAL,
        RPLIDAR_VARBITSCALE_X4_DEST_VAL,
        RPLIDAR_VARBITSCALE_X2_DEST_VAL,
        0,
    };

    static const _u32 VBS_SCALED_LVL[] = {
        4,
        3,
        2,
        1,
        0,
    };

    static const _u32 VBS_TARGET_BASE[] = {
        (0x1 << RPLIDAR_VARBITSCALE_X16_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X8_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X4_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X2_SRC_BIT),
        0,
    };

    for (size_t i = 0; i < _countof(VBS_SCALED_BASE); ++i)
    {
        int remain = ((int)scaled - (int)VBS_SCALED_BASE[i]);
        if (remain >= 0) {
            scaleLevel = VBS_SCALED_LVL[i];
            return VBS_TARGET_BASE[i] + (remain << scaleLevel);
        }
    }
    return 0;
}

void RPlidarDriverImplCommon::_ultraCapsuleToNormal(const rplidar_response_ultra_capsule_measurement_nodes_t & capsule, rplidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
{
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_ultracapsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 >  currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3) / 3;
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_ultracapsuledata.ultra_cabins); ++pos)
        {
            int dist_q2[3];
            int angle_q6[3];
            int syncBit[3];


            _u32 combined_x3 = _cached_previous_ultracapsuledata.ultra_cabins[pos].combined_x3;

            // unpack ...
            int dist_major = (combined_x3 & 0xFFF);

            // signed partical integer, using the magic shift here
            // DO NOT TOUCH

            int dist_predict1 = (((int)(combined_x3 << 10)) >> 22);
            int dist_predict2 = (((int)combined_x3) >> 22);

            int dist_major2;

            _u32 scalelvl1, scalelvl2;

            // prefetch next ...
            if (pos == _countof(_cached_previous_ultracapsuledata.ultra_cabins) - 1)
            {
                dist_major2 = (capsule.ultra_cabins[0].combined_x3 & 0xFFF);
            }
            else {
                dist_major2 = (_cached_previous_ultracapsuledata.ultra_cabins[pos + 1].combined_x3 & 0xFFF);
            }

            // decode with the var bit scale ...
            dist_major = _varbitscale_decode(dist_major, scalelvl1);
            dist_major2 = _varbitscale_decode(dist_major2, scalelvl2);


            int dist_base1 = dist_major;
            int dist_base2 = dist_major2;

            if ((!dist_major) && dist_major2) {
                dist_base1 = dist_major2;
                scalelvl1 = scalelvl2;
            }

           
            dist_q2[0] = (dist_major << 2);
            if ((dist_predict1 == 0xFFFFFE00) || (dist_predict1 == 0x1FF)) {
                dist_q2[1] = 0;
            } else {
                dist_predict1 = (dist_predict1 << scalelvl1);
                dist_q2[1] = (dist_predict1 + dist_base1) << 2;

            }

            if ((dist_predict2 == 0xFFFFFE00) || (dist_predict2 == 0x1FF)) {
                dist_q2[2] = 0;
            } else {
                dist_predict2 = (dist_predict2 << scalelvl2);
                dist_q2[2] = (dist_predict2 + dist_base2) << 2;
            }
           

            for (int cpos = 0; cpos < 3; ++cpos)
            {

                syncBit[cpos] = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;

                int offsetAngleMean_q16 = (int)(7.5 * 3.1415926535 * (1 << 16) / 180.0);

                if (dist_q2[cpos] >= (50 * 4))
                {
                    const int k1 = 98361;
                    const int k2 = int(k1 / dist_q2[cpos]);

                    offsetAngleMean_q16 = (int)(8 * 3.1415926535 * (1 << 16) / 180) - (k2 << 6) - (k2 * k2 * k2) / 98304;
                }

                angle_q6[cpos] = ((currentAngle_raw_q16 - int(offsetAngleMean_q16 * 180 / 3.14159265)) >> 10);
                currentAngle_raw_q16 += angleInc_q16;

                if (angle_q6[cpos] < 0) angle_q6[cpos] += (360 << 6);
                if (angle_q6[cpos] >= (360 << 6)) angle_q6[cpos] -= (360 << 6);

                rplidar_response_measurement_node_hq_t node;

                node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                node.quality = dist_q2[cpos] ? (0x2F << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                node.angle_z_q14 = _u16((angle_q6[cpos] << 8) / 90);
                node.dist_mm_q2 = dist_q2[cpos];

                nodebuffer[nodeCount++] = node;
            }

        }
    }

    _cached_previous_ultracapsuledata = capsule;
    _is_previous_capsuledataRdy = true;
}

u_result RPlidarDriverImplCommon::checkSupportConfigCommands(bool& outSupport, _u32 timeoutInMs)
{
    u_result ans;

    rplidar_response_device_info_t devinfo;
    ans = getDeviceInfo(devinfo, timeoutInMs);
    if (IS_FAIL(ans)) return ans;

    // if lidar firmware >= 1.24
    if (devinfo.firmware_version >= ((0x1 << 8) | 24)) {
        outSupport = true;
    }
    return ans;
}

u_result RPlidarDriverImplCommon::getLidarConf(_u32 type, std::vector<_u8> &outputBuf, const std::vector<_u8> &reserve, _u32 timeout)
{
    rplidar_payload_get_scan_conf_t query;
    memset(&query, 0, sizeof(query));
    query.type = type;
    int sizeVec = reserve.size();

    int maxLen = sizeof(query.reserved) / sizeof(query.reserved[0]);
    if (sizeVec > maxLen) sizeVec = maxLen;

    if (sizeVec > 0)
        memcpy(query.reserved, &reserve[0], reserve.size());

    u_result ans;
    {
        rp::hal::AutoLocker l(_lock);
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_LIDAR_CONF, &query, sizeof(query)))) {
            return ans;
        }

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_GET_LIDAR_CONF) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(type)) {
            return RESULT_INVALID_DATA;
        }

        if (!_chanDev->waitfordata(header_size, timeout)) {
            return RESULT_OPERATION_TIMEOUT;
        }

        std::vector<_u8> dataBuf;
        dataBuf.resize(header_size);
        _chanDev->recvdata(reinterpret_cast<_u8 *>(&dataBuf[0]), header_size);

        //check if returned type is same as asked type
        _u32 replyType = -1;
        memcpy(&replyType, &dataBuf[0], sizeof(type));
        if (replyType != type) {
            return RESULT_INVALID_DATA;
        }

        //copy all the payload into &outputBuf
        int payLoadLen = header_size - sizeof(type);

        //do consistency check
        if (payLoadLen <= 0) {
            return RESULT_INVALID_DATA;
        }
        //copy all payLoadLen bytes to outputBuf
        outputBuf.resize(payLoadLen);
        memcpy(&outputBuf[0], &dataBuf[0] + sizeof(type), payLoadLen);
    }
    return ans;
}

u_result RPlidarDriverImplCommon::getTypicalScanMode(_u16& outMode, _u32 timeoutInMs)
{
    u_result ans;
    std::vector<_u8> answer;
    bool lidarSupportConfigCmds = false;
    ans = checkSupportConfigCommands(lidarSupportConfigCmds);
    if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

    if (lidarSupportConfigCmds)
    {
        ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_TYPICAL, answer, std::vector<_u8>(), timeoutInMs);
        if (IS_FAIL(ans)) {
            return ans;
        }
        if (answer.size() < sizeof(_u16)) {
            return RESULT_INVALID_DATA;
        }

        const _u16 *p_answer = reinterpret_cast<const _u16*>(&answer[0]);
        outMode = *p_answer;
        return ans;
    }
    //old version of triangle lidar
    else
    {
        outMode = RPLIDAR_CONF_SCAN_COMMAND_EXPRESS;
        return ans;
    }
    return ans;
}

u_result RPlidarDriverImplCommon::getLidarSampleDuration(float& sampleDurationRes, _u16 scanModeID, _u32 timeoutInMs)
{
    u_result ans;
    std::vector<_u8> reserve(2);
    memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    std::vector<_u8> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE, answer, reserve, timeoutInMs);
    if (IS_FAIL(ans))
    {
        return ans;
    }
    if (answer.size() < sizeof(_u32))
    {
        return RESULT_INVALID_DATA;
    }
    const _u32 *result = reinterpret_cast<const _u32*>(&answer[0]);
    sampleDurationRes = (float)(*result >> 8);
    return ans;
}

u_result RPlidarDriverImplCommon::getMaxDistance(float &maxDistance, _u16 scanModeID, _u32 timeoutInMs)
{
    u_result ans;
    std::vector<_u8> reserve(2);
    memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    std::vector<_u8> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE, answer, reserve, timeoutInMs);
    if (IS_FAIL(ans))
    {
        return ans;
    }
    if (answer.size() < sizeof(_u32))
    {
        return RESULT_INVALID_DATA;
    }
    const _u32 *result = reinterpret_cast<const _u32*>(&answer[0]);
    maxDistance = (float)(*result >> 8);
    return ans;
}

u_result RPlidarDriverImplCommon::getScanModeAnsType(_u8 &ansType, _u16 scanModeID, _u32 timeoutInMs)
{
    u_result ans;
    std::vector<_u8> reserve(2);
    memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    std::vector<_u8> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_ANS_TYPE, answer, reserve, timeoutInMs);
    if (IS_FAIL(ans))
    {
        return ans;
    }
    if (answer.size() < sizeof(_u8))
    {
        return RESULT_INVALID_DATA;
    }
    const _u8 *result = reinterpret_cast<const _u8*>(&answer[0]);
    ansType = *result;
    return ans;
}

u_result RPlidarDriverImplCommon::getScanModeName(char* modeName, _u16 scanModeID, _u32 timeoutInMs)
{
    u_result ans;
    std::vector<_u8> reserve(2);
    memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    std::vector<_u8> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_NAME, answer, reserve, timeoutInMs);
    if (IS_FAIL(ans))
    {
        return ans;
    }
    int len = answer.size();
    if (0 == len) return RESULT_INVALID_DATA;
    memcpy(modeName, &answer[0], len);
    return ans;
}

u_result RPlidarDriverImplCommon::getAllSupportedScanModes(std::vector<RplidarScanMode>& outModes, _u32 timeoutInMs)
{
    u_result ans;
    bool confProtocolSupported = false;
    ans = checkSupportConfigCommands(confProtocolSupported);
    if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

    if (confProtocolSupported)
    {
        // 1. get scan mode count
        _u16 modeCount;
        ans = getScanModeCount(modeCount);
        if (IS_FAIL(ans))
        {
            return RESULT_INVALID_DATA;
        }
        // 2. for loop to get all fields of each scan mode
        for (_u16 i = 0; i < modeCount; i++)
        {
            RplidarScanMode scanModeInfoTmp;
            memset(&scanModeInfoTmp, 0, sizeof(scanModeInfoTmp));
            scanModeInfoTmp.id = i;
            ans = getLidarSampleDuration(scanModeInfoTmp.us_per_sample, i);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
            ans = getMaxDistance(scanModeInfoTmp.max_distance, i);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
            ans = getScanModeAnsType(scanModeInfoTmp.ans_type, i);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
            ans = getScanModeName(scanModeInfoTmp.scan_mode, i);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
            outModes.push_back(scanModeInfoTmp);
        }
        return ans;
    }
    else
    {
        rplidar_response_sample_rate_t sampleRateTmp;
        ans = getSampleDuration_uS(sampleRateTmp);
        if (IS_FAIL(ans)) return RESULT_INVALID_DATA;
        //judge if support express scan
        bool ifSupportExpScan = false;
        ans = checkExpressScanSupported(ifSupportExpScan);
        if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

        RplidarScanMode stdScanModeInfo;
        stdScanModeInfo.id = RPLIDAR_CONF_SCAN_COMMAND_STD;
        stdScanModeInfo.us_per_sample = sampleRateTmp.std_sample_duration_us;
        stdScanModeInfo.max_distance = 16;
        stdScanModeInfo.ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT;
        strcpy(stdScanModeInfo.scan_mode, "Standard");
        outModes.push_back(stdScanModeInfo);
        if (ifSupportExpScan)
        {
            RplidarScanMode expScanModeInfo;
            expScanModeInfo.id = RPLIDAR_CONF_SCAN_COMMAND_EXPRESS;
            expScanModeInfo.us_per_sample = sampleRateTmp.express_sample_duration_us;
            expScanModeInfo.max_distance = 16;
            expScanModeInfo.ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
            strcpy(expScanModeInfo.scan_mode, "Express");
            outModes.push_back(expScanModeInfo);
        }
        return ans;
    }
    return ans;
}

u_result RPlidarDriverImplCommon::getScanModeCount(_u16& modeCount, _u32 timeoutInMs)
{
    u_result ans;
    std::vector<_u8> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_COUNT, answer, std::vector<_u8>(), timeoutInMs);

    if (IS_FAIL(ans)) {
        return ans;
    }
    if (answer.size() < sizeof(_u16)) {
        return RESULT_INVALID_DATA;
    }
    const _u16 *p_answer = reinterpret_cast<const _u16*>(&answer[0]);
    modeCount = *p_answer;

    return ans;
}


u_result RPlidarDriverImplCommon::startScan(bool force, bool useTypicalScan, _u32 options, RplidarScanMode* outUsedScanMode)
{
    u_result ans;

    bool ifSupportLidarConf = false;
    ans = checkSupportConfigCommands(ifSupportLidarConf);
    if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

    if (useTypicalScan)
    {
        //if support lidar config protocol
        if (ifSupportLidarConf)
        {
            _u16 typicalMode;
            ans = getTypicalScanMode(typicalMode);
            if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

            //call startScanExpress to do the job
            return startScanExpress(false, typicalMode, 0, outUsedScanMode);
        }
        //if old version of triangle lidar
        else
        {
            bool isExpScanSupported = false;
            ans = checkExpressScanSupported(isExpScanSupported);
            if (IS_FAIL(ans)) {
                return ans;
            }
            if (isExpScanSupported)
            {
                return startScanExpress(false, RPLIDAR_CONF_SCAN_COMMAND_EXPRESS, 0, outUsedScanMode);
            }
        }
    }
    
    // 'useTypicalScan' is false, just use normal scan mode
    if(ifSupportLidarConf)
    {
        if(outUsedScanMode)
        {
            outUsedScanMode->id = RPLIDAR_CONF_SCAN_COMMAND_STD;
            ans = getLidarSampleDuration(outUsedScanMode->us_per_sample, outUsedScanMode->id);
            if(IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }

            ans = getMaxDistance(outUsedScanMode->max_distance, outUsedScanMode->id);
            if(IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }

            ans = getScanModeAnsType(outUsedScanMode->ans_type, outUsedScanMode->id);
            if(IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }

            ans = getScanModeName(outUsedScanMode->scan_mode, outUsedScanMode->id);
            if(IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
        }
    }
    else
    {
        if(outUsedScanMode)
        {
            rplidar_response_sample_rate_t sampleRateTmp;
            ans = getSampleDuration_uS(sampleRateTmp);
            if(IS_FAIL(ans)) return RESULT_INVALID_DATA;
            outUsedScanMode->us_per_sample = sampleRateTmp.std_sample_duration_us;
            outUsedScanMode->max_distance = 16;
            outUsedScanMode->ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT;
            strcpy(outUsedScanMode->scan_mode, "Standard");
        }
    }

    return startScanNormal(force);
}

u_result RPlidarDriverImplCommon::startScanExpress(bool force, _u16 scanMode, _u32 options, RplidarScanMode* outUsedScanMode, _u32 timeout)
{
    u_result ans;
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    if (_isScanning) return RESULT_ALREADY_DONE;

    stop(); //force the previous operation to stop

    if (scanMode == RPLIDAR_CONF_SCAN_COMMAND_STD)
    {
        return startScan(force, false, 0, outUsedScanMode);
    }

    
    bool ifSupportLidarConf = false;
    ans = checkSupportConfigCommands(ifSupportLidarConf);
    if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

    if (outUsedScanMode)
    {
        outUsedScanMode->id = scanMode;
        if (ifSupportLidarConf)
        {
            ans = getLidarSampleDuration(outUsedScanMode->us_per_sample, outUsedScanMode->id);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }

            ans = getMaxDistance(outUsedScanMode->max_distance, outUsedScanMode->id);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }

            ans = getScanModeAnsType(outUsedScanMode->ans_type, outUsedScanMode->id);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }

            ans = getScanModeName(outUsedScanMode->scan_mode, outUsedScanMode->id);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
        }
        else
        {
            rplidar_response_sample_rate_t sampleRateTmp;
            ans = getSampleDuration_uS(sampleRateTmp);
            if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

            outUsedScanMode->us_per_sample = sampleRateTmp.express_sample_duration_us;
            outUsedScanMode->max_distance = 16;
            outUsedScanMode->ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
            strcpy(outUsedScanMode->scan_mode, "Express");
        }
    }

    //get scan answer type to specify how to wait data
    _u8 scanAnsType;
    if (ifSupportLidarConf)
        getScanModeAnsType(scanAnsType, scanMode);
    else
        scanAnsType = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;

    {
        rp::hal::AutoLocker l(_lock);

        rplidar_payload_express_scan_t scanReq;
        memset(&scanReq, 0, sizeof(scanReq));
        if (scanMode != RPLIDAR_CONF_SCAN_COMMAND_STD && scanMode != RPLIDAR_CONF_SCAN_COMMAND_EXPRESS)
            scanReq.working_mode = _u8(scanMode);
        scanReq.working_flags = options;
        
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_EXPRESS_SCAN, &scanReq, sizeof(scanReq)))) {
            return ans;
        }

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != scanAnsType) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        
        if (scanAnsType == RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED)
        {
            if (header_size < sizeof(rplidar_response_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _isScanning = true;
            _cachethread = CLASS_THREAD(RPlidarDriverImplCommon, _cacheCapsuledScanData);
		}else if(scanAnsType ==RPLIDAR_ANS_TYPE_MEASUREMENT_HQ ){
			 if (header_size < sizeof(rplidar_response_hq_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
			 _isScanning = true;
			 _cachethread = CLASS_THREAD(RPlidarDriverImplCommon, _cacheHqScanData);
		
		}
        else
        {
            if (header_size < sizeof(rplidar_response_ultra_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _isScanning = true;
            _cachethread = CLASS_THREAD(RPlidarDriverImplCommon, _cacheUltraCapsuledScanData);
        }

        if (_cachethread.getHandle() == 0) {
            return RESULT_OPERATION_FAIL;
        }
    }
    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::stop(_u32 timeout)
{
    u_result ans;
    _disableDataGrabbing();

    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_STOP))) {
            return ans;
        }
    }

    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::grabScanData(rplidar_response_measurement_node_t * nodebuffer, size_t & count, _u32 timeout)
{
    DEPRECATED_WARN("grabScanData()", "grabScanDataHq()");

    switch (_dataEvt.wait(timeout))
    {
    case rp::hal::Event::EVENT_TIMEOUT:
        count = 0;
        return RESULT_OPERATION_TIMEOUT;
    case rp::hal::Event::EVENT_OK:
        {
            if(_cached_scan_node_hq_count == 0) return RESULT_OPERATION_TIMEOUT; //consider as timeout

            rp::hal::AutoLocker l(_lock);

            size_t size_to_copy = min(count, _cached_scan_node_hq_count);

            for (size_t i = 0; i < size_to_copy; i++)
                convert(_cached_scan_node_hq_buf[i], nodebuffer[i]);

            count = size_to_copy;
            _cached_scan_node_hq_count = 0;
        }
        return RESULT_OK;

    default:
        count = 0;
        return RESULT_OPERATION_FAIL;
    }
}

u_result RPlidarDriverImplCommon::grabScanDataHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count, _u32 timeout)
{
    switch (_dataEvt.wait(timeout))
    {
    case rp::hal::Event::EVENT_TIMEOUT:
        count = 0;
        return RESULT_OPERATION_TIMEOUT;
    case rp::hal::Event::EVENT_OK:
    {
        if (_cached_scan_node_hq_count == 0) return RESULT_OPERATION_TIMEOUT; //consider as timeout

        rp::hal::AutoLocker l(_lock);

        size_t size_to_copy = min(count, _cached_scan_node_hq_count);
        memcpy(nodebuffer, _cached_scan_node_hq_buf, size_to_copy * sizeof(rplidar_response_measurement_node_hq_t));

        count = size_to_copy;
        _cached_scan_node_hq_count = 0;
    }
    return RESULT_OK;

    default:
        count = 0;
        return RESULT_OPERATION_FAIL;
    }
}

u_result RPlidarDriverImplCommon::getScanDataWithInterval(rplidar_response_measurement_node_t * nodebuffer, size_t & count)
{
    DEPRECATED_WARN("getScanDataWithInterval(rplidar_response_measurement_node_t*, size_t&)", "getScanDataWithInterval(rplidar_response_measurement_node_hq_t*, size_t&)");

    size_t size_to_copy = 0;
    {
        rp::hal::AutoLocker l(_lock);
        if(_cached_scan_node_hq_count_for_interval_retrieve == 0)
        {
            return RESULT_OPERATION_TIMEOUT; 
        }
        //copy all the nodes(_cached_scan_node_count_for_interval_retrieve nodes) in _cached_scan_node_buf_for_interval_retrieve
        size_to_copy = _cached_scan_node_hq_count_for_interval_retrieve;
        for (size_t i = 0; i < size_to_copy; i++)
        {
            convert(_cached_scan_node_hq_buf_for_interval_retrieve[i], nodebuffer[i]);
        }
        _cached_scan_node_hq_count_for_interval_retrieve = 0;
    }
    count = size_to_copy;

    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::getScanDataWithIntervalHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count)
{
    size_t size_to_copy = 0;
    {
        rp::hal::AutoLocker l(_lock);
        if (_cached_scan_node_hq_count_for_interval_retrieve == 0)
        {
            return RESULT_OPERATION_TIMEOUT;
        }
        //copy all the nodes(_cached_scan_node_count_for_interval_retrieve nodes) in _cached_scan_node_buf_for_interval_retrieve
        size_to_copy = _cached_scan_node_hq_count_for_interval_retrieve;
        memcpy(nodebuffer, _cached_scan_node_hq_buf_for_interval_retrieve, size_to_copy * sizeof(rplidar_response_measurement_node_hq_t));
        _cached_scan_node_hq_count_for_interval_retrieve = 0;
    }
    count = size_to_copy;

    return RESULT_OK;
}

static inline float getAngle(const rplidar_response_measurement_node_t& node)
{
    return (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.f;
}

static inline void setAngle(rplidar_response_measurement_node_t& node, float v)
{
    _u16 checkbit = node.angle_q6_checkbit & RPLIDAR_RESP_MEASUREMENT_CHECKBIT;
    node.angle_q6_checkbit = (((_u16)(v * 64.0f)) << RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) | checkbit;
}

static inline float getAngle(const rplidar_response_measurement_node_hq_t& node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}

static inline void setAngle(rplidar_response_measurement_node_hq_t& node, float v)
{
    node.angle_z_q14 = _u32(v * 16384.f / 90.f);
}

static inline _u16 getDistanceQ2(const rplidar_response_measurement_node_t& node)
{
    return node.distance_q2;
}

static inline _u32 getDistanceQ2(const rplidar_response_measurement_node_hq_t& node)
{
    return node.dist_mm_q2;
}

template <class TNode>
static bool angleLessThan(const TNode& a, const TNode& b)
{
    return getAngle(a) < getAngle(b);
}

template < class TNode >
static u_result ascendScanData_(TNode * nodebuffer, size_t count)
{
    float inc_origin_angle = 360.f/count;
    size_t i = 0;

    //Tune head
    for (i = 0; i < count; i++) {
        if(getDistanceQ2(nodebuffer[i]) == 0) {
            continue;
        } else {
            while(i != 0) {
                i--;
                float expect_angle = getAngle(nodebuffer[i+1]) - inc_origin_angle;
                if (expect_angle < 0.0f) expect_angle = 0.0f;
                setAngle(nodebuffer[i], expect_angle);
            }
            break;
        }
    }

    // all the data is invalid
    if (i == count) return RESULT_OPERATION_FAIL;

    //Tune tail
    for (i = count - 1; i >= 0; i--) {
        if(getDistanceQ2(nodebuffer[i]) == 0) {
            continue;
        } else {
            while(i != (count - 1)) {
                i++;
                float expect_angle = getAngle(nodebuffer[i-1]) + inc_origin_angle;
                if (expect_angle > 360.0f) expect_angle -= 360.0f;
                setAngle(nodebuffer[i], expect_angle);
            }
            break;
        }
    }

    //Fill invalid angle in the scan
    float frontAngle = getAngle(nodebuffer[0]);
    for (i = 1; i < count; i++) {
        if(getDistanceQ2(nodebuffer[i]) == 0) {
            float expect_angle =  frontAngle + i * inc_origin_angle;
            if (expect_angle > 360.0f) expect_angle -= 360.0f;
            setAngle(nodebuffer[i], expect_angle);
        }
    }

    // Reorder the scan according to the angle value
    std::sort(nodebuffer, nodebuffer + count, &angleLessThan<TNode>);

    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::ascendScanData(rplidar_response_measurement_node_t * nodebuffer, size_t count)
{
    DEPRECATED_WARN("ascendScanData(rplidar_response_measurement_node_t*, size_t)", "ascendScanData(rplidar_response_measurement_node_hq_t*, size_t)");

    return ascendScanData_<rplidar_response_measurement_node_t>(nodebuffer, count);
}

u_result RPlidarDriverImplCommon::ascendScanData(rplidar_response_measurement_node_hq_t * nodebuffer, size_t count)
{
    return ascendScanData_<rplidar_response_measurement_node_hq_t>(nodebuffer, count);
}

u_result RPlidarDriverImplCommon::_sendCommand(_u8 cmd, const void * payload, size_t payloadsize)
{
    _u8 pkt_header[10];
    rplidar_cmd_packet_t * header = reinterpret_cast<rplidar_cmd_packet_t * >(pkt_header);
    _u8 checksum = 0;

    if (!_isConnected) return RESULT_OPERATION_FAIL;

    if (payloadsize && payload) {
        cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd;

    // send header first
    _chanDev->senddata(pkt_header, 2) ;

    if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // calc checksum
        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((_u8 *)payload)[pos];
        }

        // send size
        _u8 sizebyte = payloadsize;
        _chanDev->senddata(&sizebyte, 1);

        // send payload
        _chanDev->senddata((const _u8 *)payload, sizebyte);

        // send checksum
        _chanDev->senddata(&checksum, 1);
    }

    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::getSampleDuration_uS(rplidar_response_sample_rate_t & rateInfo, _u32 timeout)
{  
    DEPRECATED_WARN("getSampleDuration_uS", "RplidarScanMode::us_per_sample");

    if (!isConnected()) return RESULT_OPERATION_FAIL;
    
    _disableDataGrabbing();
    
    rplidar_response_device_info_t devinfo;
    // 1. fetch the device version first...
    u_result ans = getDeviceInfo(devinfo, timeout);

    rateInfo.express_sample_duration_us = _cached_sampleduration_express;
    rateInfo.std_sample_duration_us = _cached_sampleduration_std;

    if (devinfo.firmware_version < ((0x1<<8) | 17)) {
        // provide fake data...

        return RESULT_OK;
    }


    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_SAMPLERATE))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_SAMPLE_RATE) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if ( header_size < sizeof(rplidar_response_sample_rate_t)) {
            return RESULT_INVALID_DATA;
        }

        if (!_chanDev->waitfordata(header_size, timeout)) {
            return RESULT_OPERATION_TIMEOUT;
        }
        _chanDev->recvdata(reinterpret_cast<_u8 *>(&rateInfo), sizeof(rateInfo));
    }
    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::checkMotorCtrlSupport(bool & support, _u32 timeout)
{
    u_result  ans;
    support = false;
    
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    
    _disableDataGrabbing();

    {
        rp::hal::AutoLocker l(_lock);

        rplidar_payload_acc_board_flag_t flag;
        flag.reserved = 0;

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_ACC_BOARD_FLAG, &flag, sizeof(flag)))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }
        
        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_ACC_BOARD_FLAG) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if ( header_size < sizeof(rplidar_response_acc_board_flag_t)) {
            return RESULT_INVALID_DATA;
        }

        if (!_chanDev->waitfordata(header_size, timeout)) {
            return RESULT_OPERATION_TIMEOUT;
        }
        rplidar_response_acc_board_flag_t acc_board_flag;
        _chanDev->recvdata(reinterpret_cast<_u8 *>(&acc_board_flag), sizeof(acc_board_flag));

        if (acc_board_flag.support_flag & RPLIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK) {
            support = true;
        }
    }
    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::setMotorPWM(_u16 pwm)
{
    u_result ans;
    rplidar_payload_motor_pwm_t motor_pwm;
    motor_pwm.pwm_value = pwm;

    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_SET_MOTOR_PWM,(const _u8 *)&motor_pwm, sizeof(motor_pwm)))) {
            return ans;
        }
    }

    return RESULT_OK;
}

u_result RPlidarDriverImplCommon::startMotor()
{
    if (_isSupportingMotorCtrl) { // RPLIDAR A2
        setMotorPWM(DEFAULT_MOTOR_PWM);
        delay(500);
        return RESULT_OK;
    } else { // RPLIDAR A1
        rp::hal::AutoLocker l(_lock);
        _chanDev->clearDTR();
        delay(500);
        return RESULT_OK;
    }
}

u_result RPlidarDriverImplCommon::stopMotor()
{
    if (_isSupportingMotorCtrl) { // RPLIDAR A2
        setMotorPWM(0);
        delay(500);
        return RESULT_OK;
    } else { // RPLIDAR A1
        rp::hal::AutoLocker l(_lock);
        _chanDev->setDTR();
        delay(500);
        return RESULT_OK;
    }
}

void RPlidarDriverImplCommon::_disableDataGrabbing()
{
    _isScanning = false;
    _cachethread.join();
}

// Serial Driver Impl

RPlidarDriverSerial::RPlidarDriverSerial() 
{
    _chanDev = new SerialChannelDevice();
}

RPlidarDriverSerial::~RPlidarDriverSerial()
{
    // force disconnection
    disconnect();
    
    _chanDev->close();
    _chanDev->ReleaseRxTx();
}

void RPlidarDriverSerial::disconnect()
{
    if (!_isConnected) return ;
    stop();
}

u_result RPlidarDriverSerial::connect(const char * port_path, _u32 baudrate, _u32 flag)
{
    if (isConnected()) return RESULT_ALREADY_DONE;

    if (!_chanDev) return RESULT_INSUFFICIENT_MEMORY;

    {
        rp::hal::AutoLocker l(_lock);

        // establish the serial connection...
        if (!_chanDev->bind(port_path, baudrate)  ||  !_chanDev->open()) {
            return RESULT_INVALID_DATA;
        }
        _chanDev->flush();
    }

    _isConnected = true;

    checkMotorCtrlSupport(_isSupportingMotorCtrl);
    stopMotor();

    return RESULT_OK;
}

RPlidarDriverTCP::RPlidarDriverTCP() 
{
    _chanDev = new TCPChannelDevice();
}

RPlidarDriverTCP::~RPlidarDriverTCP()
{
    // force disconnection
    disconnect();
}

void RPlidarDriverTCP::disconnect()
{
    if (!_isConnected) return ;
    stop();
    _chanDev->close();
}

u_result RPlidarDriverTCP::connect(const char * ipStr, _u32 port, _u32 flag)
{
    if (isConnected()) return RESULT_ALREADY_DONE;

    if (!_chanDev) return RESULT_INSUFFICIENT_MEMORY;

    {
        rp::hal::AutoLocker l(_lock);

        // establish the serial connection...
        if(!_chanDev->bind(ipStr, port))
            return RESULT_INVALID_DATA;
    }

    _isConnected = true;

    checkMotorCtrlSupport(_isSupportingMotorCtrl);
    stopMotor();

    return RESULT_OK;
}

}}}
