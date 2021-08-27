/*
 * Slamtec LIDAR SDK
 *
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

#include "sdkcommon.h"
#include "hal/abs_rxtx.h"
#include "hal/thread.h"
#include "hal/types.h"
#include "hal/assert.h"
#include "hal/locker.h"
#include "hal/socket.h"
#include "hal/event.h"
#include "sl_lidar_driver.h"
#include "sl_crc.h" 
#include <algorithm>

#ifdef _WIN32
#define NOMINMAX
#undef min
#undef max
#endif

#if defined(__cplusplus) && __cplusplus >= 201103L
#ifndef _GXX_NULLPTR_T
#define _GXX_NULLPTR_T
typedef decltype(nullptr) nullptr_t;
#endif
#endif /* C++11.  */

namespace sl {
    static void printDeprecationWarn(const char* fn, const char* replacement)
    {
        fprintf(stderr, "*WARN* YOU ARE USING DEPRECATED API: %s, PLEASE MOVE TO %s\n", fn, replacement);
    }

    static void convert(const sl_lidar_response_measurement_node_t& from, sl_lidar_response_measurement_node_hq_t& to)
    {
        to.angle_z_q14 = (((from.angle_q6_checkbit) >> SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) << 8) / 90;  //transfer to q14 Z-angle
        to.dist_mm_q2 = from.distance_q2;
        to.flag = (from.sync_quality & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT);  // trasfer syncbit to HQ flag field
        to.quality = (from.sync_quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;  //remove the last two bits and then make quality from 0-63 to 0-255
    }

    static void convert(const sl_lidar_response_measurement_node_hq_t& from, sl_lidar_response_measurement_node_t& to)
    {
        to.sync_quality = (from.flag & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT) | ((from.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        to.angle_q6_checkbit = 1 | (((from.angle_z_q14 * 90) >> 8) << SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT);
        to.distance_q2 = from.dist_mm_q2 > sl_u16(-1) ? sl_u16(0) : sl_u16(from.dist_mm_q2);
    }

    static sl_u32 _varbitscale_decode(sl_u32 scaled, sl_u32 & scaleLevel)
    {
        static const sl_u32 VBS_SCALED_BASE[] = {
            SL_LIDAR_VARBITSCALE_X16_DEST_VAL,
            SL_LIDAR_VARBITSCALE_X8_DEST_VAL,
            SL_LIDAR_VARBITSCALE_X4_DEST_VAL,
            SL_LIDAR_VARBITSCALE_X2_DEST_VAL,
            0,
        };

        static const sl_u32 VBS_SCALED_LVL[] = {
            4,
            3,
            2,
            1,
            0,
        };

        static const sl_u32 VBS_TARGET_BASE[] = {
            (0x1 << SL_LIDAR_VARBITSCALE_X16_SRC_BIT),
            (0x1 << SL_LIDAR_VARBITSCALE_X8_SRC_BIT),
            (0x1 << SL_LIDAR_VARBITSCALE_X4_SRC_BIT),
            (0x1 << SL_LIDAR_VARBITSCALE_X2_SRC_BIT),
            0,
        };

        for (size_t i = 0; i < _countof(VBS_SCALED_BASE); ++i) {
            int remain = ((int)scaled - (int)VBS_SCALED_BASE[i]);
            if (remain >= 0) {
                scaleLevel = VBS_SCALED_LVL[i];
                return VBS_TARGET_BASE[i] + (remain << scaleLevel);
            }
        }
        return 0;
    }

    static inline float getAngle(const sl_lidar_response_measurement_node_t& node)
    {
        return (node.angle_q6_checkbit >> SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.f;
    }

    static inline void setAngle(sl_lidar_response_measurement_node_t& node, float v)
    {
        sl_u16 checkbit = node.angle_q6_checkbit & SL_LIDAR_RESP_MEASUREMENT_CHECKBIT;
        node.angle_q6_checkbit = (((sl_u16)(v * 64.0f)) << SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) | checkbit;
    }

    static inline float getAngle(const sl_lidar_response_measurement_node_hq_t& node)
    {
        return node.angle_z_q14 * 90.f / 16384.f;
    }

    static inline void setAngle(sl_lidar_response_measurement_node_hq_t& node, float v)
    {
        node.angle_z_q14 = sl_u32(v * 16384.f / 90.f);
    }

    static inline sl_u16 getDistanceQ2(const sl_lidar_response_measurement_node_t& node)
    {
        return node.distance_q2;
    }

    static inline sl_u32 getDistanceQ2(const sl_lidar_response_measurement_node_hq_t& node)
    {
        return node.dist_mm_q2;
    }
   
    template <class TNode>
    static bool angleLessThan(const TNode& a, const TNode& b)
    {
        return getAngle(a) < getAngle(b);
    }

    template < class TNode >
    static sl_result ascendScanData_(TNode * nodebuffer, size_t count)
    {
        float inc_origin_angle = 360.f / count;
        size_t i = 0;

        //Tune head
        for (i = 0; i < count; i++) {
            if (getDistanceQ2(nodebuffer[i]) == 0) {
                continue;
            }
            else {
                while (i != 0) {
                    i--;
                    float expect_angle = getAngle(nodebuffer[i + 1]) - inc_origin_angle;
                    if (expect_angle < 0.0f) expect_angle = 0.0f;
                    setAngle(nodebuffer[i], expect_angle);
                }
                break;
            }
        }

        // all the data is invalid
        if (i == count) return SL_RESULT_OPERATION_FAIL;

        //Tune tail
        for (i = count - 1; i >= 0; i--) {
            if (getDistanceQ2(nodebuffer[i]) == 0) {
                continue;
            }
            else {
                while (i != (count - 1)) {
                    i++;
                    float expect_angle = getAngle(nodebuffer[i - 1]) + inc_origin_angle;
                    if (expect_angle > 360.0f) expect_angle -= 360.0f;
                    setAngle(nodebuffer[i], expect_angle);
                }
                break;
            }
        }

        //Fill invalid angle in the scan
        float frontAngle = getAngle(nodebuffer[0]);
        for (i = 1; i < count; i++) {
            if (getDistanceQ2(nodebuffer[i]) == 0) {
                float expect_angle = frontAngle + i * inc_origin_angle;
                if (expect_angle > 360.0f) expect_angle -= 360.0f;
                setAngle(nodebuffer[i], expect_angle);
            }
        }

        // Reorder the scan according to the angle value
        std::sort(nodebuffer, nodebuffer + count, &angleLessThan<TNode>);

        return SL_RESULT_OK;
    }

    class SlamtecLidarDriver :public ILidarDriver
    {
    public:
        enum {
            LEGACY_SAMPLE_DURATION = 476,
        };

        enum {
             NORMAL_CAPSULE = 0,
             DENSE_CAPSULE = 1,
        };

        enum {
            A2A3_LIDAR_MINUM_MAJOR_ID  = 2,
            TOF_LIDAR_MINUM_MAJOR_ID = 6,
        };

    public:
        SlamtecLidarDriver()
            : _channel(NULL)
            , _isConnected(false)
            , _isScanning(false)
            , _isSupportingMotorCtrl(MotorCtrlSupportNone)
            , _cached_sampleduration_std(LEGACY_SAMPLE_DURATION)
            , _cached_sampleduration_express(LEGACY_SAMPLE_DURATION)
            , _cached_scan_node_hq_count(0)
            , _cached_scan_node_hq_count_for_interval_retrieve(0)
        {}

        sl_result connect(IChannel* channel)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;

            if (!channel) return SL_RESULT_OPERATION_FAIL;
            if (isConnected()) return SL_RESULT_ALREADY_DONE;
            _channel = channel;
            
            {
                rp::hal::AutoLocker l(_lock);
                ans = _channel->open();
                if (!ans)
                    return SL_RESULT_OPERATION_FAIL;

                _channel->flush();
            }
     
            _isConnected = true;

            ans =checkMotorCtrlSupport(_isSupportingMotorCtrl,500);
            return SL_RESULT_OK;
        
        }

        void disconnect()
        {
            if (_isConnected)
                _channel->close();
        }

        bool isConnected()
        {
            return _isConnected;
        }

        sl_result reset(sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(SL_LIDAR_CMD_RESET);
                if (!ans) {
                    return ans;
                }
            }
            return SL_RESULT_OK;
        }

        sl_result getAllSupportedScanModes(std::vector<LidarScanMode>& outModes, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            bool confProtocolSupported = false;
            ans = checkSupportConfigCommands(confProtocolSupported);
            if (!ans) return SL_RESULT_INVALID_DATA;

            if (confProtocolSupported) {
                // 1. get scan mode count
                sl_u16 modeCount;
                ans = getScanModeCount(modeCount);
                if (!ans) return ans;
                // 2. for loop to get all fields of each scan mode
                for (sl_u16 i = 0; i < modeCount; i++) {
                    LidarScanMode scanModeInfoTmp;
                    memset(&scanModeInfoTmp, 0, sizeof(scanModeInfoTmp));
                    scanModeInfoTmp.id = i;
                    ans = getLidarSampleDuration(scanModeInfoTmp.us_per_sample, i);
                    if (!ans) return ans;
                    ans = getMaxDistance(scanModeInfoTmp.max_distance, i);
                    if (!ans) return ans;
                    ans = getScanModeAnsType(scanModeInfoTmp.ans_type, i);
                    if (!ans) return ans;
                    ans = getScanModeName(scanModeInfoTmp.scan_mode, i);
                    if (!ans) return ans;
                    outModes.push_back(scanModeInfoTmp);

                }
                return ans;
            }

            return ans;        
        }

        sl_result getTypicalScanMode(sl_u16& outMode, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            std::vector<sl_u8> answer;
            bool lidarSupportConfigCmds = false;
            ans = checkSupportConfigCommands(lidarSupportConfigCmds);
            if (!ans) return ans;

            if (lidarSupportConfigCmds) {
                ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_TYPICAL, answer, std::vector<sl_u8>(), timeoutInMs);
                if (!ans) return ans;
                if (answer.size() < sizeof(sl_u16)) {
                    return SL_RESULT_INVALID_DATA;
                }
                const sl_u16 *p_answer = reinterpret_cast<const sl_u16*>(&answer[0]);
                outMode = *p_answer;
                return ans;
            }
            //old version of triangle lidar
            else {
                outMode = SL_LIDAR_CONF_SCAN_COMMAND_EXPRESS;
                return ans;
            }
            return ans;
    
        }

        sl_result startScan(bool force, bool useTypicalScan, sl_u32 options = 0, LidarScanMode* outUsedScanMode = nullptr)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            bool ifSupportLidarConf = false;
            startMotor();
            ans = checkSupportConfigCommands(ifSupportLidarConf);
            if (!ans) return ans;
            if (useTypicalScan){
                sl_u16 typicalMode;
                ans = getTypicalScanMode(typicalMode);
                if (!ans) return ans;

                //call startScanExpress to do the job
                return startScanExpress(false, typicalMode, 0, outUsedScanMode);
            }

            // 'useTypicalScan' is false, just use normal scan mode
            if (ifSupportLidarConf) {
                if (outUsedScanMode) {
                    outUsedScanMode->id = SL_LIDAR_CONF_SCAN_COMMAND_STD;
                    ans = getLidarSampleDuration(outUsedScanMode->us_per_sample, outUsedScanMode->id);
                    if (!ans) return ans;
                    ans = getMaxDistance(outUsedScanMode->max_distance, outUsedScanMode->id);
                    if (!ans) return ans;
                    ans = getScanModeAnsType(outUsedScanMode->ans_type, outUsedScanMode->id);
                    if (!ans) return ans;
                    ans = getScanModeName(outUsedScanMode->scan_mode, outUsedScanMode->id);
                    if (!ans) return ans;
                }
            }

            return startScanNormal(force);
        }

        sl_result startScanNormal(bool force, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            if (!isConnected()) return SL_RESULT_OPERATION_FAIL;
            if (_isScanning) return SL_RESULT_ALREADY_DONE;

            stop(); //force the previous operation to stop
            setMotorSpeed();
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(force ? SL_LIDAR_CMD_FORCE_SCAN : SL_LIDAR_CMD_SCAN);
                if (!ans) return ans;
                // waiting for confirmation
                sl_lidar_ans_header_t response_header;
                ans = _waitResponseHeader(&response_header, timeout);
                if (!ans) return ans;

                // verify whether we got a correct header
                if (response_header.type != SL_LIDAR_ANS_TYPE_MEASUREMENT) {
                    return SL_RESULT_INVALID_DATA;
                }

                sl_u32 header_size = (response_header.size_q30_subtype & SL_LIDAR_ANS_HEADER_SIZE_MASK);
                if (header_size < sizeof(sl_lidar_response_measurement_node_t)) {
                    return SL_RESULT_INVALID_DATA;
                }
                _isScanning = true;
                _cachethread = CLASS_THREAD(SlamtecLidarDriver, _cacheScanData);
                if (_cachethread.getHandle() == 0) {
                    return SL_RESULT_OPERATION_FAIL;
                }
            }
            return SL_RESULT_OK;
        }

        sl_result startScanExpress(bool force, sl_u16 scanMode, sl_u32 options = 0, LidarScanMode* outUsedScanMode = nullptr, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            if (!isConnected()) return SL_RESULT_OPERATION_FAIL;
            if (_isScanning) return SL_RESULT_ALREADY_DONE;
            stop(); //force the previous operation to stop

            bool ifSupportLidarConf = false;
            ans = checkSupportConfigCommands(ifSupportLidarConf);
            if (!ans) return SL_RESULT_INVALID_DATA;


            if (outUsedScanMode) {
                outUsedScanMode->id = scanMode;
                if (ifSupportLidarConf) {
                    ans = getLidarSampleDuration(outUsedScanMode->us_per_sample, outUsedScanMode->id);
                    if (!ans) return SL_RESULT_INVALID_DATA;

                    ans = getMaxDistance(outUsedScanMode->max_distance, outUsedScanMode->id);
                    if (!ans) return SL_RESULT_INVALID_DATA;

                    ans = getScanModeAnsType(outUsedScanMode->ans_type, outUsedScanMode->id);
                    if (!ans) return SL_RESULT_INVALID_DATA;

                    ans = getScanModeName(outUsedScanMode->scan_mode, outUsedScanMode->id);
                    if (!ans) return SL_RESULT_INVALID_DATA;


                }

            }

            //get scan answer type to specify how to wait data
            sl_u8 scanAnsType;
            if (ifSupportLidarConf) {
                getScanModeAnsType(scanAnsType, scanMode);
            }
            else {
                scanAnsType = SL_LIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
            }
            if (!ifSupportLidarConf || scanAnsType == SL_LIDAR_ANS_TYPE_MEASUREMENT) {
                if (scanMode == SL_LIDAR_CONF_SCAN_COMMAND_STD) {
                    return startScan(force, false, 0, outUsedScanMode);
                }
            }
            {
                rp::hal::AutoLocker l(_lock);

                startMotor();
                sl_lidar_payload_express_scan_t scanReq;
                memset(&scanReq, 0, sizeof(scanReq));
                if (!ifSupportLidarConf){
                    if (scanMode != SL_LIDAR_CONF_SCAN_COMMAND_STD && scanMode != SL_LIDAR_CONF_SCAN_COMMAND_EXPRESS)
                        scanReq.working_mode = sl_u8(scanMode);
                }else
                    scanReq.working_mode = sl_u8(scanMode);

                scanReq.working_flags = options;
				delay(5);
                ans = _sendCommand(SL_LIDAR_CMD_EXPRESS_SCAN, &scanReq, sizeof(scanReq));
                if (!ans) { 
                    ans = _sendCommand(SL_LIDAR_CMD_EXPRESS_SCAN, &scanReq, sizeof(scanReq));
                    if (!ans)
                        return SL_RESULT_INVALID_DATA; 
                }
     

                // waiting for confirmation
                sl_lidar_ans_header_t response_header;
                ans = _waitResponseHeader(&response_header, timeout);
                if (!ans) return ans;

                // verify whether we got a correct header
                if (response_header.type != scanAnsType) {
                    return SL_RESULT_INVALID_DATA;
                }

                sl_u32 header_size = (response_header.size_q30_subtype & SL_LIDAR_ANS_HEADER_SIZE_MASK);

                if (scanAnsType == SL_LIDAR_ANS_TYPE_MEASUREMENT_CAPSULED) {
                    if (header_size < sizeof(sl_lidar_response_capsule_measurement_nodes_t)) {
                        return SL_RESULT_INVALID_DATA;
                    }
                    _cached_capsule_flag = NORMAL_CAPSULE;
                    _isScanning = true;
                    _cachethread = CLASS_THREAD(SlamtecLidarDriver, _cacheCapsuledScanData);
                }
                else if (scanAnsType == SL_LIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED) {
                    if (header_size < sizeof(sl_lidar_response_capsule_measurement_nodes_t)) {
                        return SL_RESULT_INVALID_DATA;
                    }
                    _cached_capsule_flag = DENSE_CAPSULE;
                    _isScanning = true;
                    _cachethread = CLASS_THREAD(SlamtecLidarDriver, _cacheCapsuledScanData);
                }
                else if (scanAnsType == SL_LIDAR_ANS_TYPE_MEASUREMENT_HQ) {
                    if (header_size < sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t)) {
                        return SL_RESULT_INVALID_DATA;
                    }
                    _isScanning = true;
                    _cachethread = CLASS_THREAD(SlamtecLidarDriver, _cacheHqScanData);
                }
                else {
                    if (header_size < sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)) {
                        return SL_RESULT_INVALID_DATA;
                    }
                    _isScanning = true;
                    _cachethread = CLASS_THREAD(SlamtecLidarDriver, _cacheUltraCapsuledScanData);
                }

                if (_cachethread.getHandle() == 0) {
                    return SL_RESULT_OPERATION_FAIL;
                }
            }
            return SL_RESULT_OK;

        }

        sl_result stop(sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            _disableDataGrabbing();

            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(SL_LIDAR_CMD_STOP);
                if (!ans) return ans;
            }
            delay(100);

            if(_isSupportingMotorCtrl == MotorCtrlSupportPwm)
                setMotorSpeed(0);
  
            return SL_RESULT_OK;
        }
       
        sl_result grabScanDataHq(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t& count, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            switch (_dataEvt.wait(timeout))
            {
            case rp::hal::Event::EVENT_TIMEOUT:
                count = 0;
                return SL_RESULT_OPERATION_TIMEOUT;
            case rp::hal::Event::EVENT_OK:
            {
                if (_cached_scan_node_hq_count == 0) return SL_RESULT_OPERATION_TIMEOUT; //consider as timeout

                rp::hal::AutoLocker l(_lock);

                size_t size_to_copy = std::min(count, _cached_scan_node_hq_count);
                memcpy(nodebuffer, _cached_scan_node_hq_buf, size_to_copy * sizeof(sl_lidar_response_measurement_node_hq_t));

                count = size_to_copy;
                _cached_scan_node_hq_count = 0;
            }
            return SL_RESULT_OK;

            default:
                count = 0;
                return SL_RESULT_OPERATION_FAIL;
            }
        }

        sl_result getDeviceInfo(sl_lidar_response_device_info_t& info, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            _disableDataGrabbing();
			delay(20);
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(SL_LIDAR_CMD_GET_DEVICE_INFO);
                if (!ans) return ans;
                return _waitResponse(info, SL_LIDAR_ANS_TYPE_DEVINFO);
            }           
        }

        sl_result checkMotorCtrlSupport(MotorCtrlSupport & support, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            support = MotorCtrlSupportNone;
            _disableDataGrabbing();
            {
                sl_lidar_response_device_info_t devInfo;
                ans = getDeviceInfo(devInfo, 500);
                if (!ans) return ans;
                sl_u8 majorId = devInfo.model >> 4;
                if (majorId >= TOF_LIDAR_MINUM_MAJOR_ID) {
                        support = MotorCtrlSupportRpm;
                        return ans;
                }
                else if(majorId >= A2A3_LIDAR_MINUM_MAJOR_ID){

                    rp::hal::AutoLocker l(_lock);
                    sl_lidar_payload_acc_board_flag_t flag;
                    flag.reserved = 0;
                    ans = _sendCommand(SL_LIDAR_CMD_GET_ACC_BOARD_FLAG, &flag, sizeof(flag));
                    if (!ans) return ans;

                    sl_lidar_response_acc_board_flag_t acc_board_flag;
                    ans = _waitResponse(acc_board_flag, SL_LIDAR_ANS_TYPE_ACC_BOARD_FLAG);
                    if (acc_board_flag.support_flag & SL_LIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK) {
                        support = MotorCtrlSupportPwm;
                    }
                    return ans;
                }

            }
            return SL_RESULT_OK;

        }

        sl_result getFrequency(const LidarScanMode& scanMode, const sl_lidar_response_measurement_node_hq_t* nodes, size_t count, float& frequency)
        {
            float sample_duration = scanMode.us_per_sample;
            frequency = 1000000.0f / (count * sample_duration);
            return SL_RESULT_OK;
        }

		sl_result setLidarIpConf(const sl_lidar_ip_conf_t& conf, sl_u32 timeout)
		{
			sl_result ans = setLidarConf(SL_LIDAR_CONF_LIDAR_STATIC_IP_ADDR, &conf, sizeof(sl_lidar_ip_conf_t), timeout);
			return ans;
		}

        sl_result getHealth(sl_lidar_response_device_health_t& health, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;

            if (!isConnected()) 
                return SL_RESULT_OPERATION_FAIL;

            _disableDataGrabbing();

            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(SL_LIDAR_CMD_GET_DEVICE_HEALTH);
                if (!ans) return ans;
                delay(50);
                ans = _waitResponse(health, SL_LIDAR_ANS_TYPE_DEVHEALTH);
                
            }
            return ans;
            
        }

		sl_result getDeviceMacAddr(sl_u8* macAddrArray, sl_u32 timeoutInMs)
		{
			Result<nullptr_t> ans = SL_RESULT_OK;

			std::vector<sl_u8> answer;
			ans = getLidarConf(SL_LIDAR_CONF_LIDAR_MAC_ADDR, answer, std::vector<sl_u8>(), timeoutInMs);
			if (!ans) return ans;

			int len = answer.size();
			if (0 == len) return SL_RESULT_INVALID_DATA;
			memcpy(macAddrArray, &answer[0], len);
			return ans;
		}

        sl_result ascendScanData(sl_lidar_response_measurement_node_hq_t * nodebuffer, size_t count)
        {
            return ascendScanData_<sl_lidar_response_measurement_node_hq_t>(nodebuffer, count);
        }

        sl_result getScanDataWithIntervalHq(sl_lidar_response_measurement_node_hq_t * nodebuffer, size_t & count)
        {
            size_t size_to_copy = 0;
            {
                rp::hal::AutoLocker l(_lock);
                if (_cached_scan_node_hq_count_for_interval_retrieve == 0) {
                    return SL_RESULT_OPERATION_TIMEOUT;
                }
                //copy all the nodes(_cached_scan_node_count_for_interval_retrieve nodes) in _cached_scan_node_buf_for_interval_retrieve
                size_to_copy = _cached_scan_node_hq_count_for_interval_retrieve;
                memcpy(nodebuffer, _cached_scan_node_hq_buf_for_interval_retrieve, size_to_copy * sizeof(sl_lidar_response_measurement_node_hq_t));
                _cached_scan_node_hq_count_for_interval_retrieve = 0;
            }
            count = size_to_copy;

            return SL_RESULT_OK;
        }
        sl_result setMotorSpeed(sl_u16 speed = DEFAULT_MOTOR_SPEED)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            
            if(speed == DEFAULT_MOTOR_SPEED){
                sl_lidar_response_desired_rot_speed_t desired_speed;
                ans = getDesiredSpeed(desired_speed);
                if (!ans) return ans;
                if(_isSupportingMotorCtrl == MotorCtrlSupportPwm)
                    speed = desired_speed.pwm_ref;
                else
                    speed = desired_speed.rpm;
            }
            switch (_isSupportingMotorCtrl)
            {
            case MotorCtrlSupportNone:
                break;
            case MotorCtrlSupportPwm:
                sl_lidar_payload_motor_pwm_t motor_pwm;
                motor_pwm.pwm_value = speed;
                ans = _sendCommand(SL_LIDAR_CMD_SET_MOTOR_PWM, (const sl_u8 *)&motor_pwm, sizeof(motor_pwm));
                if (!ans) return ans;
                break;
            case MotorCtrlSupportRpm:
                sl_lidar_payload_motor_pwm_t motor_rpm;
                motor_rpm.pwm_value = speed;
                ans = _sendCommand(SL_LIDAR_CMD_HQ_MOTOR_SPEED_CTRL, (const sl_u8 *)&motor_rpm, sizeof(motor_rpm));
                if (!ans) return ans;
                break;
            }
            return SL_RESULT_OK;
        }

        sl_result getMotorInfo(LidarMotorInfo &motorInfo, sl_u32 timeoutInMs)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            rp::hal::AutoLocker l(_lock);
            {
                std::vector<sl_u8> answer;

			    ans = getLidarConf(RPLIDAR_CONF_MIN_ROT_FREQ, answer, std::vector<sl_u8>());
			    if (!ans) return ans;

			    const sl_u16 *min_answer = reinterpret_cast<const sl_u16*>(&answer[0]);
                motorInfo.min_speed = *min_answer;


                ans = getLidarConf(RPLIDAR_CONF_MAX_ROT_FREQ, answer, std::vector<sl_u8>());
			    if (!ans) return ans;

			    const sl_u16 *max_answer = reinterpret_cast<const sl_u16*>(&answer[0]);
                motorInfo.max_speed = *max_answer;

                sl_lidar_response_desired_rot_speed_t desired_speed;
                ans = getDesiredSpeed(desired_speed);
                if (!ans) return ans;
                if(motorInfo.motorCtrlSupport == MotorCtrlSupportPwm)
                    motorInfo.desired_speed = desired_speed.pwm_ref;
                else
                    motorInfo.desired_speed = desired_speed.rpm;

            }
            return SL_RESULT_OK;
        }

    protected:
        sl_result startMotor()
        {
            return setMotorSpeed(600);
        }

        sl_result getDesiredSpeed(sl_lidar_response_desired_rot_speed_t & motorSpeed, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            std::vector<sl_u8> answer;
            ans = getLidarConf(SL_LIDAR_CONF_DESIRED_ROT_FREQ, answer, std::vector<sl_u8>(), timeoutInMs);

            if (!ans) return ans;

            const sl_lidar_response_desired_rot_speed_t *p_answer = reinterpret_cast<const sl_lidar_response_desired_rot_speed_t*>(&answer[0]);
            motorSpeed = *p_answer;
            return SL_RESULT_OK;
        }

        sl_result checkSupportConfigCommands(bool& outSupport, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            sl_lidar_response_device_info_t devinfo;
            ans = getDeviceInfo(devinfo, timeoutInMs);
            if (!ans) return ans;

            sl_u16 modecount;
            ans = getScanModeCount(modecount, 250);
            if ((sl_result)ans == SL_RESULT_OK)
                outSupport = true;

            return SL_RESULT_OK;
        }

        sl_result getScanModeCount(sl_u16& modeCount, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            std::vector<sl_u8> answer;
            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_COUNT, answer, std::vector<sl_u8>(), timeoutInMs);

            if (!ans) return ans;

            const sl_u16 *p_answer = reinterpret_cast<const sl_u16*>(&answer[0]);
            modeCount = *p_answer;
            return SL_RESULT_OK;
        }

		sl_result setLidarConf(sl_u32 type, const void* payload, size_t payloadSize, sl_u32 timeout)
		{
			if (type < 0x00010000 || type >0x0001FFFF)
				return SL_RESULT_INVALID_DATA;
			std::vector<sl_u8> requestPkt;
			requestPkt.resize(sizeof(sl_lidar_payload_set_scan_conf_t) + payloadSize);
			if (!payload) payloadSize = 0;
			sl_lidar_payload_set_scan_conf_t* query = reinterpret_cast<sl_lidar_payload_set_scan_conf_t*>(&requestPkt[0]);

			query->type = type;

			if (payloadSize)
				memcpy(&query[1], payload, payloadSize);

			sl_result ans;
			{
				rp::hal::AutoLocker l(_lock);
				if (IS_FAIL(ans = _sendCommand(SL_LIDAR_CMD_SET_LIDAR_CONF, &requestPkt[0], requestPkt.size()))) {//
					return ans;
				}
				delay(20);
				// waiting for confirmation
				sl_lidar_ans_header_t response_header;
				if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
					return ans;
				}
				// verify whether we got a correct header
				if (response_header.type != SL_LIDAR_ANS_TYPE_SET_LIDAR_CONF) {
					return SL_RESULT_INVALID_DATA;
				}
				sl_u32 header_size = (response_header.size_q30_subtype & SL_LIDAR_ANS_HEADER_SIZE_MASK);
				if (header_size < sizeof(type)) {
					return SL_RESULT_INVALID_DATA;
				}
				if (!_channel->waitForData(header_size, timeout)) {
					return SL_RESULT_OPERATION_TIMEOUT;
				}
				delay(100);
				struct _sl_lidar_response_set_lidar_conf {
					sl_u32 type;
					sl_u32 result;
				} answer;

				_channel->read(reinterpret_cast<sl_u8*>(&answer), header_size);
				return answer.result;
    
			}

		}

        sl_result getLidarConf(sl_u32 type, std::vector<sl_u8> &outputBuf, const std::vector<sl_u8> &reserve = std::vector<sl_u8>(), sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            sl_lidar_payload_get_scan_conf_t query;
            query.type = type;
            int sizeVec = reserve.size();

            int maxLen = sizeof(query.reserved) / sizeof(query.reserved[0]);
            if (sizeVec > maxLen) sizeVec = maxLen;

            if (sizeVec > 0)
                memcpy(query.reserved, &reserve[0], reserve.size());

            Result<nullptr_t> ans = SL_RESULT_OK;
            { 
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(SL_LIDAR_CMD_GET_LIDAR_CONF, &query, sizeof(query));
                if (!ans) return ans;
				//delay(50);
                // waiting for confirmation
                sl_lidar_ans_header_t response_header;
                ans = _waitResponseHeader(&response_header, timeout);
                if (!ans)return ans;

                // verify whether we got a correct header
                if (response_header.type != SL_LIDAR_ANS_TYPE_GET_LIDAR_CONF) {
                    return SL_RESULT_INVALID_DATA;
                }

                sl_u32 header_size = (response_header.size_q30_subtype & SL_LIDAR_ANS_HEADER_SIZE_MASK);
                if (header_size < sizeof(type)) {
                    return SL_RESULT_INVALID_DATA;
                }
				//delay(100);
                if (!_channel->waitForData(header_size, timeout)) {
                    return SL_RESULT_OPERATION_TIMEOUT;
                }

                std::vector<sl_u8> dataBuf;
                dataBuf.resize(header_size);
                _channel->read(reinterpret_cast<sl_u8 *>(&dataBuf[0]), header_size);

                //check if returned type is same as asked type
                sl_u32 replyType = -1;
                memcpy(&replyType, &dataBuf[0], sizeof(type));
                if (replyType != type) {
                    return SL_RESULT_INVALID_DATA;
                }

                //copy all the payload into &outputBuf
                int payLoadLen = header_size - sizeof(type);

                //do consistency check
                if (payLoadLen <= 0) {
                    return SL_RESULT_INVALID_DATA;
                }
                //copy all payLoadLen bytes to outputBuf
                outputBuf.resize(payLoadLen);
                memcpy(&outputBuf[0], &dataBuf[0] + sizeof(type), payLoadLen);

            }

            return SL_RESULT_OK;
        }

        sl_result getLidarSampleDuration(float& sampleDurationRes, sl_u16 scanModeID, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            std::vector<sl_u8> reserve(2);
            memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

            std::vector<sl_u8> answer;
            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE, answer, reserve, timeoutInMs);

            if (!ans) return ans;

            if (answer.size() < sizeof(sl_u32)){
                return SL_RESULT_INVALID_DATA;
            }
            const sl_u32 *result = reinterpret_cast<const sl_u32*>(&answer[0]);
            sampleDurationRes = (float)(*result >> 8);
            return ans;
        }

        sl_result getMaxDistance(float &maxDistance, sl_u16 scanModeID, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            std::vector<sl_u8> reserve(2);
            memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

            std::vector<sl_u8> answer;
            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_MAX_DISTANCE, answer, reserve, timeoutInMs);
            if (!ans) return ans;
 
            if (answer.size() < sizeof(sl_u32)){
                return SL_RESULT_INVALID_DATA;
            }
            const sl_u32 *result = reinterpret_cast<const sl_u32*>(&answer[0]);
            maxDistance = (float)(*result >> 8);
            return ans;
        }

        sl_result getScanModeAnsType(sl_u8 &ansType, sl_u16 scanModeID, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            std::vector<sl_u8> reserve(2);
            memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

            std::vector<sl_u8> answer;
            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_ANS_TYPE, answer, reserve, timeoutInMs);
            if (!ans) return ans;

            if (answer.size() < sizeof(sl_u8)){
                return SL_RESULT_INVALID_DATA;
            }
            const sl_u8 *result = reinterpret_cast<const _u8*>(&answer[0]);
            ansType = *result;
            return ans;
        
        }

        sl_result getScanModeName(char* modeName, sl_u16 scanModeID, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            std::vector<sl_u8> reserve(2);
            memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

            std::vector<sl_u8> answer;
            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_NAME, answer, reserve, timeoutInMs);
            if (!ans) return ans;
            int len = answer.size();
            if (0 == len) return SL_RESULT_INVALID_DATA;
            memcpy(modeName, &answer[0], len);
            return ans;
        }

        sl_result negotiateSerialBaudRate(sl_u32 requiredBaudRate, sl_u32 * baudRateDetected)
        {
            // ask the LIDAR to stop working first...
            stop();
            _channel->flush();

            // wait for a while
            delay(10);
            _channel->clearReadCache();

            // sending magic byte to let the target LIDAR start baudrate measurement
            // More than 100 bytes per second datarate is required to trigger the measurements
            {
                

                sl_u8 magicByteSeq[16];

                memset(magicByteSeq, SL_LIDAR_AUTOBAUD_MAGICBYTE, sizeof(magicByteSeq));

                sl_u64 startTS = getms();

                while (getms() - startTS < 1500 ) //lasting for 1.5sec
                {
                    if (_channel->write(magicByteSeq, sizeof(magicByteSeq)) < 0)
                    {
                        return SL_RESULT_OPERATION_FAIL;
                    }
                    
                    size_t dataCountGot;
                    if (_channel->waitForData(1, 1, &dataCountGot)) {
                        //got reply, stop
                        break;
                    }
                }
            }

            // getback the bps measured
            _u32 bpsDetected = 0;
            size_t dataCountGot;
            if (_channel->waitForData(4, 500, &dataCountGot)) {
                //got reply, stop
                _channel->read(&bpsDetected, 4);
                if (baudRateDetected) *baudRateDetected = bpsDetected;


                // send a confirmation to the LIDAR, otherwise, the previous baudrate will be reverted back
                sl_lidar_payload_new_bps_confirmation_t confirmation;
                confirmation.flag = 0x5F5F;
                confirmation.required_bps = requiredBaudRate;
                confirmation.param = 0;
                if (SL_IS_FAIL(_sendCommand(RPLIDAR_CMD_NEW_BAUDRATE_CONFIRM, &confirmation, sizeof(confirmation))))
                    return RESULT_OPERATION_FAIL;


                return RESULT_OK;
            } 
            
            return RESULT_OPERATION_TIMEOUT;
        }

    private:
        
        sl_result  _sendCommand(sl_u16 cmd, const void * payload = NULL, size_t payloadsize = 0 )
        {
            sl_u8 pkt_header[10];
            sl_u8 checksum = 0;

            std::vector<sl_u8> cmd_packet;
            cmd_packet.clear();

            if (payloadsize && payload) {
                cmd |= SL_LIDAR_CMDFLAG_HAS_PAYLOAD;
            }
			_channel->flush();
            cmd_packet.push_back(SL_LIDAR_CMD_SYNC_BYTE);
            cmd_packet.push_back(cmd);
			
            if (cmd & SL_LIDAR_CMDFLAG_HAS_PAYLOAD) {
                std::vector<sl_u8> payloadMsg;
                checksum ^= SL_LIDAR_CMD_SYNC_BYTE;
                checksum ^= cmd;
                checksum ^= (payloadsize & 0xFF);

                // send size
                sl_u8 sizebyte = payloadsize;
                cmd_packet.push_back(sizebyte);
                // calc checksum
                for (size_t pos = 0; pos < payloadsize; ++pos) {
                    checksum ^= ((sl_u8 *)payload)[pos];
                    cmd_packet.push_back(((sl_u8 *)payload)[pos]);
                }
                cmd_packet.push_back(checksum);
  
            }
            sl_u8 packet[1024];
            for (int pos = 0; pos < cmd_packet.size(); pos++) {
                packet[pos] = cmd_packet[pos];
            }
            _channel->write(packet, cmd_packet.size());
            delay(1);
            return SL_RESULT_OK;
        }

        sl_result _waitResponseHeader(sl_lidar_ans_header_t * header, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            int  recvPos = 0;
            sl_u32 startTs = getms();
            sl_u8  recvBuffer[sizeof(sl_lidar_ans_header_t)];
            sl_u8  *headerBuffer = reinterpret_cast<sl_u8 *>(header);
            sl_u32 waitTime;

            while ((waitTime = getms() - startTs) <= timeout) {
                size_t remainSize = sizeof(sl_lidar_ans_header_t) - recvPos;
                size_t recvSize;

                bool ans = _channel->waitForData(remainSize, timeout - waitTime, &recvSize);
                if (!ans) return SL_RESULT_OPERATION_TIMEOUT;

                if (recvSize > remainSize) recvSize = remainSize;

                recvSize = _channel->read(recvBuffer, recvSize);

                for (size_t pos = 0; pos < recvSize; ++pos) {
                    sl_u8 currentByte = recvBuffer[pos];
                    switch (recvPos) {
                    case 0:
                        if (currentByte != SL_LIDAR_ANS_SYNC_BYTE1) {
                            continue;
                        }

                        break;
                    case 1:
                        if (currentByte != SL_LIDAR_ANS_SYNC_BYTE2) {
                            recvPos = 0;
                            continue;
                        }
                        break;
                    }
                    headerBuffer[recvPos++] = currentByte;

                    if (recvPos == sizeof(sl_lidar_ans_header_t)) {
                        return SL_RESULT_OK;
                    }
                }
            }

            return SL_RESULT_OPERATION_TIMEOUT;
        }

        template <typename T>
        sl_result _waitResponse(T &payload ,sl_u8 ansType, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            sl_lidar_ans_header_t response_header;
            Result<nullptr_t> ans = SL_RESULT_OK;
            //delay(100);
            ans = _waitResponseHeader(&response_header, timeout);
            if (!ans) 
                return ans;
            // verify whether we got a correct header
            if (response_header.type != ansType) {
                return SL_RESULT_INVALID_DATA;
            }
           // delay(50);
            sl_u32 header_size = (response_header.size_q30_subtype & SL_LIDAR_ANS_HEADER_SIZE_MASK);
            if (header_size < sizeof(T)) {
                return SL_RESULT_INVALID_DATA;
            }
            if (!_channel->waitForData(header_size, timeout)) {
                return SL_RESULT_OPERATION_TIMEOUT;
            }
            _channel->read(reinterpret_cast<sl_u8 *>(&payload), sizeof(T));
            return SL_RESULT_OK;
        }

        void _disableDataGrabbing()
        {
            //_clearRxDataCache();
            _isScanning = false;
            _cachethread.join();
        }
        
#define  MAX_SCAN_NODES  (8192)
        sl_result _waitNode(sl_lidar_response_measurement_node_t * node, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            int  recvPos = 0;
            sl_u32 startTs = getms();
            sl_u8  recvBuffer[sizeof(sl_lidar_response_measurement_node_t)];
            sl_u8 *nodeBuffer = (sl_u8*)node;
            sl_u32 waitTime;

            while ((waitTime = getms() - startTs) <= timeout) {
                size_t remainSize = sizeof(sl_lidar_response_measurement_node_t) - recvPos;
                size_t recvSize;

                bool ans = _channel->waitForData(remainSize, timeout - waitTime, &recvSize);
                if (!ans) return SL_RESULT_OPERATION_FAIL;

                if (recvSize > remainSize) recvSize = remainSize;

                recvSize = _channel->read(recvBuffer, recvSize);

                for (size_t pos = 0; pos < recvSize; ++pos) {
                    sl_u8 currentByte = recvBuffer[pos];
                    switch (recvPos) {
                    case 0: // expect the sync bit and its reverse in this byte
                    {
                        sl_u8 tmp = (currentByte >> 1);
                        if ((tmp ^ currentByte) & 0x1) {
                            // pass
                        }
                        else {
                            continue;
                        }

                    }
                    break;
                    case 1: // expect the highest bit to be 1
                    {
                        if (currentByte & SL_LIDAR_RESP_MEASUREMENT_CHECKBIT) {
                            // pass
                        }
                        else {
                            recvPos = 0;
                            continue;
                        }
                    }
                    break;
                    }
                    nodeBuffer[recvPos++] = currentByte;

                    if (recvPos == sizeof(sl_lidar_response_measurement_node_t)) {
                        return SL_RESULT_OK;
                    }
                }
            }

            return SL_RESULT_OPERATION_TIMEOUT;
        }

        sl_result _waitScanData(sl_lidar_response_measurement_node_t * nodebuffer, size_t & count, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            if (!_isConnected) {
                count = 0;
                return SL_RESULT_OPERATION_FAIL;
            }

            size_t   recvNodeCount = 0;
            sl_u32     startTs = getms();
            sl_u32     waitTime;
            Result<nullptr_t> ans = SL_RESULT_OK;

            while ((waitTime = getms() - startTs) <= timeout && recvNodeCount < count) {
                sl_lidar_response_measurement_node_t node;
                ans = _waitNode(&node, timeout - waitTime);
                if (!ans) return ans;

                nodebuffer[recvNodeCount++] = node;

                if (recvNodeCount == count) return SL_RESULT_OK;
            }
            count = recvNodeCount;
            return SL_RESULT_OPERATION_TIMEOUT;
        }

        sl_result _cacheScanData()
        {

            sl_lidar_response_measurement_node_t      local_buf[256];
            size_t                                   count = 256;
            sl_lidar_response_measurement_node_hq_t   local_scan[MAX_SCAN_NODES];
            size_t                                   scan_count = 0;
            Result<nullptr_t>                        ans = SL_RESULT_OK;
            memset(local_scan, 0, sizeof(local_scan));

            _waitScanData(local_buf, count); // // always discard the first data since it may be incomplete

            while (_isScanning) {
                ans = _waitScanData(local_buf, count);
   
                if (!ans) {
                    if ((sl_result)ans != SL_RESULT_OPERATION_TIMEOUT) {
                        _isScanning = false;
                        return SL_RESULT_OPERATION_FAIL;
                    }
                }

                for (size_t pos = 0; pos < count; ++pos) {
                    if (local_buf[pos].sync_quality & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT) {
                        // only publish the data when it contains a full 360 degree scan 

                        if ((local_scan[0].flag & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                            _lock.lock();
                            memcpy(_cached_scan_node_hq_buf, local_scan, scan_count * sizeof(sl_lidar_response_measurement_node_hq_t));
                            _cached_scan_node_hq_count = scan_count;
                            _dataEvt.set();
                            _lock.unlock();
                        }
                        scan_count = 0;
                    }

                    sl_lidar_response_measurement_node_hq_t nodeHq;
                    convert(local_buf[pos], nodeHq);
                    local_scan[scan_count++] = nodeHq;
                    if (scan_count == _countof(local_scan)) scan_count -= 1; // prevent overflow

                    //for interval retrieve
                    {
                        rp::hal::AutoLocker l(_lock);
                        _cached_scan_node_hq_buf_for_interval_retrieve[_cached_scan_node_hq_count_for_interval_retrieve++] = nodeHq;
                        if (_cached_scan_node_hq_count_for_interval_retrieve == _countof(_cached_scan_node_hq_buf_for_interval_retrieve)) _cached_scan_node_hq_count_for_interval_retrieve -= 1; // prevent overflow
                    }
                }
            }
            _isScanning = false;
            return SL_RESULT_OK;
        }

        void _ultraCapsuleToNormal(const sl_lidar_response_ultra_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
        {
            nodeCount = 0;
            if (_is_previous_capsuledataRdy) {
                int diffAngle_q8;
                int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF) << 2);
                int prevStartAngle_q8 = ((_cached_previous_ultracapsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

                diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
                if (prevStartAngle_q8 > currentStartAngle_q8) {
                    diffAngle_q8 += (360 << 8);
                }

                int angleInc_q16 = (diffAngle_q8 << 3) / 3;
                int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
                for (size_t pos = 0; pos < _countof(_cached_previous_ultracapsuledata.ultra_cabins); ++pos) {
                    int dist_q2[3];
                    int angle_q6[3];
                    int syncBit[3];


                    sl_u32 combined_x3 = _cached_previous_ultracapsuledata.ultra_cabins[pos].combined_x3;

                    // unpack ...
                    int dist_major = (combined_x3 & 0xFFF);

                    // signed partical integer, using the magic shift here
                    // DO NOT TOUCH

                    int dist_predict1 = (((int)(combined_x3 << 10)) >> 22);
                    int dist_predict2 = (((int)combined_x3) >> 22);

                    int dist_major2;

                    sl_u32 scalelvl1, scalelvl2;

                    // prefetch next ...
                    if (pos == _countof(_cached_previous_ultracapsuledata.ultra_cabins) - 1) {
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
                    }
                    else {
                        dist_predict1 = (dist_predict1 << scalelvl1);
                        dist_q2[1] = (dist_predict1 + dist_base1) << 2;

                    }

                    if ((dist_predict2 == 0xFFFFFE00) || (dist_predict2 == 0x1FF)) {
                        dist_q2[2] = 0;
                    }
                    else {
                        dist_predict2 = (dist_predict2 << scalelvl2);
                        dist_q2[2] = (dist_predict2 + dist_base2) << 2;
                    }


                    for (int cpos = 0; cpos < 3; ++cpos) {
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

                        sl_lidar_response_measurement_node_hq_t node;

                        node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                        node.quality = dist_q2[cpos] ? (0x2F << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                        node.angle_z_q14 = sl_u16((angle_q6[cpos] << 8) / 90);
                        node.dist_mm_q2 = dist_q2[cpos];

                        nodebuffer[nodeCount++] = node;
                    }

                }
            }

            _cached_previous_ultracapsuledata = capsule;
            _is_previous_capsuledataRdy = true;
        }

        sl_result _waitCapsuledNode(sl_lidar_response_capsule_measurement_nodes_t & node, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            int  recvPos = 0;
            sl_u32 startTs = getms();
            sl_u8  recvBuffer[sizeof(sl_lidar_response_capsule_measurement_nodes_t)];
            sl_u8 *nodeBuffer = (sl_u8*)&node;
            sl_u32 waitTime;
            while ((waitTime = getms() - startTs) <= timeout) {
                size_t remainSize = sizeof(sl_lidar_response_capsule_measurement_nodes_t) - recvPos;
                size_t recvSize;
                bool ans = _channel->waitForData(remainSize, timeout - waitTime, &recvSize);
                if (!ans) return SL_RESULT_OPERATION_TIMEOUT;

                if (recvSize > remainSize) recvSize = remainSize;
                recvSize = _channel->read(recvBuffer, recvSize);

                for (size_t pos = 0; pos < recvSize; ++pos) {
                    sl_u8 currentByte = recvBuffer[pos];

                    switch (recvPos) {
                    case 0: // expect the sync bit 1
                    {
                        sl_u8 tmp = (currentByte >> 4);
                        if (tmp == SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
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
                        sl_u8 tmp = (currentByte >> 4);
                        if (tmp == SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
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
                    if (recvPos == sizeof(sl_lidar_response_capsule_measurement_nodes_t)) {
                        // calc the checksum ...
                        sl_u8 checksum = 0;
                        sl_u8 recvChecksum = ((node.s_checksum_1 & 0xF) | (node.s_checksum_2 << 4));
                        for (size_t cpos = offsetof(sl_lidar_response_capsule_measurement_nodes_t, start_angle_sync_q6);
                            cpos < sizeof(sl_lidar_response_capsule_measurement_nodes_t); ++cpos)
                        {
                            checksum ^= nodeBuffer[cpos];
                        }
                        if (recvChecksum == checksum) {
                            // only consider vaild if the checksum matches...
                            if (node.start_angle_sync_q6 & SL_LIDAR_RESP_MEASUREMENT_EXP_SYNCBIT) {
                                // this is the first capsule frame in logic, discard the previous cached data...
                                _scan_node_synced = false;
                                _is_previous_capsuledataRdy = false;
                                return SL_RESULT_OK;
                            }
                            return SL_RESULT_OK;
                        }
                        _is_previous_capsuledataRdy = false;
                        return SL_RESULT_INVALID_DATA;
                    }
                }
            }
            _is_previous_capsuledataRdy = false;
            return SL_RESULT_OPERATION_TIMEOUT;
        }
        void _capsuleToNormal(const sl_lidar_response_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
        {
            nodeCount = 0;
            if (_is_previous_capsuledataRdy) {
                int diffAngle_q8;
                int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF) << 2);
                int prevStartAngle_q8 = ((_cached_previous_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

                diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
                if (prevStartAngle_q8 > currentStartAngle_q8) {
                    diffAngle_q8 += (360 << 8);
                }

                int angleInc_q16 = (diffAngle_q8 << 3);
                int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
                for (size_t pos = 0; pos < _countof(_cached_previous_capsuledata.cabins); ++pos) {
                    int dist_q2[2];
                    int angle_q6[2];
                    int syncBit[2];

                    dist_q2[0] = (_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0xFFFC);
                    dist_q2[1] = (_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0xFFFC);

                    int angle_offset1_q3 = ((_cached_previous_capsuledata.cabins[pos].offset_angles_q3 & 0xF) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0x3) << 4));
                    int angle_offset2_q3 = ((_cached_previous_capsuledata.cabins[pos].offset_angles_q3 >> 4) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0x3) << 4));

                    angle_q6[0] = ((currentAngle_raw_q16 - (angle_offset1_q3 << 13)) >> 10);
                    syncBit[0] = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;
                    currentAngle_raw_q16 += angleInc_q16;


                    angle_q6[1] = ((currentAngle_raw_q16 - (angle_offset2_q3 << 13)) >> 10);
                    syncBit[1] = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;
                    currentAngle_raw_q16 += angleInc_q16;

                    for (int cpos = 0; cpos < 2; ++cpos) {

                        if (angle_q6[cpos] < 0) angle_q6[cpos] += (360 << 6);
                        if (angle_q6[cpos] >= (360 << 6)) angle_q6[cpos] -= (360 << 6);

                        sl_lidar_response_measurement_node_hq_t node;

                        node.angle_z_q14 = sl_u16((angle_q6[cpos] << 8) / 90);
                        node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                        node.quality = dist_q2[cpos] ? (0x2f << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                        node.dist_mm_q2 = dist_q2[cpos];

                        nodebuffer[nodeCount++] = node;
                    }

                }
            }

            _cached_previous_capsuledata = capsule;
            _is_previous_capsuledataRdy = true;
        }

        void _dense_capsuleToNormal(const sl_lidar_response_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
        {
            static int lastNodeSyncBit = 0;
            const sl_lidar_response_dense_capsule_measurement_nodes_t *dense_capsule = reinterpret_cast<const sl_lidar_response_dense_capsule_measurement_nodes_t*>(&capsule);
            nodeCount = 0;
            if (_is_previous_capsuledataRdy) {
                int diffAngle_q8;
                int currentStartAngle_q8 = ((dense_capsule->start_angle_sync_q6 & 0x7FFF) << 2);
                int prevStartAngle_q8 = ((_cached_previous_dense_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

                diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
                if (prevStartAngle_q8 > currentStartAngle_q8) {
                    diffAngle_q8 += (360 << 8);
                }

                int angleInc_q16 = (diffAngle_q8 << 8) / 40;
                int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
                for (size_t pos = 0; pos < _countof(_cached_previous_dense_capsuledata.cabins); ++pos) {
                    int dist_q2;
                    int angle_q6;
                    int syncBit;
                    const int dist = static_cast<const int>(_cached_previous_dense_capsuledata.cabins[pos].distance);
                    dist_q2 = dist << 2;
                    angle_q6 = (currentAngle_raw_q16 >> 10);

                    syncBit = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < (angleInc_q16<<1)) ? 1 : 0;
                    syncBit = (syncBit^ lastNodeSyncBit)&syncBit;//Ensure that syncBit is exactly detected
                    if (syncBit) {
                        _scan_node_synced = true;
                    }

                    currentAngle_raw_q16 += angleInc_q16;

                    if (angle_q6 < 0) angle_q6 += (360 << 6);
                    if (angle_q6 >= (360 << 6)) angle_q6 -= (360 << 6);

                    
                    sl_lidar_response_measurement_node_hq_t node;

                    node.angle_z_q14 = sl_u16((angle_q6 << 8) / 90);
                    node.flag = (syncBit | ((!syncBit) << 1));
                    node.quality = dist_q2 ? (0x2f << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                    node.dist_mm_q2 = dist_q2;
                    if(_scan_node_synced)
                        nodebuffer[nodeCount++] = node;
                    lastNodeSyncBit = syncBit;
                }
            }
            else {
                _scan_node_synced = false;
            }

            _cached_previous_dense_capsuledata = *dense_capsule;
            _is_previous_capsuledataRdy = true;
        }

        sl_result _cacheCapsuledScanData()
        {
            sl_lidar_response_capsule_measurement_nodes_t    capsule_node;
            sl_lidar_response_measurement_node_hq_t          local_buf[256];
            size_t                                           count = 256;
            sl_lidar_response_measurement_node_hq_t          local_scan[MAX_SCAN_NODES];
            size_t                                           scan_count = 0;
            Result<nullptr_t>                                ans = SL_RESULT_OK;  
            memset(local_scan, 0, sizeof(local_scan));

            _waitCapsuledNode(capsule_node); // // always discard the first data since it may be incomplete

            while (_isScanning) {
                ans = _waitCapsuledNode(capsule_node);
                if (!ans) {
                    if ((sl_result)ans != SL_RESULT_OPERATION_TIMEOUT && (sl_result)ans != SL_RESULT_INVALID_DATA) {
                        _isScanning = false;
                        return SL_RESULT_OPERATION_FAIL;
                    }
                    else {
                        // current data is invalid, do not use it.
                        continue;
                    }
                }
                switch (_cached_capsule_flag) {
                case NORMAL_CAPSULE:
                    _capsuleToNormal(capsule_node, local_buf, count);
                    break;
                case DENSE_CAPSULE:
                    _dense_capsuleToNormal(capsule_node, local_buf, count);
                    break;
                }
                //

                for (size_t pos = 0; pos < count; ++pos) {
                    if (local_buf[pos].flag & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT) {
                        // only publish the data when it contains a full 360 degree scan 

                        if ((local_scan[0].flag & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                            _lock.lock();
                            memcpy(_cached_scan_node_hq_buf, local_scan, scan_count * sizeof(sl_lidar_response_measurement_node_hq_t));
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
            _isScanning = false;

            return SL_RESULT_OK;
        }

        sl_result _waitHqNode(sl_lidar_response_hq_capsule_measurement_nodes_t & node, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            if (!_isConnected) {
                return SL_RESULT_OPERATION_FAIL;
            }

            int  recvPos = 0;
            sl_u32 startTs = getms();
            sl_u8  recvBuffer[sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t)];
            sl_u8 *nodeBuffer = (sl_u8*)&node;
            sl_u32 waitTime;

            while ((waitTime = getms() - startTs) <= timeout) {
                size_t remainSize = sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t) - recvPos;
                size_t recvSize;

                bool ans = _channel->waitForData(remainSize, timeout - waitTime, &recvSize);
                if (!ans){
                    return SL_RESULT_OPERATION_TIMEOUT;
                }
                if (recvSize > remainSize) recvSize = remainSize;

                recvSize = _channel->read(recvBuffer, recvSize);

                for (size_t pos = 0; pos < recvSize; ++pos) {
                    sl_u8 currentByte = recvBuffer[pos];
                    switch (recvPos) {
                    case 0: // expect the sync byte
                    {
                        sl_u8 tmp = (currentByte);
                        if (tmp == SL_LIDAR_RESP_MEASUREMENT_HQ_SYNC) {
                            // pass
                        }
                        else {
                            recvPos = 0;
                            _is_previous_HqdataRdy = false;
                            continue;
                        }
                    }
                    break;
                    case sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t) - 1 - 4:
                    {

                    }
                    break;
                    case sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t) - 1:
                    {

                    }
                    break;
                    }
                    nodeBuffer[recvPos++] = currentByte;
                    if (recvPos == sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t)) {
                        sl_u32 crcCalc2 = crc32::getResult(nodeBuffer, sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t) - 4);

                        if (crcCalc2 == node.crc32) {
                            _is_previous_HqdataRdy = true;
                            return SL_RESULT_OK;
                        }
                        else {
                            _is_previous_HqdataRdy = false;
                            return SL_RESULT_INVALID_DATA;
                        }

                    }
                }
            }
            _is_previous_HqdataRdy = false;
            return SL_RESULT_OPERATION_TIMEOUT;
        }

        void _HqToNormal(const sl_lidar_response_hq_capsule_measurement_nodes_t & node_hq, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
        {
            nodeCount = 0;
            if (_is_previous_HqdataRdy) {
                for (size_t pos = 0; pos < _countof(_cached_previous_Hqdata.node_hq); ++pos) {
                    nodebuffer[nodeCount++] = node_hq.node_hq[pos];
                }
            }
            _cached_previous_Hqdata = node_hq;
            _is_previous_HqdataRdy = true;

        }

        sl_result _cacheHqScanData()
        {
            sl_lidar_response_hq_capsule_measurement_nodes_t    hq_node;
            sl_lidar_response_measurement_node_hq_t   local_buf[256];
            size_t                                   count = 256;
            sl_lidar_response_measurement_node_hq_t   local_scan[MAX_SCAN_NODES];
            size_t                                   scan_count = 0;
            Result<nullptr_t>                             ans = SL_RESULT_OK;
            memset(local_scan, 0, sizeof(local_scan));
            _waitHqNode(hq_node);
            while (_isScanning) {
                ans = _waitHqNode(hq_node);
                if (!ans) {
                    if ((sl_result)ans != SL_RESULT_OPERATION_TIMEOUT && (sl_result)ans != SL_RESULT_INVALID_DATA) {
                        _isScanning = false;
                        return SL_RESULT_OPERATION_FAIL;
                    }
                    else {
                        // current data is invalid, do not use it.
                        continue;
                    }
                }

                _HqToNormal(hq_node, local_buf, count);
                for (size_t pos = 0; pos < count; ++pos){
                    if (local_buf[pos].flag & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT){
                        // only publish the data when it contains a full 360 degree scan 
                        if ((local_scan[0].flag & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                            _lock.lock();
                            memcpy(_cached_scan_node_hq_buf, local_scan, scan_count * sizeof(sl_lidar_response_measurement_node_hq_t));
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
            return SL_RESULT_OK;
        }

        sl_result _waitUltraCapsuledNode(sl_lidar_response_ultra_capsule_measurement_nodes_t & node, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            if (!_isConnected) {
                return SL_RESULT_OPERATION_FAIL;
            }

            int  recvPos = 0;
            sl_u32 startTs = getms();
            sl_u8  recvBuffer[sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)];
            sl_u8 *nodeBuffer = (sl_u8*)&node;
            sl_u32 waitTime;

            while ((waitTime = getms() - startTs) <= timeout) {
                size_t remainSize = sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t) - recvPos;
                size_t recvSize;

                bool ans = _channel->waitForData(remainSize, timeout - waitTime, &recvSize);
                if (!ans) {
                    return SL_RESULT_OPERATION_TIMEOUT;
                }
                if (recvSize > remainSize) recvSize = remainSize;

                recvSize = _channel->read(recvBuffer, recvSize);

                for (size_t pos = 0; pos < recvSize; ++pos) {
                    sl_u8 currentByte = recvBuffer[pos];
                    switch (recvPos) {
                    case 0: // expect the sync bit 1
                    {
                        sl_u8 tmp = (currentByte >> 4);
                        if (tmp == SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
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
                        sl_u8 tmp = (currentByte >> 4);
                        if (tmp == SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
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
                    if (recvPos == sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)) {
                        // calc the checksum ...
                        sl_u8 checksum = 0;
                        sl_u8 recvChecksum = ((node.s_checksum_1 & 0xF) | (node.s_checksum_2 << 4));

                        for (size_t cpos = offsetof(sl_lidar_response_ultra_capsule_measurement_nodes_t, start_angle_sync_q6);
                            cpos < sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t); ++cpos) 
                        {
                            checksum ^= nodeBuffer[cpos];
                        }

                        if (recvChecksum == checksum) {
                            // only consider vaild if the checksum matches...
                            if (node.start_angle_sync_q6 & SL_LIDAR_RESP_MEASUREMENT_EXP_SYNCBIT) {
                                // this is the first capsule frame in logic, discard the previous cached data...
                                _is_previous_capsuledataRdy = false;
                                return SL_RESULT_OK;
                            }
                            return SL_RESULT_OK;
                        }
                        _is_previous_capsuledataRdy = false;
                        return SL_RESULT_INVALID_DATA;
                    }
                }
            }
            _is_previous_capsuledataRdy = false;
            return SL_RESULT_OPERATION_TIMEOUT;
        }

        sl_result _cacheUltraCapsuledScanData()
        {
            sl_lidar_response_ultra_capsule_measurement_nodes_t    ultra_capsule_node;
            sl_lidar_response_measurement_node_hq_t   local_buf[256];
            size_t                                   count = 256;
            sl_lidar_response_measurement_node_hq_t   local_scan[MAX_SCAN_NODES];
            size_t                                   scan_count = 0;
            Result<nullptr_t>                        ans = SL_RESULT_OK;
            memset(local_scan, 0, sizeof(local_scan));

            _waitUltraCapsuledNode(ultra_capsule_node);

            while (_isScanning) {
                ans = _waitUltraCapsuledNode(ultra_capsule_node);
                if (!ans) {
                    if ((sl_result)ans != SL_RESULT_OPERATION_TIMEOUT && (sl_result)ans != SL_RESULT_INVALID_DATA) {
                        _isScanning = false;
                        return SL_RESULT_OPERATION_FAIL;
                    }
                    else {
                        // current data is invalid, do not use it.
                        continue;
                    }
                }

                _ultraCapsuleToNormal(ultra_capsule_node, local_buf, count);

                for (size_t pos = 0; pos < count; ++pos) {
                    if (local_buf[pos].flag & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT) {
                        // only publish the data when it contains a full 360 degree scan 

                        if ((local_scan[0].flag & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                            _lock.lock();
                            memcpy(_cached_scan_node_hq_buf, local_scan, scan_count * sizeof(sl_lidar_response_measurement_node_hq_t));
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

            _isScanning = false;

            return SL_RESULT_OK;
        }
        sl_result _clearRxDataCache()
        {
            if (!isConnected())
                return SL_RESULT_OPERATION_FAIL;
            _channel->flush();
            return SL_RESULT_OK;
        }

    private:
        IChannel *_channel;
        bool _isConnected;
        bool _isScanning;
        MotorCtrlSupport        _isSupportingMotorCtrl;
        rp::hal::Locker         _lock;
        rp::hal::Event          _dataEvt;
        rp::hal::Thread         _cachethread;
        sl_u16                  _cached_sampleduration_std;
        sl_u16                  _cached_sampleduration_express;
        bool                    _scan_node_synced;

        sl_lidar_response_measurement_node_hq_t   _cached_scan_node_hq_buf[8192];
        size_t                                   _cached_scan_node_hq_count;
        sl_u8                                    _cached_capsule_flag;

        sl_lidar_response_measurement_node_hq_t   _cached_scan_node_hq_buf_for_interval_retrieve[8192];
        size_t                                   _cached_scan_node_hq_count_for_interval_retrieve;

        sl_lidar_response_capsule_measurement_nodes_t       _cached_previous_capsuledata;
        sl_lidar_response_dense_capsule_measurement_nodes_t _cached_previous_dense_capsuledata;
        sl_lidar_response_ultra_capsule_measurement_nodes_t _cached_previous_ultracapsuledata;
        sl_lidar_response_hq_capsule_measurement_nodes_t _cached_previous_Hqdata;
        bool                                         _is_previous_capsuledataRdy;
        bool                                         _is_previous_HqdataRdy;
    };

    Result<ILidarDriver*> createLidarDriver()
    {
        return new SlamtecLidarDriver();
    }
}