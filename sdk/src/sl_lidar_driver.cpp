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
#include "hal/waiter.h"
#include "hal/byteorder.h"
#include "sl_lidar_driver.h"
#include "sl_crc.h" 
#include <algorithm>
#include <memory>
#include <atomic>
#include <deque>

#include "dataunpacker/dataunpacker.h"
#include "sl_async_transceiver.h"
#include "sl_lidarprotocol_codec.h"



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
        for (i = count - 1; i < count; i--) {
            // To avoid array overruns, use the i < count condition
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

    template<typename T>
    class RawSampleNodeHolder
    {
    public:
        RawSampleNodeHolder(size_t maxcount = 8192)
            : _max_count(maxcount)
        {
           
        }
        void clear()
        {
            rp::hal::AutoLocker l(_locker);
            _data_waiter.set(false);
            _data_queue.clear();
        }

        void pushNode(_u64 timestamp_uS, const T* node)
        {
            rp::hal::AutoLocker l(_locker);
            _data_queue.push_back(*node);
            if (_data_queue.size() > _max_count) {
                _data_queue.pop_front();
            }
            _data_waiter.set();
        }

        size_t waitAndFetch(T* node, size_t maxcount, _u32 timeout)
        {
            if (_data_waiter.wait(timeout) == rp::hal::Event::EVENT_OK)
            {
                rp::hal::AutoLocker l(_locker);

                size_t copiedCount = 0;

                while (maxcount--) {
                    node[copiedCount++] = _data_queue.front();
                    _data_queue.pop_front();
                }

                return copiedCount;
            }
            return 0;
        }
    protected:
        size_t          _max_count;
        rp::hal::Locker _locker;
        rp::hal::Event  _data_waiter;
        std::deque<T>   _data_queue;
        
    };

    template<typename T>
    class ScanDataHolder
    {
    public:
        ScanDataHolder(size_t maxcount = 8192) 
            : _scan_node_buffer_size(maxcount)
            , _scan_node_available_id(-1)
            , _new_scan_ready(false)
        {
            _scanbuffer[0].reserve(_scan_node_buffer_size);
            _scanbuffer[1].reserve(_scan_node_buffer_size);

            memset(_scan_begin_timestamp_uS, 0, sizeof(_scan_begin_timestamp_uS));
        }

        size_t getMaxCacheCount() const {
            return _scan_node_buffer_size;
        }


        void reset() {
            rp::hal::AutoLocker l(_locker);
            _scan_node_available_id = -1;
            _new_scan_ready = false;
            _scanbuffer[0].clear();
            _scanbuffer[1].clear();
            _data_waiter.set(false);
            memset(_scan_begin_timestamp_uS, 0, sizeof(_scan_begin_timestamp_uS));
        }

        bool checkNewScanSignalAndReset()
        {
            return _new_scan_ready.exchange(false);
        }

        void pushScanNodeData(_u64 currentSampleTsUs, const T* hqNode)
        {
            rp::hal::AutoLocker l(_locker);

            int  operationBufID = _getOperationBufferID_locked();
            auto operationalBuf = &_scanbuffer[operationBufID];
            
            if (hqNode->flag & RPLIDAR_RESP_HQ_FLAG_SYNCBIT) {
                if (operationalBuf->size()) {
                    operationBufID = _finishCurrentScanAndSwap_locked();
                    operationalBuf = &_scanbuffer[operationBufID];

                    // publish the available scan
                    _new_scan_ready = true;
                    _data_waiter.set();

                }
                
                assert(operationalBuf->size() == 0);

                //store the timestamp info
                _scan_begin_timestamp_uS[operationBufID] = currentSampleTsUs;
            }
            else {
                if (operationalBuf->size() == 0) {
                    //discard the data, do not form partial scan
                    return;
                }
            }

            if (operationalBuf->size() >= _scan_node_buffer_size) {
                //replace the last entry if buffer is full
                operationalBuf->at(operationalBuf->size() - 1) = *hqNode;
            }
            else {
                operationalBuf->push_back(*hqNode);
            }

        }

        void rewindCurrentScanData() {
            rp::hal::AutoLocker l(_locker);
            _getOperationalBuffer_locked().clear();
        }

        std::vector<T>* waitAndLockAvailableScan(_u32 timeout, _u64 * out_timestamp_uS = nullptr)
        {
            if (_data_waiter.wait(timeout) == rp::hal::Event::EVENT_OK)
            {
                _locker.lock();
                assert(_scan_node_available_id >= 0);
                _new_scan_ready = false;
                if (out_timestamp_uS) {
                    *out_timestamp_uS = _scan_begin_timestamp_uS[_scan_node_available_id];
                }
                return &_scanbuffer[_scan_node_available_id];
            }
            else {
                return nullptr;
            }
        }

        void unlockScan(std::vector<T>* scan) {
            if (scan) {
                _locker.unlock();
            }
        }

    protected:
        int _finishCurrentScanAndSwap_locked() {
            _scan_node_available_id = _getOperationBufferID_locked();
            int newOperationalID  =  1 - _scan_node_available_id;

            _scanbuffer[newOperationalID].clear();
            return newOperationalID;
        }

        int _getOperationBufferID_locked() {
            if (_scan_node_available_id < 0) return 0;
            return 1 - _scan_node_available_id;
        }

        std::vector<T>& _getOperationalBuffer_locked()
        {
            return _scanbuffer[_getOperationBufferID_locked()];
        }


        rp::hal::Locker _locker;
        rp::hal::Event  _data_waiter;

        

        _u64   _scan_begin_timestamp_uS[2];
        size_t _scan_node_buffer_size;
        int    _scan_node_available_id;
        std::atomic<bool>   _new_scan_ready;

        std::vector<T> _scanbuffer[2];
    };

    class SlamtecLidarDriver : 
        public ILidarDriver, internal::IProtocolMessageListener, internal::LIDARSampleDataListener
    {
    public:
        enum {
            MAX_SCANNODE_CACHE_COUNT = 8192,
        };

        enum {
            A2A3_LIDAR_MINUM_MAJOR_ID  = 2,
            BUILTIN_MOTORCTL_MINUM_MAJOR_ID = 6,
        };


        enum {
            TOF_C_SERIAL_MINUM_MAJOR_ID = 4,
            TOF_S_SERIAL_MINUM_MAJOR_ID = 6,
            TOF_T_SERIAL_MINUM_MAJOR_ID = 9,
            TOF_M_SERIAL_MINUM_MAJOR_ID = 12,


            NEWDESIGN_MINUM_MAJOR_ID = TOF_C_SERIAL_MINUM_MAJOR_ID,
        };

    public:
        SlamtecLidarDriver()
            : _isConnected(false)
            , _isSupportingMotorCtrl(MotorCtrlSupportNone)
            , _op_locker(true)
            , _scanHolder(MAX_SCANNODE_CACHE_COUNT)
            , _rawSampleNodeHolder(MAX_SCANNODE_CACHE_COUNT)
            , _waiting_packet_type(0)
        {
            _protocolHandler = std::make_shared< internal::RPLidarProtocolCodec>();
            _transeiver = std::make_shared< internal::AsyncTransceiver>(*_protocolHandler);
            _dataunpacker.reset(internal::LIDARSampleDataUnpacker::CreateInstance(*this));

            _protocolHandler->setMessageListener(this);

            memset(&_cached_DevInfo, 0, sizeof(_cached_DevInfo));
        }


        virtual ~SlamtecLidarDriver()
        {
            disconnect();
            _protocolHandler->setMessageListener(nullptr);
        }


        LIDARTechnologyType getLIDARTechnologyType(const sl_lidar_response_device_info_t* devInfo)
        {
            rp::hal::AutoLocker l(_op_locker);
            if (!devInfo) {
                devInfo = &_cached_DevInfo;
            }

            return ParseLIDARTechnologyTypeByModelID(devInfo->model);
        }

        LIDARMajorType getLIDARMajorType(const sl_lidar_response_device_info_t* devInfo)
        {
            rp::hal::AutoLocker l(_op_locker);
            if (!devInfo) {
                devInfo = &_cached_DevInfo;
            }

            return ParseLIDARMajorTypeByModelID(devInfo->model);

        }

        sl_result getModelNameDescriptionString(std::string& out_description, bool fetchAliasName, const sl_lidar_response_device_info_t* devInfo, sl_u32 timeout)
        {
            rp::hal::AutoLocker l(_op_locker);
            u_result ans;
            // fetch alias (commerical) name if asked:
            if (fetchAliasName) {
                std::vector<_u8> replyData;
                ans = getLidarConf(SL_LIDAR_CONF_MODEL_NAME_ALIAS, replyData, nullptr, 0, timeout);
                if (IS_OK(ans) && replyData.size()) {
                    out_description.resize(replyData.size() + 1);
                    memcpy(&out_description[0], &replyData[0], replyData.size());
                    out_description[replyData.size()] = '\0';
                    if (out_description != "") {
                        return SL_RESULT_OK;
                    }
                }
            }


            if (!devInfo) {
                devInfo = &_cached_DevInfo;
            }


            out_description = GetModelNameStringByModelID(devInfo->model);

            return SL_RESULT_OK;
        }

        sl_result connect(IChannel* channel)
        {
            rp::hal::AutoLocker l(_op_locker);
            if (!channel) return SL_RESULT_OPERATION_FAIL;
            if (isConnected()) return SL_RESULT_ALREADY_DONE;

            _rawSampleNodeHolder.clear();

            sl_result ans;
            
       
            ans = (sl_result)_transeiver->openChannelAndBind(channel);

            if (IS_OK(ans)) {
                _isConnected = true;
                // the first dev info local cache will be taken here
                checkMotorCtrlSupport(_isSupportingMotorCtrl, 500);
            }
            
            return ans;
        }

        void disconnect()
        {
            rp::hal::AutoLocker l(_op_locker);
            if (_isConnected) {
                _disableDataGrabbing();

                _transeiver->unbindAndClose();
                _isConnected = false;
            }
        }

        bool isConnected()
        {
            return _isConnected;
        }

        sl_result reset(sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            rp::hal::AutoLocker l(_op_locker);
            // send reset message 
            return (sl_result)_sendCommandWithoutResponse(SL_LIDAR_CMD_RESET);
        }

        sl_result getAllSupportedScanModes(std::vector<LidarScanMode>& outModes, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            rp::hal::AutoLocker l(_op_locker);

            if (!isConnected()) return SL_RESULT_OPERATION_NOT_SUPPORT;

            Result<nullptr_t> ans = SL_RESULT_OK;
            bool confProtocolSupported = false;
            ans = checkSupportConfigCommands(confProtocolSupported, timeoutInMs);
            if (!ans) return SL_RESULT_INVALID_DATA;

            if (confProtocolSupported) {
                // 1. get scan mode count
                sl_u16 modeCount;
                ans = getScanModeCount(modeCount, timeoutInMs);
                if (!ans) return ans;
                // 2. for loop to get all fields of each scan mode
                for (sl_u16 i = 0; i < modeCount; i++) {
                    LidarScanMode scanModeInfoTmp;
                    memset(&scanModeInfoTmp, 0, sizeof(scanModeInfoTmp));
                    scanModeInfoTmp.id = i;
                    ans = getLidarSampleDuration(scanModeInfoTmp.us_per_sample, i, timeoutInMs);
                    if (!ans) return ans;
                    ans = getMaxDistance(scanModeInfoTmp.max_distance, i, timeoutInMs);
                    if (!ans) return ans;
                    ans = getScanModeAnsType(scanModeInfoTmp.ans_type, i, timeoutInMs);
                    if (!ans) return ans;
                    ans = getScanModeName(scanModeInfoTmp.scan_mode, sizeof(scanModeInfoTmp.scan_mode), i, timeoutInMs);
                    if (!ans) return ans;
                    outModes.push_back(scanModeInfoTmp);

                }
                return ans;
            }

            return ans;        
        }

        sl_result getTypicalScanMode(sl_u16& outMode, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            rp::hal::AutoLocker l(_op_locker);
            if (!isConnected()) return SL_RESULT_OPERATION_NOT_SUPPORT;

            Result<nullptr_t> ans = SL_RESULT_OK;
            std::vector<sl_u8> answer;
            bool lidarSupportConfigCmds = false;
            ans = checkSupportConfigCommands(lidarSupportConfigCmds);
            if (!ans) return ans;

            if (lidarSupportConfigCmds) {
                ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_TYPICAL, answer, nullptr, 0, timeoutInMs);
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
            rp::hal::AutoLocker l(_op_locker);
            if (!isConnected()) return SL_RESULT_OPERATION_NOT_SUPPORT;

            Result<nullptr_t> ans = SL_RESULT_OK;
            bool ifSupportLidarConf = false;
            LidarScanMode localMode;

            if (!isConnected()) return SL_RESULT_OPERATION_FAIL;
            stop();

            if (!outUsedScanMode) outUsedScanMode = &localMode;


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


            return startScanNormal_commonpath(force, ifSupportLidarConf , *outUsedScanMode, DEFAULT_TIMEOUT);
        }


        // this path make sure the working mode has always been retrieved
        sl_result startScanNormal_commonpath(bool force, bool ifSupportLidarConf, LidarScanMode& outUsedScanMode, sl_u32 timeout)
        {

            Result<nullptr_t> ans = SL_RESULT_OK;

            if (ifSupportLidarConf) {

                outUsedScanMode.id = SL_LIDAR_CONF_SCAN_COMMAND_STD;
                ans = getLidarSampleDuration(outUsedScanMode.us_per_sample, outUsedScanMode.id);
                if (!ans) return ans;
                ans = getMaxDistance(outUsedScanMode.max_distance, outUsedScanMode.id);
                if (!ans) return ans;
                ans = getScanModeAnsType(outUsedScanMode.ans_type, outUsedScanMode.id);
                if (!ans) return ans;
                ans = getScanModeName(outUsedScanMode.scan_mode, sizeof(outUsedScanMode.scan_mode), outUsedScanMode.id);
                if (!ans) return ans;

            }
            else {
                // a legacy device
                rplidar_response_sample_rate_t sampleRateTmp;
                ans = _getLegacySampleDuration_uS(sampleRateTmp, timeout);

                if (!ans) return SL_RESULT_INVALID_DATA;
                outUsedScanMode.us_per_sample = sampleRateTmp.std_sample_duration_us;
                outUsedScanMode.max_distance = 16;
                outUsedScanMode.ans_type = SL_LIDAR_ANS_TYPE_MEASUREMENT;
                strcpy(outUsedScanMode.scan_mode, "Standard");
            }


            _updateTimingDesc(_cached_DevInfo, outUsedScanMode.us_per_sample);

            startMotor();

            _scanHolder.reset();
            _dataunpacker->enable();

            ans = _sendCommandWithoutResponse(force ? SL_LIDAR_CMD_FORCE_SCAN : SL_LIDAR_CMD_SCAN, nullptr, 0, true);
            if (ans) delay(10); // wait rplidar to handle it
            return ans;
        }


        sl_result startScanNormal(bool force, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            rp::hal::AutoLocker l(_op_locker);
            if (!isConnected()) return SL_RESULT_OPERATION_NOT_SUPPORT;


            LidarScanMode localMode;
            bool ifSupportLidarConf;

            if (!isConnected()) return SL_RESULT_OPERATION_FAIL;
            stop();

            Result<nullptr_t> ans = checkSupportConfigCommands(ifSupportLidarConf);
            if (!ans) return ans;

            return startScanNormal_commonpath(force, ifSupportLidarConf, localMode, timeout);
        }

        sl_result startScanExpress(bool force, sl_u16 scanMode, sl_u32 options = 0, LidarScanMode* outUsedScanMode = nullptr, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            rp::hal::AutoLocker l(_op_locker);
            if (!isConnected()) return SL_RESULT_OPERATION_NOT_SUPPORT;


            Result<nullptr_t> ans = SL_RESULT_OK;
            if (!isConnected()) return SL_RESULT_OPERATION_FAIL;
            stop(); //force the previous operation to stop

            LidarScanMode localMode;

            if (!outUsedScanMode) outUsedScanMode = &localMode;

            bool ifSupportLidarConf = false;
            ans = checkSupportConfigCommands(ifSupportLidarConf);
            if (!ans) return SL_RESULT_INVALID_DATA;


            
            outUsedScanMode->id = scanMode;
            if (ifSupportLidarConf) {
                ans = getLidarSampleDuration(outUsedScanMode->us_per_sample, outUsedScanMode->id);
                if (!ans) return SL_RESULT_INVALID_DATA;

                ans = getMaxDistance(outUsedScanMode->max_distance, outUsedScanMode->id);
                if (!ans) return SL_RESULT_INVALID_DATA;

                ans = getScanModeAnsType(outUsedScanMode->ans_type, outUsedScanMode->id);
                if (!ans) return SL_RESULT_INVALID_DATA;

                ans = getScanModeName(outUsedScanMode->scan_mode, sizeof(outUsedScanMode->scan_mode), outUsedScanMode->id);
                if (!ans) return SL_RESULT_INVALID_DATA;
            }
            else {
                // legacy device support
                if (scanMode != RPLIDAR_CONF_SCAN_COMMAND_STD) {
                    rplidar_response_sample_rate_t sampleRateTmp;
                    ans = _getLegacySampleDuration_uS(sampleRateTmp, timeout);
                    if (!ans) return RESULT_INVALID_DATA;

                    outUsedScanMode->us_per_sample = sampleRateTmp.express_sample_duration_us;
                    outUsedScanMode->max_distance = 16;
                    outUsedScanMode->ans_type = SL_LIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
                    strcpy(outUsedScanMode->scan_mode, "Express");
                }
                else {
                    outUsedScanMode->ans_type = SL_LIDAR_ANS_TYPE_MEASUREMENT;
                }
            }

            if (outUsedScanMode->ans_type == SL_LIDAR_ANS_TYPE_MEASUREMENT)
            {
                // redirect to the correct function...
                return startScanNormal(force, timeout);
            }
            
            _updateTimingDesc(_cached_DevInfo, outUsedScanMode->us_per_sample);
            startMotor();

            _scanHolder.reset();
            _dataunpacker->enable();

            sl_lidar_payload_express_scan_t scanReq;
            memset(&scanReq, 0, sizeof(scanReq));

            if (!ifSupportLidarConf) {
                if (scanMode != SL_LIDAR_CONF_SCAN_COMMAND_STD && scanMode != SL_LIDAR_CONF_SCAN_COMMAND_EXPRESS)
                    scanReq.working_mode = sl_u8(scanMode);
            }
            else
                scanReq.working_mode = sl_u8(scanMode);

            scanReq.working_flags = options;

            ans = _sendCommandWithoutResponse(SL_LIDAR_CMD_EXPRESS_SCAN, &scanReq, sizeof(scanReq), true);
            if (ans) delay(10); // wait rplidar to handle it
            return ans;

        }

        sl_result stop(sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            rp::hal::AutoLocker l(_op_locker);


            u_result ans = SL_RESULT_OK;
            ans = _sendCommandWithoutResponse(SL_LIDAR_CMD_STOP);
            _disableDataGrabbing();

            if (IS_FAIL(ans)) return ans;
            

            delay(100);

            if(_isSupportingMotorCtrl == MotorCtrlSupportPwm)
                setMotorSpeed(0);
  
            return SL_RESULT_OK;
        }

        sl_result grabScanDataHqWithTimeStamp(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t& count, sl_u64& timestamp_uS, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            rp::hal::AutoLocker l(_op_locker);

            if (!nodebuffer)
                return SL_RESULT_INVALID_DATA;

            auto availBuffer = _scanHolder.waitAndLockAvailableScan(timeout, &timestamp_uS);
            if (!availBuffer) return SL_RESULT_OPERATION_TIMEOUT;

            count = std::min<size_t>(count, availBuffer->size());

            std::copy(availBuffer->begin(), availBuffer->begin() + count, nodebuffer);

            _scanHolder.unlockScan(availBuffer);

            return RESULT_OK;
        }

        sl_result grabScanDataHq(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t& count, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            _u64 localTS;
            return grabScanDataHqWithTimeStamp(nodebuffer, count, localTS, timeout);
        }

        sl_result getDeviceInfo(sl_lidar_response_device_info_t& info, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            rp::hal::AutoLocker l(_op_locker);
            if (!isConnected()) return SL_RESULT_OPERATION_NOT_SUPPORT;


            u_result ans;
            internal::message_autoptr_t ans_frame;

            ans = _sendCommandWithResponse(SL_LIDAR_CMD_GET_DEVICE_INFO, SL_LIDAR_ANS_TYPE_DEVINFO, ans_frame, timeout);

            if (IS_FAIL(ans)) return ans;
            if (ans_frame->getPayloadSize() < sizeof(rplidar_response_device_info_t))
            {
                return RESULT_INVALID_DATA;
            }
            info = *(rplidar_response_device_info_t*)ans_frame->getDataBuf();
#ifdef _CPU_ENDIAN_BIG
            info.firmware_version = le16_to_cpu(info.firmware_version);
#endif

            _cached_DevInfo = info;
            return (sl_result)ans;
        }

        sl_result checkMotorCtrlSupport(MotorCtrlSupport & support, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            rp::hal::AutoLocker l(_op_locker);
            if (!isConnected()) return SL_RESULT_OPERATION_NOT_SUPPORT;

            Result<nullptr_t> ans = SL_RESULT_OK;
            support = MotorCtrlSupportNone;
            _disableDataGrabbing();

            {
                sl_lidar_response_device_info_t devInfo;
                ans = getDeviceInfo(devInfo, 500);
                if (!ans) return ans;
                sl_u8 majorId = devInfo.model >> 4;
                if (majorId >= BUILTIN_MOTORCTL_MINUM_MAJOR_ID) {
                        support = MotorCtrlSupportRpm;
                        return ans;
                }
                else if(majorId >= A2A3_LIDAR_MINUM_MAJOR_ID){

                    rp::hal::AutoLocker l(_op_locker);
                    sl_lidar_payload_acc_board_flag_t flag;
                    flag.reserved = 0;
                    internal::message_autoptr_t ans_frame;

                    ans = _sendCommandWithResponse(SL_LIDAR_CMD_GET_ACC_BOARD_FLAG, SL_LIDAR_ANS_TYPE_ACC_BOARD_FLAG, ans_frame, timeout, &flag, sizeof(flag));
                    if (!ans) return ans;

                    if (ans_frame->getPayloadSize() < sizeof(rplidar_response_acc_board_flag_t))
                    {
                        return RESULT_INVALID_DATA;
                    }

                    const sl_lidar_response_acc_board_flag_t* acc_board_flag
                        = reinterpret_cast<const sl_lidar_response_acc_board_flag_t*>(ans_frame->getDataBuf());

                    if (acc_board_flag->support_flag & SL_LIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK) {
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
            rp::hal::AutoLocker l(_op_locker);
            if (!isConnected()) return SL_RESULT_OPERATION_NOT_SUPPORT;

			sl_result ans = setLidarConf(SL_LIDAR_CONF_LIDAR_STATIC_IP_ADDR, &conf, sizeof(sl_lidar_ip_conf_t), timeout);
			return ans;
		}

        sl_result getLidarIpConf(sl_lidar_ip_conf_t& conf, sl_u32 timeout)
        {
            rp::hal::AutoLocker l(_op_locker);
            if (!isConnected()) return SL_RESULT_OPERATION_NOT_SUPPORT;


            Result<nullptr_t> ans = SL_RESULT_OK;
            std::vector<sl_u8> reserve(2); //keep backward compatibility

            std::vector<sl_u8> answer;
            ans = getLidarConf(SL_LIDAR_CONF_LIDAR_STATIC_IP_ADDR, answer, &reserve[0], 2, timeout);
            size_t len = answer.size();
            if (0 == len) return SL_RESULT_INVALID_DATA;
            memcpy(&conf, &answer[0], len);
            return ans;
        }
       
        sl_result getHealth(sl_lidar_response_device_health_t& health, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            rp::hal::AutoLocker l(_op_locker);
            if (!isConnected()) return SL_RESULT_OPERATION_NOT_SUPPORT;


            u_result ans;
            internal::message_autoptr_t ans_frame;

            ans = _sendCommandWithResponse(SL_LIDAR_CMD_GET_DEVICE_HEALTH, SL_LIDAR_ANS_TYPE_DEVHEALTH, ans_frame, timeout);

            if (IS_FAIL(ans)) return ans;
            if (ans_frame->getPayloadSize() < sizeof(rplidar_response_device_health_t))
            {
                return SL_RESULT_INVALID_DATA;
            }
            health = *(rplidar_response_device_health_t*)ans_frame->getDataBuf();
#ifdef _CPU_ENDIAN_BIG
            health.error_code = le16_to_cpu(health.error_code);
#endif

            return ans;
        }

		sl_result getDeviceMacAddr(sl_u8* macAddrArray, sl_u32 timeoutInMs)
		{
            rp::hal::AutoLocker l(_op_locker);
            if (!isConnected()) return SL_RESULT_OPERATION_NOT_SUPPORT;


            u_result ans;

            std::vector<_u8> answer(6, 0);
            ans = getLidarConf(SL_LIDAR_CONF_LIDAR_MAC_ADDR, answer, NULL, 0, timeoutInMs);
            if (IS_FAIL(ans))
            {
                return ans;
            }
            size_t len = answer.size();
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
            count = _rawSampleNodeHolder.waitAndFetch(nodebuffer, count, 0);
            return SL_RESULT_OK;
        }

        sl_result setMotorSpeed(sl_u16 speed = DEFAULT_MOTOR_SPEED)
        {
            rp::hal::AutoLocker l(_op_locker);
            if (!isConnected()) return SL_RESULT_OPERATION_NOT_SUPPORT;


            Result<nullptr_t> ans = SL_RESULT_OK;
            
            if(speed == DEFAULT_MOTOR_SPEED){
                sl_lidar_response_desired_rot_speed_t desired_speed;
                ans = getDesiredSpeed(desired_speed);
                if (ans) {
                    if (_isSupportingMotorCtrl == MotorCtrlSupportPwm)
                        speed = desired_speed.pwm_ref;
                    else
                        speed = desired_speed.rpm;
                }
                else {
                    //set a dummy default value
                    speed = 600;
                }
            }
            switch (_isSupportingMotorCtrl)
            {
            case MotorCtrlSupportNone:
                if (_transeiver->getBindedChannel()->getChannelType() == CHANNEL_TYPE_SERIALPORT) {
                    ISerialPortChannel* serialChanel = (ISerialPortChannel*)_transeiver->getBindedChannel();
                    if (!speed) {
                        serialChanel->setDTR(true);
                    }else{
                        serialChanel->setDTR(false);
                    }
                }
                break;
            case MotorCtrlSupportPwm:
                sl_lidar_payload_motor_pwm_t motor_pwm;
                motor_pwm.pwm_value = speed;


                ans = _sendCommandWithoutResponse(SL_LIDAR_CMD_SET_MOTOR_PWM, &motor_pwm, sizeof(motor_pwm), true);
                if (!ans) return ans;
                delay(10);
                break;
            case MotorCtrlSupportRpm:
                sl_lidar_payload_motor_pwm_t motor_rpm;
                motor_rpm.pwm_value = speed;

                ans = _sendCommandWithoutResponse(SL_LIDAR_CMD_HQ_MOTOR_SPEED_CTRL, &motor_rpm, sizeof(motor_rpm), true);
                if (!ans) return ans;
                delay(10);
                break;
            }
            return SL_RESULT_OK;
        }

        sl_result getMotorInfo(LidarMotorInfo &motorInfo, sl_u32 timeoutInMs)
        {
            Result<nullptr_t> ans = SL_RESULT_OK;
            rp::hal::AutoLocker l(_op_locker);
            if (!isConnected()) return SL_RESULT_OPERATION_NOT_SUPPORT;


            {
                std::vector<sl_u8> answer;

			    ans = getLidarConf(SL_LIDAR_CONF_MIN_ROT_FREQ, answer);
			    if (!ans) return ans;

			    const sl_u16 *min_answer = reinterpret_cast<const sl_u16*>(&answer[0]);
                motorInfo.min_speed = *min_answer;


                ans = getLidarConf(SL_LIDAR_CONF_MAX_ROT_FREQ, answer);
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

        sl_result negotiateSerialBaudRate(sl_u32 requiredBaudRate, sl_u32* baudRateDetected)
        {
            // ask the LIDAR to stop working first...
            stop();

            rp::hal::AutoLocker l(_op_locker);

            IChannel* cachedChannel = _transeiver->getBindedChannel();
            if (!cachedChannel) return SL_RESULT_OPERATION_FAIL;
            if (cachedChannel->getChannelType() != CHANNEL_TYPE_SERIALPORT)
            {
                // only works for UART connection
                return RESULT_OPERATION_NOT_SUPPORT;
            }

            // disable the transeiver as it may interrupt the operation...
            _transeiver->unbindAndClose();

            sl_result ans = SL_RESULT_OK;

            do {
                // reopen the channel...

                if (!cachedChannel->open()) {
                    // failed to reopen
                    // try to revert back...
                    ans = SL_RESULT_OPERATION_FAIL;
                    break;
                }

                cachedChannel->flush();

                // wait for a while
                delay(10);
                cachedChannel->clearReadCache();

                // sending magic byte to let the target LIDAR start baudrate measurement
                // More than 100 bytes per second datarate is required to trigger the measurements
                {


                    sl_u8 magicByteSeq[16];

                    memset(magicByteSeq, SL_LIDAR_AUTOBAUD_MAGICBYTE, sizeof(magicByteSeq));

                    sl_u64 startTS = getms();

                    while (getms() - startTS < 1500) //lasting for 1.5sec
                    {
                        if (cachedChannel->write(magicByteSeq, sizeof(magicByteSeq)) < 0)
                        {
                            ans = SL_RESULT_OPERATION_FAIL;
                            break;
                        }

                        size_t dataCountGot;
                        if (cachedChannel->waitForData(1, 1, &dataCountGot)) {
                            //got reply, stop
                            ans = SL_RESULT_OK;
                            break;
                        }
                    }
                }

                if (IS_FAIL(ans)) break;

                // getback the bps measured
                _u32 bpsDetected = 0;
                size_t dataCountGot;
                if (cachedChannel->waitForData(4, 500, &dataCountGot)) {
                    //got reply, stop
                    cachedChannel->read(&bpsDetected, 4);
                    if (baudRateDetected) *baudRateDetected = bpsDetected;


                    cachedChannel->close();
                    // restart the transiever 
                    ans = _transeiver->openChannelAndBind(cachedChannel);
                    if (IS_FAIL(ans)) return ans;


                    // send a confirmation to the LIDAR, otherwise, the previous baudrate will be reverted back
                    sl_lidar_payload_new_bps_confirmation_t confirmation;
                    confirmation.flag = 0x5F5F;
                    confirmation.required_bps = requiredBaudRate;
                    confirmation.param = 0;


                    ans = _sendCommandWithoutResponse(SL_LIDAR_CMD_NEW_BAUDRATE_CONFIRM, &confirmation, sizeof(confirmation));

                    return ans;
                }
            } while (0);

            _transeiver->openChannelAndBind(cachedChannel);

            return ans;
        }

    protected:
        sl_result startMotor()
        {
            return setMotorSpeed(DEFAULT_MOTOR_SPEED);
        }

        u_result getDesiredSpeed(sl_lidar_response_desired_rot_speed_t & motorSpeed, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            u_result ans;
            std::vector<sl_u8> answer;
            ans = getLidarConf(SL_LIDAR_CONF_DESIRED_ROT_FREQ, answer, nullptr, 0, timeoutInMs);

            if (IS_FAIL(ans)) return ans;

            const sl_lidar_response_desired_rot_speed_t *p_answer = reinterpret_cast<const sl_lidar_response_desired_rot_speed_t*>(&answer[0]);
            motorSpeed = *p_answer;
            return RESULT_OK;
        }

        u_result checkSupportConfigCommands(bool& outSupport, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            u_result ans;
            rplidar_response_device_info_t devinfo;
            ans = getDeviceInfo(devinfo, timeoutInMs);
            if (IS_FAIL(ans)) {
                outSupport = false;
                return ans;
            }


            if (_checkNDMagicNumber(devinfo.model)) {

                outSupport = true;
            }
            else {
                // if lidar firmware >= 1.24
                outSupport = (devinfo.firmware_version >= ((0x1 << 8) | 24));
            }
            return RESULT_OK;
        }


        u_result getScanModeCount(sl_u16& modeCount, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            u_result ans;
            std::vector<_u8> answer;
            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_COUNT, answer);
            if (IS_FAIL(ans)) {
                return ans;
            }
            if (answer.size() < sizeof(_u16)) {
                return RESULT_INVALID_DATA;
            }
            const _u16* p_answer = reinterpret_cast<const _u16*>(&answer[0]);
            modeCount = *p_answer;
            return ans;
        }

        u_result setLidarConf(_u32 type, const void* payload, size_t payloadSize, _u32 timeout)
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
            internal::message_autoptr_t ans_frame;
            ans = _sendCommandWithResponse(SL_LIDAR_CMD_SET_LIDAR_CONF, SL_LIDAR_ANS_TYPE_SET_LIDAR_CONF, ans_frame, timeout, &requestPkt[0], requestPkt.size());

            if (IS_FAIL(ans)) {
                return ans;
            }

            //check if returned size is even less than sizeof(type) 
            if (ans_frame->getPayloadSize() < sizeof(rplidar_response_set_lidar_conf_t)) {
                return RESULT_INVALID_DATA;
            }

            const rplidar_response_set_lidar_conf_t* response =
                reinterpret_cast<const rplidar_response_set_lidar_conf_t*>(ans_frame->getDataBuf());


            if (ans_frame->getPayloadSize() == 4) {
                // legacy device?
                return *(const u_result*)(ans_frame->getDataBuf());
            }
            else {
                if (response->type != type) {
                    return RESULT_INVALID_DATA;
                }

                return (u_result)response->result;
            }
		}

        u_result getLidarConf(_u32 type, std::vector<_u8>& outputBuf, const void* payload = NULL, size_t payloadSize = 0, _u32 timeout = DEFAULT_TIMEOUT)
        {
            std::vector<_u8> requestPkt;

            if (!payload) payloadSize = 0;
            requestPkt.resize(sizeof(rplidar_payload_get_scan_conf_t) + payloadSize);
            rplidar_payload_get_scan_conf_t* query = reinterpret_cast<rplidar_payload_get_scan_conf_t*>(&requestPkt[0]);

            query->type = type;

            if (payloadSize)
                memcpy(&query[1], payload, payloadSize);

            u_result ans;
            internal::message_autoptr_t ans_frame;
            ans = _sendCommandWithResponse(SL_LIDAR_CMD_GET_LIDAR_CONF, SL_LIDAR_ANS_TYPE_GET_LIDAR_CONF, ans_frame, timeout, &requestPkt[0], requestPkt.size());
            if (IS_FAIL(ans)) {
                return ans;
            }
            //check if returned size is even less than sizeof(type) 
            if (ans_frame->getPayloadSize() < offsetof(rplidar_response_get_lidar_conf_t, payload)) {
                return SL_RESULT_INVALID_DATA;
            }

            //check if returned type is same as asked type
            const rplidar_response_get_lidar_conf_t* replied =
                reinterpret_cast<const rplidar_response_get_lidar_conf_t*>(ans_frame->getDataBuf());


            if (replied->type != type) {
                return SL_RESULT_INVALID_DATA;
            }
            //copy all the payload into &outputBuf
            int payLoadLen = (int)ans_frame->getPayloadSize() - (int)offsetof(rplidar_response_get_lidar_conf_t, payload);
            //do consistency check
            if (payLoadLen < 0) {
                return SL_RESULT_INVALID_DATA;
            }
            //copy all payLoadLen bytes to outputBuf
            outputBuf.resize(payLoadLen);
            if (payLoadLen)
                memcpy(&outputBuf[0], replied->payload, payLoadLen);
            return ans;
        }

        u_result getLidarSampleDuration(float& sampleDurationRes, sl_u16 scanModeID, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            u_result ans;

            std::vector<_u8> answer;
            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE, answer, &scanModeID, sizeof(_u16), timeoutInMs);
            if (IS_FAIL(ans))
            {
                return ans;
            }
            if (answer.size() < sizeof(_u32))
            {
                return SL_RESULT_INVALID_DATA;
            }
            const _u32* result = reinterpret_cast<const _u32*>(&answer[0]);
            sampleDurationRes = (float)(*result / 256.0);
            return ans;
        }

        u_result getMaxDistance(float &maxDistance, sl_u16 scanModeID, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            u_result ans;


            std::vector<_u8> answer;
            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_MAX_DISTANCE, answer, &scanModeID, sizeof(_u16), timeoutInMs);
            if (IS_FAIL(ans))
            {
                return ans;
            }
            if (answer.size() < sizeof(_u32))
            {
                return SL_RESULT_INVALID_DATA;
            }
            const _u32* result = reinterpret_cast<const _u32*>(&answer[0]);
            maxDistance = (float)(*result >> 8);
            return ans;
        }

        u_result getScanModeAnsType(sl_u8 &ansType, sl_u16 scanModeID, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            u_result ans;

            std::vector<_u8> answer;
            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_ANS_TYPE, answer, &scanModeID, sizeof(_u16), timeoutInMs);
            if (IS_FAIL(ans))
            {
                return ans;
            }
            if (answer.size() < sizeof(_u8))
            {
                return SL_RESULT_INVALID_DATA;
            }
            const _u8* result = reinterpret_cast<const _u8*>(&answer[0]);
            ansType = *result;
            return ans;
        }

        u_result getScanModeName(char* modeName, size_t stringSize, _u16 scanModeID, _u32 timeoutInMs = DEFAULT_TIMEOUT)
        {
            u_result ans;

            std::vector<_u8> answer;
            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_NAME, answer, &scanModeID, sizeof(_u16), timeoutInMs);
            if (IS_FAIL(ans))
            {
                return ans;
            }
            size_t len = std::min<size_t>(answer.size(), stringSize);
            if (0 == len) return SL_RESULT_INVALID_DATA;

            memcpy(modeName, &answer[0], len);
            return ans;
        }


        static LIDARTechnologyType ParseLIDARTechnologyTypeByModelID(_u8 modelID)
        {
            _u8 majorModelID = (modelID >> 4);
            // FIXME: stupid implementation here
            if (majorModelID < NEWDESIGN_MINUM_MAJOR_ID) {
                return LIDAR_TECHNOLOGY_TRIANGULATION;
            }
            else {
                return LIDAR_TECHNOLOGY_DTOF;
            }
        }

        static LIDARMajorType ParseLIDARMajorTypeByModelID(_u8 modelID)
        {
            _u8 majorModelID = (modelID >> 4);


            if (majorModelID >= TOF_M_SERIAL_MINUM_MAJOR_ID) {
                return LIDAR_MAJOR_TYPE_M_SERIES;
            }
            else if (majorModelID >= TOF_T_SERIAL_MINUM_MAJOR_ID) {
                return LIDAR_MAJOR_TYPE_T_SERIES;
            }
            else if (majorModelID >= TOF_S_SERIAL_MINUM_MAJOR_ID) {
                return LIDAR_MAJOR_TYPE_S_SERIES;
            }
            else if (majorModelID >= TOF_C_SERIAL_MINUM_MAJOR_ID) {
                return LIDAR_MAJOR_TYPE_C_SERIES;
            }
            else {
                return LIDAR_MAJOR_TYPE_A_SERIES;
            }
        }

        static std::string         GetModelNameStringByModelID(_u8 modelID)
        {

            char stringBuffer[100];
            auto majorType = ParseLIDARMajorTypeByModelID(modelID);


            switch (majorType) {
            case LIDAR_MAJOR_TYPE_A_SERIES:
                sprintf(stringBuffer, "A%dM%d", (modelID >> 4), (modelID & 0xF));

                break;

            case LIDAR_MAJOR_TYPE_S_SERIES:
                sprintf(stringBuffer, "S%dM%d", (modelID >> 4) - (TOF_S_SERIAL_MINUM_MAJOR_ID)+1, (modelID & 0xF));

                break;

            case LIDAR_MAJOR_TYPE_T_SERIES:
                sprintf(stringBuffer, "T%dM%d", (modelID >> 4) - (TOF_T_SERIAL_MINUM_MAJOR_ID)+1, (modelID & 0xF));

                break;

            case LIDAR_MAJOR_TYPE_M_SERIES:
                sprintf(stringBuffer, "M%dM%d", (modelID >> 4) - (TOF_M_SERIAL_MINUM_MAJOR_ID)+1, (modelID & 0xF));

                break;

            case LIDAR_MAJOR_TYPE_C_SERIES:
                sprintf(stringBuffer, "C%dM%d", (modelID >> 4) - (TOF_C_SERIAL_MINUM_MAJOR_ID)+1, (modelID & 0xF));

                break;


            default:
                sprintf(stringBuffer, "unknown(%x)", modelID);
            }

            return std::string(stringBuffer);
        }

    protected:
        
        void _disableDataGrabbing()
        {
            _dataunpacker->disable();
            _protocolHandler->exitLoopMode(); // exit loop mode
        }
        

   
        bool _checkNDMagicNumber(_u8 model)
        {
            return ((model >> 4) >= NEWDESIGN_MINUM_MAJOR_ID);
        }




        u_result _detectLIDARNativeInterfaceType(LIDARInterfaceType & outputType, const rplidar_response_device_info_t& devInfo, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            
            LIDARMajorType majorType = ParseLIDARMajorTypeByModelID(devInfo.model);
            
            switch (majorType) {
            case LIDAR_MAJOR_TYPE_A_SERIES:
            case LIDAR_MAJOR_TYPE_M_SERIES:
            case LIDAR_MAJOR_TYPE_C_SERIES:
          
                outputType = LIDAR_INTERFACE_UART;
                return SL_RESULT_OK;


            case LIDAR_MAJOR_TYPE_T_SERIES:
                outputType = LIDAR_INTERFACE_ETHERNET;
                return SL_RESULT_OK;

            case LIDAR_MAJOR_TYPE_S_SERIES:
            {
                // ethernet version exists, check whether it is
                _u8 macAddr[6];
                u_result ans = getDeviceMacAddr(macAddr, timeout);
                if (IS_FAIL(ans)) {
                    // cannot retrieve the device mac address, consider a UART interface version
                    outputType = LIDAR_INTERFACE_UART;
                }
                else {
                    outputType = LIDAR_INTERFACE_ETHERNET;
                }
                return SL_RESULT_OK;
            }


            case LIDAR_MAJOR_TYPE_UNKNOWN:
            default:
                outputType = LIDAR_INTERFACE_UNKNOWN;
                return SL_RESULT_OK;
            }
        }

        _u32 _getNativeBaudRate(const rplidar_response_device_info_t & devInfo)
        {
            _u8 majorModelID = (devInfo.model >> 4);
            switch (majorModelID)
            {
            case 1:
            case 2:
            case 3: //A1..A3 series
                return (devInfo.hardware_version >= 6) ? 256000 : 115200;
            case 4: //C series
                return 460800;
            case 6: //model ID of S1
                return 256000;
            case 7: //model ID of S2
            case 8: //model ID of S3
                if (devInfo.model == (0x82)) return 460800; 
                return 1000000;
            default:
                return 0; //0 as unknown
            }
        }

        bool _updateTimingDesc(const rplidar_response_device_info_t& devInfo, float selectedSampleDuration)
        {
            _timing_desc.native_baudrate = _getNativeBaudRate(devInfo);
            _detectLIDARNativeInterfaceType(_timing_desc.native_interface_type, devInfo, 500);
            
            _timing_desc.sample_duration_uS = (_u64)(selectedSampleDuration + 0.5f);

            //FIXME: will be changed in future releases
            _timing_desc.native_timestamp_support = false; 
            _timing_desc.linkage_delay_uS = 0;


            // notify the data unpacker
            _dataunpacker->updateUnpackerContext(internal::LIDARSampleDataUnpacker::UNPACKER_CONTEXT_TYPE_LIDAR_TIMING ,&_timing_desc, sizeof(_timing_desc));
            return true;

        }

        u_result _getLegacySampleDuration_uS(rplidar_response_sample_rate_t& rateInfo, _u32 timeout)
        {
            
            static const _u32 LEGACY_SAMPLE_DURATION = 476;

            rplidar_response_device_info_t devinfo;
            // 1. fetch the device version first...
            u_result ans = getDeviceInfo(devinfo, timeout);

            rateInfo.express_sample_duration_us = LEGACY_SAMPLE_DURATION;
            rateInfo.std_sample_duration_us = LEGACY_SAMPLE_DURATION;

            if (IS_FAIL(ans)) {
                return ans;
            }

            if (getLIDARMajorType(&devinfo) == LIDAR_MAJOR_TYPE_A_SERIES) {
                if (devinfo.firmware_version < ((0x1 << 8) | 17)) {
                    // very very rare and old model found!!
                    return SL_RESULT_OK;
                }
            }


            internal::message_autoptr_t ans_frame;

            ans = _sendCommandWithResponse(SL_LIDAR_CMD_GET_SAMPLERATE, SL_LIDAR_ANS_TYPE_SAMPLE_RATE, ans_frame, timeout);

            if (IS_FAIL(ans)) return ans;
            if (ans_frame->getPayloadSize() < sizeof(rplidar_response_sample_rate_t))
            {
                return RESULT_INVALID_DATA;
            }
            memcpy(&rateInfo, ans_frame->getDataBuf(), sizeof(rateInfo));

#ifdef _CPU_ENDIAN_BIG
            rateInfo.express_sample_duration_us = le16_to_cpu(rateInfo.express_sample_duration_us);
            rateInfo.std_sample_duration_us = le16_to_cpu(rateInfo.std_sample_duration_us);
#endif

            return ans;
        }


        u_result _sendCommandWithoutResponse(_u8 cmd, const void* payload = NULL, size_t payloadsize = 0, bool noForceStop = false)
        {
            if (!noForceStop) {
                _disableDataGrabbing();
            }
            _response_waiter.set(false);

            internal::message_autoptr_t message(new internal::ProtocolMessage(cmd, (const _u8*)payload, payloadsize));
            return _transeiver->sendMessage(message);

        }

        u_result _sendCommandWithResponse(_u8 cmd, _u8 responseType, internal::message_autoptr_t& ansPkt, _u32 timeout = DEFAULT_TIMEOUT, const void* payload = NULL, size_t payloadsize = 0)
        {
            u_result ans;

            _data_locker.lock();

            internal::message_autoptr_t message(new internal::ProtocolMessage(cmd, (const _u8*)payload, payloadsize));
            _disableDataGrabbing();
            _waiting_packet_type = responseType;
            _response_waiter.set(false);
            _data_locker.unlock();

            ans = _transeiver->sendMessage(message);

            if (IS_FAIL(ans)) return ans;

            do {
                switch (_response_waiter.wait(timeout)) {
                case rp::hal::Event::EVENT_TIMEOUT:
                    return RESULT_OPERATION_TIMEOUT;
                case rp::hal::Event::EVENT_OK:
                    _data_locker.lock();
                    ansPkt = _lastAnsPkt;
                    _data_locker.unlock();
                    return RESULT_OK;
                default:
                    return RESULT_OPERATION_FAIL;
                }
            } while (1);
        }
        
    public:

        virtual void onHQNodeDecoded(_u64 timestamp_uS, const rplidar_response_measurement_node_hq_t* node)
        {
            _scanHolder.pushScanNodeData(timestamp_uS, node);
            _rawSampleNodeHolder.pushNode(timestamp_uS, node);
        }

        virtual void onHQNodeScanResetReq() {
            _scanHolder.rewindCurrentScanData();
        }

        virtual void onProtocolMessageDecoded(const internal::ProtocolMessage& msg)
        {
            internal::message_autoptr_t message = std::make_shared<internal::ProtocolMessage>(msg);

            if (_dataunpacker->onSampleData(message->cmd, message->getDataBuf(), message->getPayloadSize()))
            {
                return;
            }

            if (message->cmd == _waiting_packet_type) {
                _data_locker.lock();
                _lastAnsPkt = message;
                _response_waiter.setResult(message->cmd);
                _data_locker.unlock();
            }

            
        }
    private:

        std::shared_ptr<internal::RPLidarProtocolCodec> _protocolHandler;
        std::shared_ptr<internal::AsyncTransceiver> _transeiver;
        std::shared_ptr<internal::LIDARSampleDataUnpacker> _dataunpacker;

        bool _isConnected;

        MotorCtrlSupport          _isSupportingMotorCtrl;


        rp::hal::Locker           _op_locker;
        rp::hal::Locker           _data_locker;
        rp::hal::Waiter<_u32>     _response_waiter;

        ScanDataHolder<sl_lidar_response_measurement_node_hq_t> _scanHolder;
        RawSampleNodeHolder<sl_lidar_response_measurement_node_hq_t> _rawSampleNodeHolder;
        _u32                          _waiting_packet_type;
        internal::message_autoptr_t   _lastAnsPkt;

        sl_lidar_response_device_info_t _cached_DevInfo;
        SlamtecLidarTimingDesc         _timing_desc;

    };

    Result<ILidarDriver*> createLidarDriver()
    {
        return new SlamtecLidarDriver();
    }
}