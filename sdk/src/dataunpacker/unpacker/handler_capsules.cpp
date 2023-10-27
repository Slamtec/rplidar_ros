/*
 *  Slamtec LIDAR SDK
 *
 *  Copyright (c) 2014 - 2023 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

 /*
  *  Sample Data Unpacker System
  *  Capsule Style Sample Node Handlers
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

#include "../dataunnpacker_commondef.h"
#include "../dataunpacker.h"
#include "../dataunnpacker_internal.h"



#include "handler_capsules.h"

BEGIN_DATAUNPACKER_NS()
	
namespace unpacker{


// UnpackerHandler_CapsuleNode
///////////////////////////////////////////////////////////////////////////////////

static _u64 _getSampleDelayOffsetInExpressMode(const SlamtecLidarTimingDesc& timing, int sampleIdx)
{
    // FIXME: to eval
    // 
    // guess channel baudrate by LIDAR model ....
    const _u64 channelBaudRate = timing.native_baudrate? timing.native_baudrate:115200;

    _u64 tranmissionDelay = 1000000ULL * sizeof(rplidar_response_capsule_measurement_nodes_t) * 10 / channelBaudRate;

    if (timing.native_interface_type == LIDARInterfaceType::LIDAR_INTERFACE_ETHERNET)
    {
        tranmissionDelay = 100; //dummy value
    }

    // center of the sample duration
    const _u64 sampleDelay = (timing.sample_duration_uS >> 1);
    const _u64 sampleFilterDelay = timing.sample_duration_uS;
    const _u64 groupingDelay = (31 - sampleIdx) * timing.sample_duration_uS;


    return sampleFilterDelay + sampleDelay + tranmissionDelay + timing.linkage_delay_uS + groupingDelay;
}


UnpackerHandler_CapsuleNode::UnpackerHandler_CapsuleNode()
    : _cached_scan_node_buf_pos(0)
    , _is_previous_capsuledataRdy(false)
    , _cached_last_data_timestamp_us(0)
{
    _cached_scan_node_buf.resize(sizeof(rplidar_response_capsule_measurement_nodes_t));
    memset(&_cachedTimingDesc, 0, sizeof(_cachedTimingDesc));
}

UnpackerHandler_CapsuleNode::~UnpackerHandler_CapsuleNode()
{

}

void UnpackerHandler_CapsuleNode::onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size)
{
    if (type == LIDARSampleDataUnpacker::UNPACKER_CONTEXT_TYPE_LIDAR_TIMING) {
        assert(size == sizeof(_cachedTimingDesc));
        _cachedTimingDesc = *reinterpret_cast<const SlamtecLidarTimingDesc*>(data);
    }
}


_u8 UnpackerHandler_CapsuleNode::getSampleAnswerType() const
{
	return RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
}

void UnpackerHandler_CapsuleNode::onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t cnt)
{
    for (size_t pos = 0; pos < cnt; ++pos) {
        _u8 current_data = data[pos];
        switch (_cached_scan_node_buf_pos) {
        case 0: // expect the sync bit 1
        {
            _u8 tmp = (current_data >> 4);
            if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
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
            _u8 tmp = (current_data >> 4);
            if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                // pass
            }
            else {
                _cached_scan_node_buf_pos = 0;
                _is_previous_capsuledataRdy = false;
                continue;
            }
        }
        break;

        case sizeof(rplidar_response_capsule_measurement_nodes_t) - 1: // new data ready
        {
            _cached_scan_node_buf[sizeof(rplidar_response_capsule_measurement_nodes_t) - 1] = current_data;
            _cached_scan_node_buf_pos = 0;

            rplidar_response_capsule_measurement_nodes_t* node = reinterpret_cast<rplidar_response_capsule_measurement_nodes_t*>(&_cached_scan_node_buf[0]);

            // calc the checksum ...
            _u8 checksum = 0;
            _u8 recvChecksum = ((node->s_checksum_1 & 0xF) | (node->s_checksum_2 << 4));
            for (size_t cpos = offsetof(rplidar_response_capsule_measurement_nodes_t, start_angle_sync_q6);
                cpos < sizeof(rplidar_response_capsule_measurement_nodes_t); ++cpos)
            {
                checksum ^= _cached_scan_node_buf[cpos];
            }

            if (recvChecksum == checksum)
            {
                // only consider vaild if the checksum matches...

                // perform data endianess convertion if necessary
#ifdef _CPU_ENDIAN_BIG
                node->start_angle_sync_q6 = le16_to_cpu(node->start_angle_sync_q6);
                for (size_t cpos = 0; cpos < _countof(node->cabins); ++cpos) {
                    node->cabins[cpos].distance_angle_1 = le16_to_cpu(node->cabins[cpos].distance_angle_1);
                    node->cabins[cpos].distance_angle_2 = le16_to_cpu(node->cabins[cpos].distance_angle_2);
                }
#endif
                if (node->start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT)
                {
                    if (_is_previous_capsuledataRdy) {
                        engine->publishDecodingErrorMsg(LIDARSampleDataUnpacker::ERR_EVENT_ON_EXP_ENCODER_RESET
                            , RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED, node, sizeof(*node));
                    }
                    // this is the first capsule frame in logic, discard the previous cached data...
                    _is_previous_capsuledataRdy = false;
                    engine->publishNewScanReset();


                }
                _onScanNodeCapsuleData(*node, engine);
            }
            else {
                _is_previous_capsuledataRdy = false;


                engine->publishDecodingErrorMsg(LIDARSampleDataUnpacker::ERR_EVENT_ON_EXP_CHECKSUM_ERR
                    , RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED, node, sizeof(*node));

            }
            continue;
        }
        break;

        }
        _cached_scan_node_buf[_cached_scan_node_buf_pos++] = current_data;
    }

}

void UnpackerHandler_CapsuleNode::reset()
{
    _cached_scan_node_buf_pos = 0;
    _is_previous_capsuledataRdy = false;
    _cached_last_data_timestamp_us = 0;
}

void UnpackerHandler_CapsuleNode::_onScanNodeCapsuleData(rplidar_response_capsule_measurement_nodes_t& capsule, LIDARSampleDataUnpackerInner* engine)
{
    _u64 currentTS = engine->getCurrentTimestamp_uS();
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
        for (int pos = 0; pos < (int)_countof(_cached_previous_capsuledata.cabins); ++pos)
        {
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

                rplidar_response_measurement_node_hq_t hqNode;


                hqNode.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                hqNode.quality = dist_q2[cpos] ? (0x2F << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;

                hqNode.angle_z_q14 = (angle_q6[cpos] << 8) / 90;
                hqNode.dist_mm_q2 = dist_q2[cpos];

                engine->publishHQNode(_cached_last_data_timestamp_us - _getSampleDelayOffsetInExpressMode(_cachedTimingDesc, pos * 2 + cpos), &hqNode);
            }

        }
    }

    _cached_previous_capsuledata = capsule;
    _is_previous_capsuledataRdy = true;
    _cached_last_data_timestamp_us = currentTS;

}


// UnpackerHandler_UltraCapsuleNode
///////////////////////////////////////////////////////////////////////////////////

static _u64 _getSampleDelayOffsetInUltraBoostMode(const SlamtecLidarTimingDesc& timing, int sampleIdx)
{
    // FIXME: to eval
    // 
    // guess channel baudrate by LIDAR model ....
    const _u64 channelBaudRate = timing.native_baudrate ? timing.native_baudrate : 256000;

    _u64 tranmissionDelay = 1000000ULL * sizeof(rplidar_response_ultra_capsule_measurement_nodes_t) * 10 / channelBaudRate;

    if (timing.native_interface_type == LIDARInterfaceType::LIDAR_INTERFACE_ETHERNET)
    {
        tranmissionDelay = 100; //dummy value
    }

    // center of the sample duration
    const _u64 sampleDelay = (timing.sample_duration_uS >> 1);
    const _u64 sampleFilterDelay = timing.sample_duration_uS;
    const _u64 groupingDelay = ((32 * 3 - 1) - sampleIdx) * timing.sample_duration_uS;


    return sampleFilterDelay + sampleDelay + tranmissionDelay + timing.linkage_delay_uS + groupingDelay;
}


UnpackerHandler_UltraCapsuleNode::UnpackerHandler_UltraCapsuleNode()
    : _cached_scan_node_buf_pos(0)
    , _is_previous_capsuledataRdy(false)
    , _cached_last_data_timestamp_us(0)
{
    _cached_scan_node_buf.resize(sizeof(rplidar_response_ultra_capsule_measurement_nodes_t));
    memset(&_cachedTimingDesc, 0, sizeof(_cachedTimingDesc));
}

UnpackerHandler_UltraCapsuleNode::~UnpackerHandler_UltraCapsuleNode()
{

}

void UnpackerHandler_UltraCapsuleNode::onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size)
{
    if (type == LIDARSampleDataUnpacker::UNPACKER_CONTEXT_TYPE_LIDAR_TIMING) {
        assert(size == sizeof(_cachedTimingDesc));
        _cachedTimingDesc = *reinterpret_cast<const SlamtecLidarTimingDesc*>(data);
    }
}


_u8 UnpackerHandler_UltraCapsuleNode::getSampleAnswerType() const
{
    return RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA;
}

void UnpackerHandler_UltraCapsuleNode::onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t cnt)
{

    for (size_t pos = 0; pos < cnt; ++pos) {
        _u8 current_data = data[pos];
        switch (_cached_scan_node_buf_pos) {
        case 0: // expect the sync bit 1
        {
            _u8 tmp = (current_data >> 4);
            if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
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
            _u8 tmp = (current_data >> 4);
            if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                // pass
            }
            else {
                _cached_scan_node_buf_pos = 0;
                _is_previous_capsuledataRdy = false;
                continue;
            }
        }
        break;

        case sizeof(rplidar_response_ultra_capsule_measurement_nodes_t) - 1: // new data ready
        {
            _cached_scan_node_buf[sizeof(rplidar_response_ultra_capsule_measurement_nodes_t) - 1] = current_data;
            _cached_scan_node_buf_pos = 0;

            rplidar_response_ultra_capsule_measurement_nodes_t* node = reinterpret_cast<rplidar_response_ultra_capsule_measurement_nodes_t*>(&_cached_scan_node_buf[0]);

            // calc the checksum ...
            _u8 checksum = 0;
            _u8 recvChecksum = ((node->s_checksum_1 & 0xF) | (node->s_checksum_2 << 4));
            for (size_t cpos = offsetof(rplidar_response_ultra_capsule_measurement_nodes_t, start_angle_sync_q6);
                cpos < sizeof(rplidar_response_ultra_capsule_measurement_nodes_t); ++cpos)
            {
                checksum ^= _cached_scan_node_buf[cpos];
            }

            if (recvChecksum == checksum)
            {
                // only consider vaild if the checksum matches...

                // perform data endianess convertion if necessary
#ifdef _CPU_ENDIAN_BIG
                node->start_angle_sync_q6 = le16_to_cpu(node->start_angle_sync_q6);
                for (size_t cpos = 0; cpos < _countof(node->ultra_cabins); ++cpos) {
                    node->ultra_cabins[cpos].combined_x3 = le32_to_cpu(node->ultra_cabins[cpos].combined_x3);
                }
#endif
                if (node->start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT)
                {
                    if (_is_previous_capsuledataRdy) {
                        engine->publishDecodingErrorMsg(LIDARSampleDataUnpacker::ERR_EVENT_ON_EXP_ENCODER_RESET
                            , RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA, node, sizeof(*node));

                    }
                    // this is the first capsule frame in logic, discard the previous cached data...
                    _is_previous_capsuledataRdy = false;

                    engine->publishNewScanReset();

                }
                _onScanNodeUltraCapsuleData(*node, engine);
            }
            else {
                _is_previous_capsuledataRdy = false;

                engine->publishDecodingErrorMsg(LIDARSampleDataUnpacker::ERR_EVENT_ON_EXP_CHECKSUM_ERR
                    , RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA, node, sizeof(*node));

            }
            continue;
        }
        break;

        }
        _cached_scan_node_buf[_cached_scan_node_buf_pos++] = current_data;
    }

}

void UnpackerHandler_UltraCapsuleNode::reset()
{
    _cached_scan_node_buf_pos = 0;
    _is_previous_capsuledataRdy = false;
}

static _u32 _varbitscale_decode(_u32 scaled, _u32& scaleLevel)
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

void UnpackerHandler_UltraCapsuleNode::_onScanNodeUltraCapsuleData(rplidar_response_ultra_capsule_measurement_nodes_t& capsule, LIDARSampleDataUnpackerInner* engine)
{
    _u64 currentTS = engine->getCurrentTimestamp_uS();
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
        for (int pos = 0; pos < (int)_countof(_cached_previous_ultracapsuledata.ultra_cabins); ++pos)
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

            _u32 scalelvl1=0, scalelvl2 = 0;

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
            if (((_u32)dist_predict1 == 0xFFFFFE00) || ((_u32)dist_predict1 == 0x1FF)) {
                dist_q2[1] = 0;
            }
            else {
                dist_predict1 = (int)(dist_predict1 << scalelvl1);
                dist_q2[1] = (dist_predict1 + dist_base1) << 2;

            }

            if (((_u32)dist_predict2 == 0xFFFFFE00) || ((_u32)dist_predict2 == 0x1FF)) {
                dist_q2[2] = 0;
            }
            else {
                dist_predict2 = (int)(dist_predict2 << scalelvl2);
                dist_q2[2] = (dist_predict2 + dist_base2) << 2;
            }

            for (int cpos = 0; cpos < 3; ++cpos)
            {

                syncBit[cpos] = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;


                rplidar_response_measurement_node_hq_t hqNode;


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


                hqNode.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                hqNode.quality = dist_q2[cpos] ? (0x2F << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;

                hqNode.angle_z_q14 = (angle_q6[cpos] << 8) / 90;
                hqNode.dist_mm_q2 = dist_q2[cpos];

                engine->publishHQNode(_cached_last_data_timestamp_us - _getSampleDelayOffsetInUltraBoostMode(_cachedTimingDesc, pos * 3 + cpos), &hqNode);
            }

        }
    }

    _cached_previous_ultracapsuledata = capsule;
    _is_previous_capsuledataRdy = true;
    _cached_last_data_timestamp_us = currentTS;

}


// UnpackerHandler_DenseCapsuleNode
///////////////////////////////////////////////////////////////////////////////////

static _u64 _getSampleDelayOffsetInDenseMode(const SlamtecLidarTimingDesc& timing, int sampleIdx)
{
    // FIXME: to eval
    // 
    // guess channel baudrate by LIDAR model ....
    const _u64 channelBaudRate = timing.native_baudrate ? timing.native_baudrate : 256000;

    _u64 tranmissionDelay = 1000000ULL * sizeof(rplidar_response_dense_capsule_measurement_nodes_t) * 10 / channelBaudRate;

    if (timing.native_interface_type == LIDARInterfaceType::LIDAR_INTERFACE_ETHERNET)
    {
        tranmissionDelay = 100; //dummy value
    }

    // center of the sample duration
    const _u64 sampleDelay = (timing.sample_duration_uS >> 1);
    const _u64 sampleFilterDelay = timing.sample_duration_uS;
    const _u64 groupingDelay = (39 - sampleIdx) * timing.sample_duration_uS;


    return sampleFilterDelay + sampleDelay + tranmissionDelay + timing.linkage_delay_uS + groupingDelay;
}

UnpackerHandler_DenseCapsuleNode::UnpackerHandler_DenseCapsuleNode()
    : _cached_scan_node_buf_pos(0)
    , _is_previous_capsuledataRdy(false)
    , _cached_last_data_timestamp_us(0)

{
    _cached_scan_node_buf.resize(sizeof(rplidar_response_dense_capsule_measurement_nodes_t));
    memset(&_cachedTimingDesc, 0, sizeof(_cachedTimingDesc));
}

UnpackerHandler_DenseCapsuleNode::~UnpackerHandler_DenseCapsuleNode()
{

}

void UnpackerHandler_DenseCapsuleNode::onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size)
{
    if (type == LIDARSampleDataUnpacker::UNPACKER_CONTEXT_TYPE_LIDAR_TIMING) {
        assert(size == sizeof(_cachedTimingDesc));
        _cachedTimingDesc = *reinterpret_cast<const SlamtecLidarTimingDesc*>(data);
    }
}


_u8 UnpackerHandler_DenseCapsuleNode::getSampleAnswerType() const
{
    return RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED;
}


void UnpackerHandler_DenseCapsuleNode::onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t cnt)
{

    for (size_t pos = 0; pos < cnt; ++pos) {
        _u8 current_data = data[pos];
        switch (_cached_scan_node_buf_pos) {
        case 0: // expect the sync bit 1
        {
            _u8 tmp = (current_data >> 4);
            if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
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
            _u8 tmp = (current_data >> 4);
            if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                // pass
            }
            else {
                _cached_scan_node_buf_pos = 0;
                _is_previous_capsuledataRdy = false;
                continue;
            }
        }
        break;

        case sizeof(rplidar_response_dense_capsule_measurement_nodes_t) - 1: // new data ready
        {
            _cached_scan_node_buf[sizeof(rplidar_response_dense_capsule_measurement_nodes_t) - 1] = current_data;
            _cached_scan_node_buf_pos = 0;

            rplidar_response_dense_capsule_measurement_nodes_t* node = reinterpret_cast<rplidar_response_dense_capsule_measurement_nodes_t*>(&_cached_scan_node_buf[0]);

            // calc the checksum ...
            _u8 checksum = 0;
            _u8 recvChecksum = ((node->s_checksum_1 & 0xF) | (node->s_checksum_2 << 4));
            for (size_t cpos = offsetof(rplidar_response_dense_capsule_measurement_nodes_t, start_angle_sync_q6);
                cpos < sizeof(rplidar_response_dense_capsule_measurement_nodes_t); ++cpos)
            {
                checksum ^= _cached_scan_node_buf[cpos];
            }

            if (recvChecksum == checksum)
            {
                // only consider vaild if the checksum matches...

                // perform data endianess convertion if necessary
#ifdef _CPU_ENDIAN_BIG
                node->start_angle_sync_q6 = le16_to_cpu(node->start_angle_sync_q6);
                for (size_t cpos = 0; cpos < _countof(node->cabins); ++cpos) {
                    node->cabins[cpos].distance_angle_1 = le16_to_cpu(node->cabins[cpos].distance_angle_1);
                    node->cabins[cpos].distance_angle_2 = le16_to_cpu(node->cabins[cpos].distance_angle_2);
                }
#endif
                if (node->start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT)
                {
                    if (_is_previous_capsuledataRdy) {
                        engine->publishDecodingErrorMsg(LIDARSampleDataUnpacker::ERR_EVENT_ON_EXP_ENCODER_RESET
                            , RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED, node, sizeof(*node));
                    }
                    // this is the first capsule frame in logic, discard the previous cached data...
                    _is_previous_capsuledataRdy = false;
                    engine->publishNewScanReset();


                }
                _onScanNodeDenseCapsuleData(*node, engine);
            }
            else {
                _is_previous_capsuledataRdy = false;

                engine->publishDecodingErrorMsg(LIDARSampleDataUnpacker::ERR_EVENT_ON_EXP_CHECKSUM_ERR
                    , RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED, node, sizeof(*node));

            }
            continue;
        }
        break;

        }
        _cached_scan_node_buf[_cached_scan_node_buf_pos++] = current_data;
    }
}

void UnpackerHandler_DenseCapsuleNode::reset()
{
    _cached_scan_node_buf_pos = 0;
    _cached_last_data_timestamp_us = 0;
}

void UnpackerHandler_DenseCapsuleNode::_onScanNodeDenseCapsuleData(rplidar_response_dense_capsule_measurement_nodes_t& dense_capsule, LIDARSampleDataUnpackerInner* engine)
{
    static int lastNodeSyncBit = 0;
    _u64 currentTs = engine->getCurrentTimestamp_uS();

    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((dense_capsule.start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_dense_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 > currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }
        int maxDiffAngleThreshold_q8 = (360/* 360 degree */ * 100 /*100Hz*/ * _countof(dense_capsule.cabins) /*40 points per capsule*/ / (1000000 / _cachedTimingDesc.sample_duration_uS)) << 8;
        if (diffAngle_q8 > maxDiffAngleThreshold_q8) {//discard
            _cached_previous_dense_capsuledata = dense_capsule;
            return;
        }

        int angleInc_q16 = (diffAngle_q8 << 8) / 40;
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (int pos = 0; pos < (int)_countof(_cached_previous_dense_capsuledata.cabins); ++pos)
        {
            int dist_q2;
            int angle_q6;
            int syncBit;
            const int dist = static_cast<const int>(_cached_previous_dense_capsuledata.cabins[pos].distance);
            dist_q2 = dist << 2;
            angle_q6 = (currentAngle_raw_q16 >> 10);
            syncBit = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < (angleInc_q16 << 1)) ? 1 : 0;
            syncBit = (syncBit ^ lastNodeSyncBit) & syncBit;//Ensure that syncBit is exactly detected

            currentAngle_raw_q16 += angleInc_q16;

            if (angle_q6 < 0) angle_q6 += (360 << 6);
            if (angle_q6 >= (360 << 6)) angle_q6 -= (360 << 6);

            rplidar_response_measurement_node_hq_t hqNode;


            hqNode.flag = (syncBit | ((!syncBit) << 1));
            hqNode.quality = dist_q2 ? (0x2F << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
            hqNode.angle_z_q14 = (angle_q6 << 8) / 90;
            hqNode.dist_mm_q2 = dist_q2;
            engine->publishHQNode(currentTs - _getSampleDelayOffsetInDenseMode(_cachedTimingDesc, pos), &hqNode);
            
            lastNodeSyncBit = syncBit;

        }
    }

    _cached_previous_dense_capsuledata = dense_capsule;
    _is_previous_capsuledataRdy = true;

}

// UnpackerHandler_UltraDenseCapsuleNode
///////////////////////////////////////////////////////////////////////////////////

static _u64 _getSampleDelayOffsetInUltraDenseMode(const SlamtecLidarTimingDesc& timing, int sampleIdx)
{
    // FIXME: to eval
    // 
    // guess channel baudrate by LIDAR model ....
    const _u64 channelBaudRate = timing.native_baudrate ? timing.native_baudrate : 1000000;

    _u64 tranmissionDelay = 1000000ULL * sizeof(sl_lidar_response_ultra_dense_capsule_measurement_nodes_t) * 10 / channelBaudRate;

    if (timing.native_interface_type == LIDARInterfaceType::LIDAR_INTERFACE_ETHERNET)
    {
        tranmissionDelay = 100; //dummy value
    }

    // center of the sample duration
    const _u64 sampleDelay = (timing.sample_duration_uS >> 1);
    const _u64 sampleFilterDelay = timing.sample_duration_uS;
    const _u64 groupingDelay = (31 - sampleIdx) * timing.sample_duration_uS;


    return sampleFilterDelay + sampleDelay + tranmissionDelay + timing.linkage_delay_uS + groupingDelay;
}


UnpackerHandler_UltraDenseCapsuleNode::UnpackerHandler_UltraDenseCapsuleNode()
    : _cached_scan_node_buf_pos(0)
    , _is_previous_capsuledataRdy(false)
    , _cached_last_data_timestamp_us(0)
    , _last_node_sync_bit(0)
    , _last_dist_q2(0)

{
    _cached_scan_node_buf.resize(sizeof(rplidar_response_ultra_dense_capsule_measurement_nodes_t));
    memset(&_cachedTimingDesc, 0, sizeof(_cachedTimingDesc));
}

UnpackerHandler_UltraDenseCapsuleNode::~UnpackerHandler_UltraDenseCapsuleNode()
{

}


void UnpackerHandler_UltraDenseCapsuleNode::onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size)
{
    if (type == LIDARSampleDataUnpacker::UNPACKER_CONTEXT_TYPE_LIDAR_TIMING) {
        assert(size == sizeof(_cachedTimingDesc));
        _cachedTimingDesc = *reinterpret_cast<const SlamtecLidarTimingDesc*>(data);
    }
}


_u8 UnpackerHandler_UltraDenseCapsuleNode::getSampleAnswerType() const
{
    return RPLIDAR_ANS_TYPE_MEASUREMENT_ULTRA_DENSE_CAPSULED;
}

void UnpackerHandler_UltraDenseCapsuleNode::onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t cnt)
{
    for (size_t pos = 0; pos < cnt; ++pos) {
        _u8 current_data = data[pos];
        switch (_cached_scan_node_buf_pos) {
        case 0: // expect the sync bit 1
        {
            _u8 tmp = (current_data >> 4);
            if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
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
            _u8 tmp = (current_data >> 4);
            if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                // pass
            }
            else {
                _cached_scan_node_buf_pos = 0;
                _is_previous_capsuledataRdy = false;
                continue;
            }
        }
        break;

        case sizeof(rplidar_response_ultra_dense_capsule_measurement_nodes_t) - 1: // new data ready
        {
            _cached_scan_node_buf[sizeof(rplidar_response_ultra_dense_capsule_measurement_nodes_t) - 1] = current_data;
            _cached_scan_node_buf_pos = 0;

            rplidar_response_ultra_dense_capsule_measurement_nodes_t* node = reinterpret_cast<rplidar_response_ultra_dense_capsule_measurement_nodes_t*>(&_cached_scan_node_buf[0]);

            // calc the checksum ...
            _u8 checksum = 0;
            _u8 recvChecksum = ((node->s_checksum_1 & 0xF) | (node->s_checksum_2 << 4));
            for (size_t cpos = offsetof(rplidar_response_ultra_dense_capsule_measurement_nodes_t, time_stamp);
                cpos < sizeof(rplidar_response_ultra_dense_capsule_measurement_nodes_t); ++cpos)
            {
                checksum ^= _cached_scan_node_buf[cpos];
            }

            if (recvChecksum == checksum)
            {
                // only consider vaild if the checksum matches...

                // perform data endianess convertion if necessary
#ifdef _CPU_ENDIAN_BIG
                node->start_angle_sync_q6 = le16_to_cpu(node->start_angle_sync_q6);
                for (size_t cpos = 0; cpos < _countof(node->cabins); ++cpos) {
                    node->cabins[cpos].qualityl_distance_scale[0] = le16_to_cpu(node->cabins[cpos].qualityl_distance_scale[0]);
                    node->cabins[cpos].qualityl_distance_scale[1] = le16_to_cpu(node->cabins[cpos].qualityl_distance_scale[1]);
                }
#endif
                if (node->start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT)
                {
                    if (_is_previous_capsuledataRdy) {
                        engine->publishDecodingErrorMsg(LIDARSampleDataUnpacker::ERR_EVENT_ON_EXP_ENCODER_RESET
                            , RPLIDAR_ANS_TYPE_MEASUREMENT_ULTRA_DENSE_CAPSULED, node, sizeof(*node));

                    }
                    // this is the first capsule frame in logic, discard the previous cached data...
                    _is_previous_capsuledataRdy = false;
                    engine->publishNewScanReset();

                }
                _onScanNodeUltraDenseCapsuleData(*node, engine);
            }
            else {
                _is_previous_capsuledataRdy = false;

                engine->publishDecodingErrorMsg(LIDARSampleDataUnpacker::ERR_EVENT_ON_EXP_CHECKSUM_ERR
                    , RPLIDAR_ANS_TYPE_MEASUREMENT_ULTRA_DENSE_CAPSULED, node, sizeof(*node));

            }
            continue;
        }
        break;

        }
        _cached_scan_node_buf[_cached_scan_node_buf_pos++] = current_data;
    }

}

void UnpackerHandler_UltraDenseCapsuleNode::reset()
{
    _cached_scan_node_buf_pos = 0;
    _cached_last_data_timestamp_us = 0;
    _last_node_sync_bit = 0;
    _last_dist_q2 = 0;
}

void UnpackerHandler_UltraDenseCapsuleNode::_onScanNodeUltraDenseCapsuleData(rplidar_response_ultra_dense_capsule_measurement_nodes_t& capsule, LIDARSampleDataUnpackerInner* engine)
{
    _u64 currentTimestamp = engine->getCurrentTimestamp_uS();

    const rplidar_response_ultra_dense_capsule_measurement_nodes_t* ultra_dense_capsule = reinterpret_cast<const rplidar_response_ultra_dense_capsule_measurement_nodes_t*>(&capsule);
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((ultra_dense_capsule->start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_ultra_dense_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);



        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 > currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        int maxDiffAngleThreshold_q8 = (360/* 360 degree */ * 100 /*100Hz*/ * _countof(ultra_dense_capsule->cabins) /*64 points per capsule*/ / (1000000 / _cachedTimingDesc.sample_duration_uS)) << 8;
        if (diffAngle_q8 > maxDiffAngleThreshold_q8) {//discard
            _cached_previous_ultra_dense_capsuledata = *ultra_dense_capsule;
            return;
        }
#define DISTANCE_THRESHOLD_TO_SCALE_1 2046  // (2^10 - 1)*2 mm
#define DISTANCE_THRESHOLD_TO_SCALE_2 8187  // (2^11 - 1)*3 + 2046 mm
#define DISTANCE_THRESHOLD_TO_SCALE_3 24567 // (2^12 - 1)*4 + 8187 mm
        int angleInc_q16 = (diffAngle_q8 << 8) / 64;
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (int pos = 0; pos < (int)_countof(_cached_previous_ultra_dense_capsuledata.cabins) * 2; ++pos)
        {
            int angle_q6;
            int syncBit;
            size_t cabin_idx = pos >> 1;
            _u32  quality_dist_scale;
            if (!(pos & 0x1)) {
                quality_dist_scale = _cached_previous_ultra_dense_capsuledata.cabins[cabin_idx].qualityl_distance_scale[0] | ((_cached_previous_ultra_dense_capsuledata.cabins[cabin_idx].qualityh_array & 0x0F) << 16);
            }
            else {
                quality_dist_scale = _cached_previous_ultra_dense_capsuledata.cabins[cabin_idx].qualityl_distance_scale[1] | ((_cached_previous_ultra_dense_capsuledata.cabins[cabin_idx].qualityh_array >> 4) << 16);
            }

            _u8 scale = quality_dist_scale & 0x3;
            _u8 quality = 0;
            int dist_q2 = 0;

            switch (scale) {
            case 0:
                quality = quality_dist_scale >> 12;
                dist_q2 = (quality_dist_scale & 0xFFC) * 2;
                if (_last_dist_q2) {
                    if (abs(dist_q2 - _last_dist_q2) <= 8/*2mm *2*/) {
                        dist_q2 = (dist_q2 + _last_dist_q2) >> 1;
                    }
                }
                break;
            case 1:
                quality = (quality_dist_scale >> 13) << 1;
                dist_q2 = (quality_dist_scale & 0x1FFC) * 3 + (DISTANCE_THRESHOLD_TO_SCALE_1 << 2);
                break;
            case 2:
                quality = (quality_dist_scale >> 14) << 2;
                dist_q2 = (quality_dist_scale & 0x3FFC) * 4 + (DISTANCE_THRESHOLD_TO_SCALE_2 << 2);
                break;
            case 3:
                quality = (quality_dist_scale >> 15) << 3;
                dist_q2 = (quality_dist_scale & 0x7FFC) * 5 + (DISTANCE_THRESHOLD_TO_SCALE_3 << 2);
                break;
            }
            _last_dist_q2 = dist_q2;
            angle_q6 = (currentAngle_raw_q16 >> 10);
            syncBit = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < (angleInc_q16 << 1)) ? 1 : 0;
            syncBit = (syncBit ^ _last_node_sync_bit) & syncBit;//Ensure that syncBit is exactly detected

            currentAngle_raw_q16 += angleInc_q16;

            if (angle_q6 < 0) angle_q6 += (360 << 6);
            if (angle_q6 >= (360 << 6)) angle_q6 -= (360 << 6);


            rplidar_response_measurement_node_hq_t hqNode;



            hqNode.flag = (syncBit | ((!syncBit) << 1));
            hqNode.quality = quality;
            hqNode.angle_z_q14 = (angle_q6 << 8) / 90;
            hqNode.dist_mm_q2 = dist_q2;
            engine->publishHQNode(currentTimestamp - _getSampleDelayOffsetInUltraDenseMode(_cachedTimingDesc, pos), &hqNode);
            
            _last_node_sync_bit = syncBit;

        }
    }

    _cached_previous_ultra_dense_capsuledata = *ultra_dense_capsule;
    _is_previous_capsuledataRdy = true;

}



}


END_DATAUNPACKER_NS()