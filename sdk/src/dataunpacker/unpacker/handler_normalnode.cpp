/*
 *  Slamtec LIDAR SDK
 *
 *  Copyright (c) 2014 - 2023 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

 /*
  *  Sample Data Unpacker System
  *  Normal Sample Node Handler
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


#include "handler_normalnode.h"

BEGIN_DATAUNPACKER_NS()
	
namespace unpacker{


static _u64 _getSampleDelayOffsetInLegacyMode(const SlamtecLidarTimingDesc& timing)
{
    // guess channel baudrate by LIDAR model ....
    const _u64 channelBaudRate = timing.native_baudrate? timing.native_baudrate:115200;

    _u64 tranmissionDelay = 1000000ULL * sizeof(rplidar_response_measurement_node_t) * 10 / channelBaudRate;

    if (timing.native_interface_type == LIDARInterfaceType::LIDAR_INTERFACE_ETHERNET)
    {
        tranmissionDelay = 100; //dummy value
    }

    // center of the sample duration
    const _u64 sampleDelay = (timing.sample_duration_uS >> 1);
    const _u64 sampleFilterDelay = timing.sample_duration_uS;

    return sampleFilterDelay + sampleDelay + tranmissionDelay + timing.linkage_delay_uS;
}

UnpackerHandler_NormalNode::UnpackerHandler_NormalNode()
    : _cached_scan_node_buf_pos(0)
{
    _cached_scan_node_buf.resize(sizeof(rplidar_response_measurement_node_t));
    memset(&_cachedTimingDesc, 0, sizeof(_cachedTimingDesc));
;}

UnpackerHandler_NormalNode::~UnpackerHandler_NormalNode()
{

}

_u8 UnpackerHandler_NormalNode::getSampleAnswerType() const
{
	return RPLIDAR_ANS_TYPE_MEASUREMENT;
}

void UnpackerHandler_NormalNode::onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t cnt)
{
    for (size_t pos = 0; pos < cnt; ++pos) {
        _u8 current_data = data[pos];
        switch (_cached_scan_node_buf_pos) {
        case 0: // expect the sync bit and its reverse in this byte
        {
            _u8 tmp = (current_data >> 1);
            if ((tmp ^ current_data) & 0x1) {
                // pass
            }
            else {
                continue;
            }

        }
        break;
        case 1: // expect the highest bit to be 1
        {
            if (current_data & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
                // pass
            }
            else {
                _cached_scan_node_buf_pos = 0;
                continue;
            }
        }
        break;
        case sizeof(rplidar_response_measurement_node_t) - 1: // new data ready
        {
            _cached_scan_node_buf[sizeof(rplidar_response_measurement_node_t) - 1] = current_data;
            _cached_scan_node_buf_pos = 0;

            rplidar_response_measurement_node_t* node = reinterpret_cast<rplidar_response_measurement_node_t*>(&_cached_scan_node_buf[0]);
#ifdef _CPU_ENDIAN_BIG
            node->angle_q6_checkbit = le16_to_cpu(node->angle_q6_checkbit);
            node->distance_q2 = le16_to_cpu(node->distance_q2);
#endif
            //cast node to rplidar_response_measurement_node_hq_t
            rplidar_response_measurement_node_hq_t hqNode;
            hqNode.angle_z_q14 = (((node->angle_q6_checkbit) >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) << 8) / 90;  //transfer to q14 Z-angle
            hqNode.dist_mm_q2 = node->distance_q2;
            hqNode.flag = (node->sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);  // trasfer syncbit to HQ flag field
            hqNode.quality = (node->sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;  //remove the last two bits and then make quality from 0-63 to 0-255
            
            
            engine->publishHQNode(engine->getCurrentTimestamp_uS() - _getSampleDelayOffsetInLegacyMode(_cachedTimingDesc), &hqNode);
            continue;

        }
        break;
        }
        _cached_scan_node_buf[_cached_scan_node_buf_pos++] = current_data;
    }
}


void UnpackerHandler_NormalNode::onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size)
{
    if (type == LIDARSampleDataUnpacker::UNPACKER_CONTEXT_TYPE_LIDAR_TIMING) {
        assert(size == sizeof(_cachedTimingDesc));
        _cachedTimingDesc = *reinterpret_cast<const SlamtecLidarTimingDesc*>(data);
    }
}

void UnpackerHandler_NormalNode::reset()
{
    _cached_scan_node_buf_pos = 0;
}
}


END_DATAUNPACKER_NS()