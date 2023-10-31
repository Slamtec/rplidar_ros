/*
 *  Slamtec LIDAR SDK
 *
 *  Copyright (c) 2014 - 2023 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

 /*
  *  Sample Data Unpacker System
  *  HQNode Sample Node Handler
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

#ifdef CONF_NO_BOOST_CRC_SUPPORT
#include "sl_crc.h" 
#endif

#include "handler_hqnode.h"

BEGIN_DATAUNPACKER_NS()
	
namespace unpacker{


static _u64 _getSampleDelayOffsetInHQMode(const SlamtecLidarTimingDesc& timing)
{
    // FIXME: to eval
    // 
    // guess channel baudrate by LIDAR model ....
    const _u64 channelBaudRate = timing.native_baudrate? timing.native_baudrate:1000000;

    _u64 tranmissionDelay = 1000000ULL * sizeof(rplidar_response_measurement_node_hq_t) * 10 / channelBaudRate;

    if (timing.native_interface_type == LIDARInterfaceType::LIDAR_INTERFACE_ETHERNET)
    {
        tranmissionDelay = 100; //dummy value
    }

    // center of the sample duration
    const _u64 sampleDelay = (timing.sample_duration_uS >> 1);
    const _u64 sampleFilterDelay = timing.sample_duration_uS;

    return sampleFilterDelay + sampleDelay + tranmissionDelay + timing.linkage_delay_uS;
}

UnpackerHandler_HQNode::UnpackerHandler_HQNode()
    : _cached_scan_node_buf_pos(0)
{
    _cached_scan_node_buf.resize(sizeof(rplidar_response_hq_capsule_measurement_nodes_t));
    memset(&_cachedTimingDesc, 0, sizeof(_cachedTimingDesc));
}

UnpackerHandler_HQNode::~UnpackerHandler_HQNode()
{

}

_u8 UnpackerHandler_HQNode::getSampleAnswerType() const
{
	return RPLIDAR_ANS_TYPE_MEASUREMENT_HQ;
}

void UnpackerHandler_HQNode::onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t cnt)
{

    for (size_t pos = 0; pos < cnt; ++pos)
    {
        _u8 current_data = data[pos];

        switch (_cached_scan_node_buf_pos)
        {
        case 0: // expect the sync byte
        {
            if (current_data == RPLIDAR_RESP_MEASUREMENT_HQ_SYNC) {
                // pass
            }
            else {
                continue;
            }
        }
        break;

        case sizeof(rplidar_response_hq_capsule_measurement_nodes_t) - 1 - 4:    // get bytes to calculate crc ready
        {
           
        }
        break;

        case sizeof(rplidar_response_hq_capsule_measurement_nodes_t) - 1: // new data ready
        {
            _cached_scan_node_buf[sizeof(rplidar_response_hq_capsule_measurement_nodes_t) - 1] = current_data;
            _cached_scan_node_buf_pos = 0;
            rplidar_response_hq_capsule_measurement_nodes_t* nodesData = reinterpret_cast<rplidar_response_hq_capsule_measurement_nodes_t*>(&_cached_scan_node_buf[0]);

#ifdef CONF_NO_BOOST_CRC_SUPPORT
            _u32 crcCalc = crc32::getResult(&_cached_scan_node_buf[0], sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t) - 4);


#else
            // calculate crc with boost crc method
            boost::crc_optimal<32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true> mycrc;
            std::vector<_u8> crcInputData;
            crcInputData.resize(sizeof(rplidar_response_hq_capsule_measurement_nodes_t) - 4);
            memcpy(&crcInputData[0], nodesData, sizeof(rplidar_response_hq_capsule_measurement_nodes_t) - 4);
            //supplement crcInputData to mutiples of 4
            int leftBytes = 4 - (crcInputData.size() & 3);
            for (int i = 0; i < leftBytes; i++)
                crcInputData.push_back(0);
            mycrc.process_bytes(&crcInputData[0], crcInputData.size());
            _u32 crcCalc = mycrc.checksum();
            
#endif

            _u32 recvCRC = nodesData->crc32;
#ifdef _CPU_ENDIAN_BIG
            recvCRC = le32_to_cpu(recvCRC);
            nodesData->time_stamp = le64_to_cpu(nodesData->time_stamp);
#endif
            if (recvCRC == crcCalc)
            {
                for (size_t pos = 0; pos < _countof(nodesData->node_hq); ++pos)
                {
                    rplidar_response_measurement_node_hq_t hqNode = nodesData->node_hq[pos];
#ifdef _CPU_ENDIAN_BIG
                    hqNode.angle_z_q14 = le16_to_cpu(hqNode.angle_z_q14);
                    hqNode.dist_mm_q2 = le32_to_cpu(hqNode.dist_mm_q2);
#endif
                    engine->publishHQNode(engine->getCurrentTimestamp_uS() - _getSampleDelayOffsetInHQMode(_cachedTimingDesc), &hqNode);
                }
            }
            else  //crc check not passed 
            {
                engine->publishDecodingErrorMsg(LIDARSampleDataUnpacker::ERR_EVENT_ON_EXP_CHECKSUM_ERR
                    , RPLIDAR_ANS_TYPE_MEASUREMENT_HQ, nodesData, sizeof(*nodesData));
            }
            continue;
        }
        break;


        }
        _cached_scan_node_buf[_cached_scan_node_buf_pos++] = current_data;
    }

}


void UnpackerHandler_HQNode::onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size)
{
    if (type == LIDARSampleDataUnpacker::UNPACKER_CONTEXT_TYPE_LIDAR_TIMING) {
        assert(size == sizeof(_cachedTimingDesc));
        _cachedTimingDesc = *reinterpret_cast<const SlamtecLidarTimingDesc*>(data);
    }
}

void UnpackerHandler_HQNode::reset()
{
    _cached_scan_node_buf_pos = 0;
}
}


END_DATAUNPACKER_NS()