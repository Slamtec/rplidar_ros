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

#pragma once

BEGIN_DATAUNPACKER_NS()

namespace unpacker {

class UnpackerHandler_CapsuleNode : public IDataUnpackerHandler {
public:
	UnpackerHandler_CapsuleNode();
	virtual ~UnpackerHandler_CapsuleNode();

	virtual _u8 getSampleAnswerType() const;
	virtual void onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t size);
	virtual void reset();
	virtual void onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size);
protected:

	void _onScanNodeCapsuleData(rplidar_response_capsule_measurement_nodes_t &, LIDARSampleDataUnpackerInner* engine);

	std::vector<_u8> _cached_scan_node_buf;
	int              _cached_scan_node_buf_pos;
	bool             _is_previous_capsuledataRdy;

	rplidar_response_capsule_measurement_nodes_t _cached_previous_capsuledata;
	_u64             _cached_last_data_timestamp_us;

	SlamtecLidarTimingDesc _cachedTimingDesc;
};

class UnpackerHandler_UltraCapsuleNode : public IDataUnpackerHandler {
public:
	UnpackerHandler_UltraCapsuleNode();
	virtual ~UnpackerHandler_UltraCapsuleNode();

	virtual _u8 getSampleAnswerType() const;
	virtual void onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t size);
	virtual void reset();
	virtual void onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size);
protected:
	void _onScanNodeUltraCapsuleData(rplidar_response_ultra_capsule_measurement_nodes_t&, LIDARSampleDataUnpackerInner* engine);


	std::vector<_u8> _cached_scan_node_buf;
	int              _cached_scan_node_buf_pos;
	bool             _is_previous_capsuledataRdy;

	rplidar_response_ultra_capsule_measurement_nodes_t _cached_previous_ultracapsuledata;
	_u64             _cached_last_data_timestamp_us;

	SlamtecLidarTimingDesc _cachedTimingDesc;

};



class UnpackerHandler_DenseCapsuleNode : public IDataUnpackerHandler {
public:
	UnpackerHandler_DenseCapsuleNode();
	virtual ~UnpackerHandler_DenseCapsuleNode();

	virtual _u8 getSampleAnswerType() const;
	virtual void onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t size);
	virtual void reset();
	virtual void onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size);
protected:
	void _onScanNodeDenseCapsuleData(rplidar_response_dense_capsule_measurement_nodes_t&, LIDARSampleDataUnpackerInner* engine);


	std::vector<_u8> _cached_scan_node_buf;
	int              _cached_scan_node_buf_pos;
	bool             _is_previous_capsuledataRdy;

	rplidar_response_dense_capsule_measurement_nodes_t _cached_previous_dense_capsuledata;
	_u64             _cached_last_data_timestamp_us;

	SlamtecLidarTimingDesc _cachedTimingDesc;

};


class UnpackerHandler_UltraDenseCapsuleNode : public IDataUnpackerHandler {
public:
	UnpackerHandler_UltraDenseCapsuleNode();
	virtual ~UnpackerHandler_UltraDenseCapsuleNode();

	virtual _u8 getSampleAnswerType() const;
	virtual void onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t size);
	virtual void reset();
	virtual void onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size);
protected:
	void _onScanNodeUltraDenseCapsuleData(rplidar_response_ultra_dense_capsule_measurement_nodes_t&, LIDARSampleDataUnpackerInner* engine);

	std::vector<_u8> _cached_scan_node_buf;
	int              _cached_scan_node_buf_pos;
	bool             _is_previous_capsuledataRdy;

	rplidar_response_ultra_dense_capsule_measurement_nodes_t _cached_previous_ultra_dense_capsuledata;
	_u64             _cached_last_data_timestamp_us;



	int              _last_node_sync_bit;
	int              _last_dist_q2;

	SlamtecLidarTimingDesc _cachedTimingDesc;
};


}

END_DATAUNPACKER_NS()