/*
 *  Slamtec LIDAR SDK
 *
 *  Copyright (c) 2014 - 2023 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

 /*
  *  Sample Data Unpacker System
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

#include "dataupacker_namespace.h"

BEGIN_DATAUNPACKER_NS()


class LIDARSampleDataListener
{


public:
	virtual void onHQNodeScanResetReq() = 0;
	virtual void onHQNodeDecoded(_u64 timestamp_uS, const rplidar_response_measurement_node_hq_t* node) = 0;
	virtual void onCustomSampleDataDecoded(_u8 ansType, _u32 customCode, const void* data, size_t size) {}

	virtual void onDecodingError(int errMsg, _u8 ansType, const void* payload, size_t size) {}
};

class LIDARSampleDataUnpacker
{
public:
	enum {
		ERR_EVENT_ON_EXP_ENCODER_RESET = 0x8001,
		ERR_EVENT_ON_EXP_CHECKSUM_ERR = 0x8002,
	};

	enum UnpackerContextType {
		UNPACKER_CONTEXT_TYPE_LIDAR_UNKNOWN = 0,
		UNPACKER_CONTEXT_TYPE_LIDAR_TIMING = 1,
		UNPACKER_CONTEXT_TYPE_TRIANGULATION_OPTICAL_FACTOR = 2,

	};

	virtual ~LIDARSampleDataUnpacker();
	static LIDARSampleDataUnpacker* CreateInstance(LIDARSampleDataListener& listener);
	static void ReleaseInstance(LIDARSampleDataUnpacker*);

	virtual void updateUnpackerContext(UnpackerContextType type, const void* data, size_t size) = 0;

	virtual void enable() = 0;
	virtual void disable() = 0;

	virtual bool onSampleData(_u8 ansType, const void* buffer, size_t size) = 0;
	virtual void reset() = 0;
	virtual void clearCache() = 0;

protected:
	LIDARSampleDataUnpacker(LIDARSampleDataListener&);
	LIDARSampleDataListener& _listener;

};

END_DATAUNPACKER_NS()