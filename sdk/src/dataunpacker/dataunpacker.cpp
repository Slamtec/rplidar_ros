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

#include "dataunnpacker_commondef.h"
#include "dataunpacker.h"
#include "dataunnpacker_internal.h"


#include <map>


#define REGISTER_HANDLER(_c_) {     \
		auto newBorn = new unpacker::_c_();   \
		if (!newBorn) return false; \
		handlerList.push_back(newBorn); \
	} 

// How to include new handlers?
// 1. add extra include line below if a new handle is to be included
// 2. update the code in function _registerDataUnpackerHandlers
#include "unpacker/handler_capsules.h"
#include "unpacker/handler_hqnode.h"
#include "unpacker/handler_normalnode.h"


#define  DEF_REGISTER_HANDLER_LIST


BEGIN_DATAUNPACKER_NS()


static bool _registerDataUnpackerHandlers(std::vector<IDataUnpackerHandler *> & handlerList)
{
	REGISTER_HANDLER(UnpackerHandler_NormalNode);
	REGISTER_HANDLER(UnpackerHandler_HQNode);
	REGISTER_HANDLER(UnpackerHandler_CapsuleNode);
	REGISTER_HANDLER(UnpackerHandler_UltraCapsuleNode);
	REGISTER_HANDLER(UnpackerHandler_DenseCapsuleNode);
	REGISTER_HANDLER(UnpackerHandler_UltraDenseCapsuleNode);
	return true;
}


class LIDARSampleDataUnpackerImpl : public LIDARSampleDataUnpackerInner
{
public:

	void registerHandler(_u8 ansType, IDataUnpackerHandler* handler)
	{
		_handlerMap[ansType] = handler;
	}


	void unregisterAllHandlers()
	{
		for (auto itr = _handlerMap.begin(); itr != _handlerMap.end(); ++itr)
		{
			delete itr->second;
		}
		_handlerMap.clear();
	}

	LIDARSampleDataUnpackerImpl(LIDARSampleDataListener& l)
		: LIDARSampleDataUnpackerInner(l)
		, _enabled(false)
		, _lastActiveAnsType(0)
		, _lastActiveHandler(nullptr)
	{

	}

	virtual ~LIDARSampleDataUnpackerImpl()
	{
		unregisterAllHandlers();
	}


	virtual void updateUnpackerContext(UnpackerContextType type, const void* data, size_t size)
	{
	
		// notify the handlers ...
		for (auto itr = _handlerMap.begin(); itr != _handlerMap.end(); ++itr)
		{
			itr->second->onUnpackerContextSet(type, data, size);
		}
	}

	virtual bool onSampleData(_u8 ansType, const void* buffer, size_t size) {
		if (!_enabled) return false;


		if (_lastActiveAnsType != ansType) {
			onDeselectHandler();

			auto itr = _handlerMap.find(ansType);
			if (itr != _handlerMap.end()) {
				onSelectHandler(ansType, itr->second);
			}
			else {
				onSelectHandler(ansType, nullptr);
			}
			
		}

		if (_lastActiveHandler) {
			_lastActiveHandler->onData(this, reinterpret_cast<const _u8 *>(buffer), size);
			return true;
		}
		else {
			return false;
		}
	}

	virtual void reset()
	{
		clearCache();
		_lastActiveHandler = nullptr;
		_lastActiveAnsType = 0;

	}

	virtual void enable()
	{
		_enabled = true;
		reset();
	}

	virtual void disable()
	{
		_enabled = false;
		reset();

	}

	virtual void clearCache()
	{
		if (_lastActiveHandler) {
			_lastActiveHandler->reset();
		}
	}

	virtual _u64 getCurrentTimestamp_uS() {
		return getus();
	}

	virtual void publishHQNode(_u64 timestamp_uS, const rplidar_response_measurement_node_hq_t* node)
	{
		_listener.onHQNodeDecoded(timestamp_uS, node);
	}


	virtual void publishDecodingErrorMsg(int errorType, _u8 ansType, const void* payload, size_t size)
	{
		_listener.onDecodingError(errorType, ansType, payload, size);

	}

	virtual void publishCustomData(_u8 ansType, _u32 customCode, const void* payload, size_t size)
	{
		_listener.onCustomSampleDataDecoded(ansType, customCode, payload, size);
	}


	virtual void publishNewScanReset()
	{
		_listener.onHQNodeScanResetReq();
	}
protected:

	void onSelectHandler(_u8 ansType, IDataUnpackerHandler* handler)
	{
		_lastActiveHandler = handler;
		_lastActiveAnsType = ansType;
	}

	void onDeselectHandler()
	{
		reset();
	}


protected:
	bool _enabled;
	std::map<_u8, IDataUnpackerHandler*> _handlerMap;

	_u8 _lastActiveAnsType;
	IDataUnpackerHandler* _lastActiveHandler;
};

LIDARSampleDataUnpacker* LIDARSampleDataUnpacker::CreateInstance(LIDARSampleDataListener& listener)
{
	LIDARSampleDataUnpackerImpl* impl = new LIDARSampleDataUnpackerImpl(listener);
	
	std::vector<IDataUnpackerHandler*> list;
	if (!_registerDataUnpackerHandlers(list)) {
		delete  impl;
		for (auto itr = list.begin(); itr != list.end(); ++itr) {
			delete* itr;
		}
		impl = nullptr;
	}

	for (auto itr = list.begin(); itr != list.end(); ++itr) {
		impl->registerHandler((*itr)->getSampleAnswerType(), (*itr));
	}
	return impl;
}

void LIDARSampleDataUnpacker::ReleaseInstance(LIDARSampleDataUnpacker* unpacker) {
	delete unpacker;
}

LIDARSampleDataUnpacker::~LIDARSampleDataUnpacker() {

}

LIDARSampleDataUnpacker::LIDARSampleDataUnpacker(LIDARSampleDataListener& l) 
	: _listener(l)
{

}


END_DATAUNPACKER_NS()