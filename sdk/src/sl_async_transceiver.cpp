/*
 *  Slamtec LIDAR SDK
 *
 *  Copyright (c) 2014 - 2023 Shanghai Slamtec Co., Ltd.
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

#include "sl_async_transceiver.h"



namespace sl { namespace internal {




ProtocolMessage::ProtocolMessage()
        : len(0)
        , cmd(0)
        , data(NULL)
        , _databufsize(0)
		, _usingOutterData(false)
{
    _changeBufSize();
}

ProtocolMessage::ProtocolMessage(_u8 cmd, const void* buffer, size_t size)
	: len(size)
	, cmd(cmd)
	, data(NULL)
    , _databufsize(0)
	, _usingOutterData(false)
{
    _changeBufSize();
	if (buffer)
	{
		memcpy(data, buffer, size);
	}
}

ProtocolMessage::ProtocolMessage(const ProtocolMessage& srcMsg)
	: len(srcMsg.len)
	, cmd(srcMsg.cmd)
	, data(NULL)
    , _databufsize(0)
	, _usingOutterData(false)
{
    _changeBufSize( true );
	if (srcMsg.data && len)
	{
		memcpy(data, srcMsg.data, len);
	}
}

ProtocolMessage::~ProtocolMessage()
{
	this->cleanData();
}

ProtocolMessage& ProtocolMessage::operator =(const ProtocolMessage& srcMessage)
{
	this->cleanData();


	this->len = srcMessage.len;
	this->cmd = srcMessage.cmd;

    _changeBufSize( true );
	if (srcMessage.data && len)
	{
		memcpy(data, srcMessage.data, len);
	}

	return *this;
}

void ProtocolMessage::setDataBuf(_u8 *buffer, size_t size)
{
	this->cleanData();

	len = size;
	data = buffer;
    _databufsize = size;
	_usingOutterData = true;
}

void ProtocolMessage::fillData(const void * buffer, size_t size)
{
	len = size;
    _changeBufSize();
    if (buffer)
	    memcpy(data, buffer, size);
}

void ProtocolMessage::cleanData()
{
	if (data) 
	{
		if (!_usingOutterData)
		{
			delete [] data;
		}
		data = NULL;
		len = 1;
        _databufsize = 0;
	}
}

void ProtocolMessage::_changeBufSize( bool force_compact)
{
    size_t actual_size  = getPayloadSize();

    size_t new_buf_size = actual_size;


    if (!_usingOutterData)
    {
        // nothing to do
        if ( new_buf_size == _databufsize ) return;

        if ( new_buf_size < _databufsize){

            if ( (_databufsize >> 1) < new_buf_size)
            {
                // reuse the current buffer
                if (!force_compact) return;
            }else
            {
                // the current buffer size is much bigger, we need to release it to save memory
            }
        }
    }

    // we need to change the buffer
    cleanData();
    // the cleanData() will reset the length info, so we need to restore it
    len = actual_size;
    data = new _u8[new_buf_size];
    _databufsize = new_buf_size;
}


AsyncTransceiver::AsyncTransceiver(IAsyncProtocolCodec& codec)
	: _bindedChannel(NULL)
	, _codec(codec)
	, _isWorking(false)
    , _workingFlag(0)
{

}

AsyncTransceiver::~AsyncTransceiver()
{
    unbindAndClose();
}

u_result AsyncTransceiver::openChannelAndBind(IChannel* channel)
{
    if (!channel) return RESULT_INVALID_DATA;

    unbindAndClose();
	u_result ans = RESULT_OK;
	do 
	{
		rp::hal::AutoLocker l(_opLocker);

        // try to open the channel ...
        Result<nullptr_t> ans = SL_RESULT_OK;

        if (!channel->open()) {
            ans= RESULT_OPERATION_FAIL;
            break;
        }


        // force a flush to clear any pending data
        channel->flush();

		_dataEvt.set(false);

		_isWorking = true;
        _workingFlag = 0;
        _bindedChannel = channel;


		_decoderThread = CLASS_THREAD(AsyncTransceiver, _proc_decoderThread);
		_rxThread = CLASS_THREAD(AsyncTransceiver, _proc_rxThread);

        


	} while (0);

	return ans;
}

void AsyncTransceiver::unbindAndClose()
{
	rp::hal::AutoLocker l(_opLocker);
	if (!_isWorking) return;

    assert(_bindedChannel);

    
	_isWorking = false;
	_dataEvt.set(); // set signal to wake up threads

	_decoderThread.join();
	_rxThread.join();


    _bindedChannel->close();

    _bindedChannel = NULL;


    for (std::list< Buffer* >::iterator itr = _rxQueue.begin(); itr != _rxQueue.end(); ++itr)
    {
        delete [] *itr;
    }
    _rxQueue.clear();

}

u_result AsyncTransceiver::sendMessage(message_autoptr_t& msg)
{
    assert(msg);

    if (!_isWorking) return RESULT_OPERATION_NOT_SUPPORT;

    rp::hal::AutoLocker l(_opLocker);

    size_t requiredBufferSize = _codec.estimateLength(msg);

    if (requiredBufferSize == 0) {
        // nothing to send
        return RESULT_OK;
    }

    u_result ans = RESULT_OK;

    _u8* txBuffer = new _u8[requiredBufferSize];

    do {
  
        if (!txBuffer) {
            return RESULT_INSUFFICIENT_MEMORY;
        }

        _codec.onEncodeData(msg, txBuffer, &requiredBufferSize);

        int txSize = _bindedChannel->write(txBuffer, requiredBufferSize);

        if (txSize < 0) ans = RESULT_OPERATION_FAIL;

    } while (0);


    delete[] txBuffer;
    return ans;
}

sl_result AsyncTransceiver::_proc_rxThread()
{
    assert(_bindedChannel);

    rp::hal::Thread::SetSelfPriority(rp::hal::Thread::PRIORITY_HIGH);

    u_result result;
    size_t hintedSize = 0;
    while (_isWorking)
    {
        result = _bindedChannel->waitForDataExt(hintedSize, 1000);

        if (IS_FAIL(result))
        {
            // timeout is allowed
            if (result == RESULT_OPERATION_TIMEOUT) {
                continue;
            }
            if (_isWorking) {
                _workingFlag |= WORKING_FLAG_ERROR;
                _codec.onChannelError(result);
                break;
            }
        }

        // no data in buffer, sleep and wait for the next round
        if (!hintedSize)
        {
            continue;
        }


        Buffer* decodeBuffer = new Buffer();
        
        decodeBuffer->data = new _u8[hintedSize];

        decodeBuffer->size = _bindedChannel->read(decodeBuffer->data, hintedSize);
#ifdef _DEBUG_DUMP_PACKET
        printf("Revc: %d\n", decodeBuffer->size);
#endif
         
        if  (!decodeBuffer->size) {
            delete decodeBuffer;

            
            _workingFlag |= WORKING_FLAG_ERROR;
            _codec.onChannelError(RESULT_OPERATION_ABORTED);
            break;
        }

        assert(hintedSize >= decodeBuffer->size);


#ifdef _DEBUG_DUMP_PACKET
        printf("=== Dump RX Packet, size = %d ===\n", decodeBuffer->size);
        for (int pos = 0; pos < decodeBuffer->size; pos++)
        {
            printf("%02x ", decodeBuffer->data[pos]);
        }
        printf("\n=== END ===\n");
#endif

        _rxLocker.lock();
        _rxQueue.push_back(decodeBuffer);
        _dataEvt.set();
        _rxLocker.unlock();


    }
    _workingFlag |= WORKING_FLAG_RX_DISABLED;
    return RESULT_OK;
}

sl_result AsyncTransceiver::_proc_decoderThread()
{

    assert(_bindedChannel);
    rp::hal::Thread::SetSelfPriority(rp::hal::Thread::PRIORITY_HIGH);
    _codec.onDecodeReset();
    

    while (_isWorking)
    {
        _rxLocker.lock();

        if (_rxQueue.empty())
        {
            _rxLocker.unlock();

            if (_dataEvt.wait(1000))
                continue;

            _rxLocker.lock();
        }
        assert(!_rxQueue.empty());

        Buffer * bufferToDecode = _rxQueue.front();
        _rxQueue.pop_front();

        _rxLocker.unlock();

        //cout<<"decoding "<< bufferToDecode->size <<" bytes of data"<<endl;
        _codec.onDecodeData(bufferToDecode->data, bufferToDecode->size);


        delete bufferToDecode;
    }

    return RESULT_OK;

}


}}