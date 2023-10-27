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
#include "hal/byteorder.h"
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

#include "sl_async_transceiver.h"
#include "sl_lidarprotocol_codec.h"



namespace sl { namespace internal {



RPLidarProtocolCodec::RPLidarProtocolCodec()
    : IAsyncProtocolCodec()
    , _listener(NULL)
    , _op_locker(true)
{
    onDecodeReset();
}

void RPLidarProtocolCodec::exitLoopMode() {
    onDecodeReset();
}



void RPLidarProtocolCodec::setMessageListener(IProtocolMessageListener* listener)
{
    rp::hal::AutoLocker l(_op_locker);
    _listener = listener;
}

size_t RPLidarProtocolCodec::estimateLength(message_autoptr_t& message)
{
    size_t actualSize = 2; //1-byte's sync byte, 1-byte's cmd byte

    if (message->cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        actualSize += (message->getPayloadSize() & 0xFF);
        actualSize += 2; //1-byte for size field, 1-byte for checksum
    }

    return actualSize;
}


void RPLidarProtocolCodec::onEncodeData(message_autoptr_t& message, _u8* buffer, size_t* size)
{
    _u8 checksum = 0;
    size_t writeSize = std::min<size_t>(*size, estimateLength(message));
    size_t currentPos = 0;

    while (currentPos < writeSize) {
        _u8 currentTxByte;
        switch (currentPos) {
        case 0: // sync byte
            currentTxByte = RPLIDAR_CMD_SYNC_BYTE;
            break;
        case 1: // cmd byte
            currentTxByte = message->cmd;
            break;
        case 2: // size byte
            currentTxByte = (_u8)message->getPayloadSize();
            break;
        default:
        {
            size_t payloadPos = currentPos - 3;
            if (payloadPos == message->getPayloadSize()) {
                // checksum byte
                currentTxByte = checksum;
                assert(currentPos + 1 == writeSize);
            }
            else {
                // payload
                currentTxByte = message->getDataBuf()[payloadPos];
            }
        }
        }


        checksum ^= currentTxByte;
        buffer[currentPos++] = currentTxByte;
    } while (0);

    *size = currentPos;
}

void   RPLidarProtocolCodec::onDecodeReset() {
    rp::hal::AutoLocker autolock(_op_locker);
    // flush the pending data
    _decodingMessage.cleanData();
    // reset to initial state
    _rx_pos = 0;
    _working_states = STATUS_WAIT_SYNC1;
}


void RPLidarProtocolCodec::onDecodeData(const void* buffer, size_t size)
{
    rp::hal::AutoLocker autolock(_op_locker);

    const _u8* data = reinterpret_cast<const _u8*>(buffer);
    const _u8* dataEnd = data + size;


    while (data != dataEnd) {
        _u8 currentByte = *data;
        ++data;

        switch (_working_states & ((_u32)STATUS_LOOP_MODE_FLAG - 1)) {
        case STATUS_WAIT_SYNC1:
            if (currentByte == RPLIDAR_ANS_SYNC_BYTE1) {
                _working_states = STATUS_WAIT_SYNC2;
            }
            break;
        case STATUS_WAIT_SYNC2:
            if (currentByte == RPLIDAR_ANS_SYNC_BYTE2) {
                _working_states = STATUS_WAIT_SIZE_FLAG;
                _rx_pos = 0; // init rx pos for recv size and flag
            }
            else {
                // reset to the initial state
                _working_states = STATUS_WAIT_SYNC1;
            }
            break;
        case STATUS_WAIT_SIZE_FLAG:
        {
            assert(sizeof(_decodingMessage.len) >= 4);
            _u8* byteArr = reinterpret_cast<_u8*>(&_decodingMessage.len);
            byteArr[_rx_pos++] = currentByte;

            if (_rx_pos == 4) {
                _working_states = STATUS_WAIT_TYPE;
                _decodingMessage.len = le32_to_cpu(_decodingMessage.len);

                // 30bit size + 2bit flag has been received
                _u32 flagbits = (_u32)(_decodingMessage.len >> RPLIDAR_ANS_HEADER_SUBTYPE_SHIFT);
                if (flagbits & RPLIDAR_ANS_PKTFLAG_LOOP) {
                    _working_states |= STATUS_LOOP_MODE_FLAG;
                }
                _decodingMessage.len = (_decodingMessage.len & RPLIDAR_ANS_HEADER_SIZE_MASK);
                // alloc buffer
                _decodingMessage.fillData(NULL, _decodingMessage.getPayloadSize());
                _rx_pos = 0;
            }
        }
        break;
        case STATUS_WAIT_TYPE:
            // save the type field as a cmd 
            _decodingMessage.cmd = currentByte;

            // recv payload...
            _working_states = (_working_states & STATUS_LOOP_MODE_FLAG)
                | STATUS_RECV_PAYLOAD;

            if (!_decodingMessage.getPayloadSize()) {
                // zero payload packet? 
                _working_states = STATUS_WAIT_SYNC1;
            }
            break;
        case STATUS_RECV_PAYLOAD:
            _decodingMessage.getDataBuf()[_rx_pos++] = currentByte;

            if ((size_t)_rx_pos == _decodingMessage.getPayloadSize()) {
                if (_working_states & STATUS_LOOP_MODE_FLAG) {
                    // rewind to the payload recv status in loop mode
                    _rx_pos = 0;
                }
                else {
                    // reset the decoder
                    _working_states = STATUS_WAIT_SYNC1;
                }

                IProtocolMessageListener* cachedLister = _listener;

                autolock.forceUnlock(); //unlock the oplock to prevent deadlock


                if (cachedLister) {
                    cachedLister->onProtocolMessageDecoded(_decodingMessage);
                }

                _op_locker.lock(); // relock it
            }
            break;
        }

    }
}




}}