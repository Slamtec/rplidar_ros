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


#pragma once

#include "sl_async_transceiver.h"

namespace sl { namespace internal {


class IProtocolMessageListener {
public:
    virtual void onProtocolMessageDecoded(const ProtocolMessage&) = 0;
};


class RPLidarProtocolCodec : public IAsyncProtocolCodec
{
public:

    enum {
        STATUS_WAIT_SYNC1 = 0x0,
        STATUS_WAIT_SYNC2 = 0x1,
        STATUS_WAIT_SIZE_FLAG = 0x2,
        STATUS_WAIT_TYPE = 0x3,
        STATUS_RECV_PAYLOAD = 0x4,
        STATUS_LOOP_MODE_FLAG = 0x80000000,
    };

    RPLidarProtocolCodec();

    void exitLoopMode();


    virtual size_t estimateLength(message_autoptr_t& message);


    virtual void onEncodeData(message_autoptr_t& message, _u8* txbuffer, size_t* size);

    virtual void   onDecodeReset();
    virtual void   onDecodeData(const void* buffer, size_t size);
    
    void setMessageListener(IProtocolMessageListener* l);

protected:

    IProtocolMessageListener* _listener;
    ProtocolMessage          _decodingMessage;
    rp::hal::Locker          _op_locker;
                            
    _u32                     _working_states;
    int                      _rx_pos;
};

}}



