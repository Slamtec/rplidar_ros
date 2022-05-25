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

#pragma once
#include "sl_lidar_driver.h"
#include "hal/abs_rxtx.h"
#include "hal/socket.h"


namespace sl {
	class UdpChannel : public IChannel
	{
	public:
		UdpChannel(const std::string& ip, int port) : _binded_socket(rp::net::DGramSocket::CreateSocket()) {
            _ip = ip;
            _port = port;
        }

		bool bind(const std::string & ip, sl_s32 port)
        {
            _socket = rp::net::SocketAddress(ip.c_str(), port);
            return SL_RESULT_OK;
        }

        bool open()
        {
            if(SL_IS_FAIL(bind(_ip, _port)))
                return false;
            return SL_IS_OK(_binded_socket->setPairAddress(&_socket));         
        }

        void close()
        {
            _binded_socket->dispose();
            _binded_socket = NULL;
        }
        void flush()
        {
        
        }

		bool waitForData(size_t size, sl_u32 timeoutInMs, size_t* actualReady)
        {
            if (actualReady)
                *actualReady = size;
            return (_binded_socket->waitforData(timeoutInMs) == RESULT_OK);

        }

        int write(const void* data, size_t size)
        {
            return _binded_socket->sendTo(_socket, data, size);
        }

        int read(void* buffer, size_t size)
        {
            u_result ans;
            size_t recCnt = 0;
            size_t lenRec = 0;
            while (size > recCnt)
            {
				sl_u8 *temp = (sl_u8 *)buffer+recCnt;
                ans = _binded_socket->recvFrom(temp, size, lenRec);
                recCnt += lenRec;
                if (ans)
                    break;
            }
            return recCnt;
        
        }

        void clearReadCache() {}

        void setStatus(_u32 flag){}

	private:
		rp::net::DGramSocket * _binded_socket;
		rp::net::SocketAddress _socket;
        std::string _ip;
        int _port;
	};

    Result<IChannel*> createUdpChannel(const std::string& ip, int port)
    {
        return new  UdpChannel(ip, port);
    }
}