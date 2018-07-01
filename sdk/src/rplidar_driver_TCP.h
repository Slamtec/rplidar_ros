/*
 *  RPLIDAR SDK
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2018 Shanghai Slamtec Co., Ltd.
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

namespace rp { namespace standalone{ namespace rplidar {

class TCPChannelDevice :public ChannelDevice
{
public:
    rp::net::StreamSocket * _binded_socket;
    TCPChannelDevice():_binded_socket(rp::net::StreamSocket::CreateSocket()){}

    bool bind(const char * ipStr, uint32_t port)
    {
        rp::net::SocketAddress socket(ipStr, port);
        return IS_OK(_binded_socket->connect(socket));
    }
    void close()
    {
        _binded_socket->dispose();
        _binded_socket = NULL;
    }
    bool waitfordata(size_t data_count,_u32 timeout = -1, size_t * returned_size = NULL)
    {
        if(returned_size)
            *returned_size = data_count;
        return (_binded_socket->waitforData(timeout) == RESULT_OK);
    }
    int senddata(const _u8 * data, size_t size)
    {
        return _binded_socket->send(data, size) ;
    }
    int recvdata(unsigned char * data, size_t size)
    {
        size_t lenRec = 0;
        _binded_socket->recv(data, size, lenRec);
        return lenRec;
    }
};


class RPlidarDriverTCP : public RPlidarDriverImplCommon
{
public:

    RPlidarDriverTCP();
    virtual ~RPlidarDriverTCP();
    virtual u_result connect(const char * ipStr, _u32 port, _u32 flag = 0);
    virtual void disconnect();
};


}}}