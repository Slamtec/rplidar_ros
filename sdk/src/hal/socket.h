/*
 *  RoboPeak Project
 *  HAL Layer - Socket Interface
 *  Copyright 2009 - 2013 RoboPeak Project
 */

#pragma once

#include <vector>

namespace rp{ namespace net {

class _single_thread SocketAddress 
{

public:
    enum address_type_t {
        ADDRESS_TYPE_UNSPEC = 0,
        ADDRESS_TYPE_INET   = 1,
        ADDRESS_TYPE_INET6  = 2,
    };

public:



    SocketAddress();
    SocketAddress(const char * addrString, int port, address_type_t = ADDRESS_TYPE_INET);
    // do not use this function, internal usage
    SocketAddress(void * platform_data);
    SocketAddress(const SocketAddress &);
    
    SocketAddress & operator = (const SocketAddress &);

    virtual ~SocketAddress();
    
    virtual int getPort() const;
    virtual u_result setPort(int port);

    virtual u_result setAddressFromString(const char * address_string,  address_type_t = ADDRESS_TYPE_INET);
    virtual u_result getAddressAsString(char * buffer, size_t buffersize) const;
    
    virtual address_type_t getAddressType() const; 

    virtual u_result getRawAddress(_u8 * buffer, size_t bufferSize) const;

    const void * getPlatformData() const {
        return _platform_data;
    }

    virtual void setLoopbackAddress(address_type_t = ADDRESS_TYPE_INET);
    virtual void setBroadcastAddressIPv4();
    virtual void setAnyAddress(address_type_t = ADDRESS_TYPE_INET);

public:
    static size_t LoopUpHostName(const char * hostname, const char * sevicename, std::vector<SocketAddress> &addresspool , bool performDNS = true, address_type_t = ADDRESS_TYPE_INET);

protected:
    void * _platform_data;
};



class SocketBase 
{
public:
    enum socket_family_t {
        SOCKET_FAMILY_INET  = 0,
        SOCKET_FAMILY_INET6 = 1,
        SOCKET_FAMILY_RAW   = 2,
    };


    enum socket_direction_mask {
        SOCKET_DIR_RD = 0x1,
        SOCKET_DIR_WR = 0x2,
        SOCKET_DIR_BOTH = (SOCKET_DIR_RD | SOCKET_DIR_WR),
    };

    enum {
        DEFAULT_SOCKET_TIMEOUT = 10000, //10sec
    };

    virtual ~SocketBase() {}
    virtual void dispose() = 0;
    virtual u_result bind(const SocketAddress & ) = 0;
    
    virtual u_result getLocalAddress(SocketAddress & ) = 0;
    virtual u_result setTimeout(_u32 timeout, socket_direction_mask msk = SOCKET_DIR_BOTH) = 0;

    virtual u_result waitforSent(_u32 timeout  = DEFAULT_SOCKET_TIMEOUT) = 0;
    virtual u_result waitforData(_u32 timeout  = DEFAULT_SOCKET_TIMEOUT)  = 0;
protected:
    SocketBase() {} 
};


class _single_thread StreamSocket : public SocketBase
{
public:

    enum {
        MAX_BACKLOG = 128,
    };

    static StreamSocket * CreateSocket(socket_family_t family = SOCKET_FAMILY_INET);
    
    virtual u_result connect(const SocketAddress & pairAddress) = 0;
    
    virtual u_result listen(int backlog = MAX_BACKLOG) = 0;
    virtual StreamSocket * accept(SocketAddress * pairAddress = NULL) = 0;
    virtual u_result waitforIncomingConnection(_u32 timeout  = DEFAULT_SOCKET_TIMEOUT) = 0;

    virtual u_result send(const void * buffer, size_t len) = 0;
    
    virtual u_result recv(void *buf, size_t len, size_t & recv_len) = 0;
    
    virtual u_result getPeerAddress(SocketAddress & ) = 0;

    virtual u_result shutdown(socket_direction_mask mask) = 0;

    virtual u_result enableKeepAlive(bool enable = true) = 0;
    
    virtual u_result enableNoDelay(bool enable = true) = 0;

protected:
    virtual ~StreamSocket() {} // use dispose();
    StreamSocket() {}
};

class _single_thread DGramSocket: public SocketBase
{

public:

    static DGramSocket * CreateSocket(socket_family_t family = SOCKET_FAMILY_INET);
        
    
    
    virtual u_result sendTo(const SocketAddress & target, const void * buffer, size_t len) = 0;
   
    virtual u_result recvFrom(void *buf, size_t len, size_t & recv_len, SocketAddress * sourceAddr = NULL) = 0;

    
protected:
    virtual ~DGramSocket() {} // use dispose();

    DGramSocket() {}
};

}}
