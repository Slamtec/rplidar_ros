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

#include "arch/linux/arch_linux.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
// linux specific

#include <errno.h>
#include <fcntl.h>

#include <time.h>
#include "hal/types.h"
#include "arch/linux/net_serial.h"
#include <sys/select.h>

#include <algorithm>
//__GNUC__
#if defined(__GNUC__)
// for Linux extension
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
extern "C" int tcflush(int fildes, int queue_selector);
#else
// for other standard UNIX
#include <termios.h>
#include <sys/ioctl.h>

#endif


namespace rp{ namespace arch{ namespace net{

raw_serial::raw_serial()
    : rp::hal::serial_rxtx()
    , _baudrate(0)
    , _flags(0)
    , serial_fd(-1)
{
    _init();
}

raw_serial::~raw_serial()
{
    close();

}

bool raw_serial::open()
{
    return open(_portName, _baudrate, _flags);
}

bool raw_serial::bind(const char * portname, uint32_t baudrate, uint32_t flags)
{   
    strncpy(_portName, portname, sizeof(_portName));
    _baudrate = baudrate;
    _flags    = flags;
    return true;
}

bool raw_serial::open(const char * portname, uint32_t baudrate, uint32_t flags)
{
    if (isOpened()) close();
    
    serial_fd = ::open(portname, O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_fd == -1) return false;

    

#if !defined(__GNUC__)
    // for standard UNIX
    struct termios options, oldopt;
    tcgetattr(serial_fd, &oldopt);
    bzero(&options,sizeof(struct termios));

    // enable rx and tx
    options.c_cflag |= (CLOCAL | CREAD);

    _u32 termbaud = getTermBaudBitmap(baudrate);

    if (termbaud == (_u32)-1) {
        close();
        return false;
    }
    cfsetispeed(&options, termbaud);
    cfsetospeed(&options, termbaud);

    options.c_cflag &= ~PARENB; //no checkbit
    options.c_cflag &= ~CSTOPB; //1bit stop bit
    options.c_cflag &= ~CRTSCTS; //no flow control

    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; /* Select 8 data bits */

#ifdef CNEW_RTSCTS
    options.c_cflag &= ~CNEW_RTSCTS; // no hw flow control
#endif

    options.c_iflag &= ~(IXON | IXOFF | IXANY); // no sw flow control

    // raw input mode   
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // raw output mode   
    options.c_oflag &= ~OPOST;



    if (tcsetattr(serial_fd, TCSANOW, &options))
    {
        close();
        return false;
    }

#else

    // using Linux extension ...
    struct termios2 tio;

    ioctl(serial_fd, TCGETS2, &tio);
    bzero(&tio, sizeof(struct termios2));

    tio.c_cflag = BOTHER;
    tio.c_cflag |= (CLOCAL | CREAD | CS8); //8 bit no hardware handshake

    tio.c_cflag &= ~CSTOPB;   //1 stop bit
    tio.c_cflag &= ~CRTSCTS;  //No CTS
    tio.c_cflag &= ~PARENB;   //No Parity

#ifdef CNEW_RTSCTS
    tio.c_cflag &= ~CNEW_RTSCTS; // no hw flow control
#endif

    tio.c_iflag &= ~(IXON | IXOFF | IXANY); // no sw flow control


    tio.c_cc[VMIN] = 0;         //min chars to read
    tio.c_cc[VTIME] = 0;        //time in 1/10th sec wait

    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // raw output mode   
    tio.c_oflag &= ~OPOST;

    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;


    ioctl(serial_fd, TCSETS2, &tio);

#endif


    tcflush(serial_fd, TCIFLUSH);

    if (fcntl(serial_fd, F_SETFL, FNDELAY))
    {
        close();
        return false;
    }


    _is_serial_opened = true;
    _operation_aborted = false;

    //Clear the DTR bit to let the motor spin
    clearDTR();
    do {
        // create self pipeline for wait cancellation
        if (pipe(_selfpipe) == -1) break;

        int flags = fcntl(_selfpipe[0], F_GETFL);
        if (flags == -1)
            break;

        flags |= O_NONBLOCK;                /* Make read end nonblocking */
        if (fcntl(_selfpipe[0], F_SETFL, flags) == -1)
            break;

        flags = fcntl(_selfpipe[1], F_GETFL);
        if (flags == -1)
            break;

        flags |= O_NONBLOCK;                /* Make write end nonblocking */
        if (fcntl(_selfpipe[1], F_SETFL, flags) == -1)
            break;

    } while (0);
    
    return true;
}

void raw_serial::close()
{
    if (serial_fd != -1)
        ::close(serial_fd);
    serial_fd = -1;
    
    if (_selfpipe[0] != -1)
        ::close(_selfpipe[0]);

    if (_selfpipe[1] != -1)
        ::close(_selfpipe[1]);

    _selfpipe[0] = _selfpipe[1] = -1;

    _operation_aborted = false;
    _is_serial_opened = false;
}

int raw_serial::senddata(const unsigned char * data, size_t size)
{
// FIXME: non-block io should be used
    if (!isOpened()) return 0;

    if (data == NULL || size ==0) return 0;
    
    size_t tx_len = 0;
    required_tx_cnt = 0;
    do {
        int ans = ::write(serial_fd, data + tx_len, size-tx_len);
        
        if (ans == -1) return tx_len;
        
        tx_len += ans;
        required_tx_cnt = tx_len;
    }while (tx_len<size);
    
    
    return tx_len;
}


int raw_serial::recvdata(unsigned char * data, size_t size)
{
    if (!isOpened()) return 0;
    
    int ans = ::read(serial_fd, data, size);
    
    if (ans == -1) ans=0;
    required_rx_cnt = ans;
    return ans;
}


void raw_serial::flush( _u32 flags)
{
    tcflush(serial_fd,TCIFLUSH); 
}

int raw_serial::waitforsent(_u32 timeout, size_t * returned_size)
{
    if (returned_size) *returned_size = required_tx_cnt;
    return 0;
}

int raw_serial::waitforrecv(_u32 timeout, size_t * returned_size)
{
    if (!isOpened() ) return -1;
   
    if (returned_size) *returned_size = required_rx_cnt;
    return 0;
}

int raw_serial::waitfordata(size_t data_count, _u32 timeout, size_t * returned_size)
{
    size_t length = 0;
    if (returned_size==NULL) returned_size=(size_t *)&length;
    *returned_size = 0;

    int max_fd;
    fd_set input_set;
    struct timeval timeout_val;

    /* Initialize the input set */
    FD_ZERO(&input_set);
    FD_SET(serial_fd, &input_set);

    if (_selfpipe[0] != -1)
        FD_SET(_selfpipe[0], &input_set);

    max_fd =  std::max<int>(serial_fd, _selfpipe[0]) + 1;

    /* Initialize the timeout structure */
    timeout_val.tv_sec = timeout / 1000;
    timeout_val.tv_usec = (timeout % 1000) * 1000;

    if ( isOpened() )
    {
        if ( ioctl(serial_fd, FIONREAD, returned_size) == -1) return ANS_DEV_ERR;
        if (*returned_size >= data_count)
        {
            return 0;
        }
    }

    while ( isOpened() )
    {
        /* Do the select */
        int n = ::select(max_fd, &input_set, NULL, NULL, &timeout_val);

        if (n < 0)
        {
            // select error
            *returned_size =  0;
            return ANS_DEV_ERR;
        }
        else if (n == 0)
        {
            // time out
            *returned_size =0;
            return ANS_TIMEOUT;
        }
        else
        {
            if (FD_ISSET(_selfpipe[0], &input_set)) {   
                // require aborting the current operation
                int ch;
                for (;;) {                    
                    if (::read(_selfpipe[0], &ch, 1) == -1) {
                        break;
                    }
                    
                }

                // treat as  timeout
                *returned_size = 0;
                return ANS_TIMEOUT;
            }

            // data avaliable
            assert (FD_ISSET(serial_fd, &input_set));


            if ( ioctl(serial_fd, FIONREAD, returned_size) == -1) return ANS_DEV_ERR;
            if (*returned_size >= data_count)
            {
                return 0;
            }
            else 
            {
                int remain_timeout = timeout_val.tv_sec*1000000 + timeout_val.tv_usec;
                int expect_remain_time = (data_count - *returned_size)*1000000*8/_baudrate;
                if (remain_timeout > expect_remain_time)
                    usleep(expect_remain_time);
            }
        }
        
    }

    return ANS_DEV_ERR;
}

size_t raw_serial::rxqueue_count()
{
    if  ( !isOpened() ) return 0;
    size_t remaining;
    
    if (::ioctl(serial_fd, FIONREAD, &remaining) == -1) return 0;
    return remaining;
}

void raw_serial::setDTR()
{
    if ( !isOpened() ) return;

    uint32_t dtr_bit = TIOCM_DTR;
    ioctl(serial_fd, TIOCMBIS, &dtr_bit);
}

void raw_serial::clearDTR()
{
    if ( !isOpened() ) return;

    uint32_t dtr_bit = TIOCM_DTR;
    ioctl(serial_fd, TIOCMBIC, &dtr_bit);
}

void raw_serial::_init()
{
    serial_fd = -1;  
    _portName[0] = 0;
    required_tx_cnt = required_rx_cnt = 0;
    _operation_aborted = false;
    _selfpipe[0] = _selfpipe[1] = -1;
}

void raw_serial::cancelOperation()
{
    _operation_aborted = true;
    if (_selfpipe[1] == -1) return;

    ::write(_selfpipe[1], "x", 1);
}

_u32 raw_serial::getTermBaudBitmap(_u32 baud)
{
#define BAUD_CONV( _baud_) case _baud_:  return B##_baud_ 
switch (baud) {
        BAUD_CONV(1200);
        BAUD_CONV(1800);
        BAUD_CONV(2400);
        BAUD_CONV(4800);
        BAUD_CONV(9600);
        BAUD_CONV(19200);
        BAUD_CONV(38400);
        BAUD_CONV(57600);
        BAUD_CONV(115200);
        BAUD_CONV(230400);
        BAUD_CONV(460800);
        BAUD_CONV(500000);
        BAUD_CONV(576000);
        BAUD_CONV(921600);
        BAUD_CONV(1000000);
        BAUD_CONV(1152000);
        BAUD_CONV(1500000);
        BAUD_CONV(2000000);
        BAUD_CONV(2500000);
        BAUD_CONV(3000000);
        BAUD_CONV(3500000);
        BAUD_CONV(4000000);
    }
    return -1;
}

}}} //end rp::arch::net

//begin rp::hal
namespace rp{ namespace hal{

serial_rxtx * serial_rxtx::CreateRxTx()
{
    return new rp::arch::net::raw_serial();
}

void serial_rxtx::ReleaseRxTx(serial_rxtx *rxtx)
{
    delete rxtx;
}

}} //end rp::hal
