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

#include "sl_lidar_driver.h"
#include "hal/abs_rxtx.h"
#include "hal/socket.h"


namespace sl {
    
    class SerialPortChannel : public ISerialPortChannel
    {
    public:
        SerialPortChannel(const std::string& device, int baudrate) :_rxtxSerial(rp::hal::serial_rxtx::CreateRxTx())
        {
            _device = device;
            _baudrate = baudrate;
        }

        ~SerialPortChannel()
        {
            if (_rxtxSerial)
                delete _rxtxSerial;
        }

        bool bind(const std::string& device, sl_s32 baudrate)
        {
            _closePending = false;
            return _rxtxSerial->bind(device.c_str(), baudrate);
        }

        bool open()
        {
            if(!bind(_device, _baudrate))
                return false;
            return _rxtxSerial->open();
        }

        void close()
        {
            _closePending = true;
            _rxtxSerial->cancelOperation();
            _rxtxSerial->close();
        }
        void flush()
        {
            _rxtxSerial->flush(0);
        }

        sl_result waitForDataExt(size_t& size_hint, sl_u32 timeoutInMs)
        {
            _word_size_t result;
            size_t size_holder;
            size_hint = 0;

            if (_closePending) return  RESULT_OPERATION_TIMEOUT;

            if (!_rxtxSerial->isOpened()) {
                return RESULT_OPERATION_FAIL;
            }

            result = _rxtxSerial->waitfordata(1, timeoutInMs, &size_holder);
            size_hint = size_holder;
            if (result == (_word_size_t)rp::hal::serial_rxtx::ANS_DEV_ERR)
                return RESULT_OPERATION_FAIL;
            if (result == (_word_size_t)rp::hal::serial_rxtx::ANS_TIMEOUT)
                return RESULT_OPERATION_TIMEOUT;

            return RESULT_OK;
        }

        bool waitForData(size_t size, sl_u32 timeoutInMs, size_t* actualReady)
        {
            if (_closePending) return false;
            return (_rxtxSerial->waitfordata(size, timeoutInMs, actualReady) == rp::hal::serial_rxtx::ANS_OK);
        }

        int write(const void* data, size_t size)
        {
           return _rxtxSerial->senddata((const sl_u8 * )data, size);
        }

        int read(void* buffer, size_t size)
        {
            size_t lenRec = 0;
            lenRec = _rxtxSerial->recvdata((sl_u8 *)buffer, size);
            return (int)lenRec;
        }

        void clearReadCache()
        {
           
        }

        void setDTR(bool dtr)
        {
            dtr ? _rxtxSerial->setDTR() : _rxtxSerial->clearDTR();
        }

        int getChannelType() {
            return CHANNEL_TYPE_SERIALPORT;
        }

    private:
        rp::hal::serial_rxtx  * _rxtxSerial;
        bool _closePending;
        std::string _device;
        int _baudrate;

    };

    Result<IChannel*> createSerialPortChannel(const std::string& device, int baudrate)
    {
        return new  SerialPortChannel(device, baudrate);
    }

}