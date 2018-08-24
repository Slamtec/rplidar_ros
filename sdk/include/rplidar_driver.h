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


#ifndef __cplusplus
#error "The RPlidar SDK requires a C++ compiler to be built"
#endif

#ifndef DEPRECATED
    #ifdef __GNUC__
        #define DEPRECATED(func) func __attribute__ ((deprecated))
    #elif defined(_MSC_VER)
        #define DEPRECATED(func) __declspec(deprecated) func
    #else
        #pragma message("WARNING: You need to implement DEPRECATED for this compiler")
        #define DEPRECATED(func) func
    #endif
#endif

namespace rp { namespace standalone{ namespace rplidar {

struct RplidarScanMode {
    _u16    id;
    float   us_per_sample;   // microseconds per sample
    float   max_distance;    // max distance
    _u8     ans_type;         // the answer type of the scam mode, its value should be RPLIDAR_ANS_TYPE_MEASUREMENT*
    char    scan_mode[64];    // name of scan mode, max 63 characters
};

enum {
    DRIVER_TYPE_SERIALPORT = 0x0,
    DRIVER_TYPE_TCP = 0x1,
};

class ChannelDevice
{
public:
    virtual bool bind(const char*, uint32_t ) = 0;
    virtual bool open() {return true;}
    virtual void close() = 0;
    virtual void flush() {return;}
    virtual bool waitfordata(size_t data_count,_u32 timeout = -1, size_t * returned_size = NULL) = 0;
    virtual int senddata(const _u8 * data, size_t size) = 0;
    virtual int recvdata(unsigned char * data, size_t size) = 0;
    virtual void setDTR() {return;}
    virtual void clearDTR() {return;}
    virtual void ReleaseRxTx() {return;}
};

class RPlidarDriver {
public:
    enum {
        DEFAULT_TIMEOUT = 2000, //2000 ms
    };

    enum {
        MAX_SCAN_NODES = 8192,
    };

    enum {
        LEGACY_SAMPLE_DURATION = 476,
    };

public:
    /// Create an RPLIDAR Driver Instance
    /// This interface should be invoked first before any other operations
    ///
    /// \param drivertype the connection type used by the driver. 
    static RPlidarDriver * CreateDriver(_u32 drivertype = DRIVER_TYPE_SERIALPORT);

    /// Dispose the RPLIDAR Driver Instance specified by the drv parameter
    /// Applications should invoke this interface when the driver instance is no longer used in order to free memory
    static void DisposeDriver(RPlidarDriver * drv);


    /// Open the specified serial port and connect to a target RPLIDAR device
    ///
    /// \param port_path     the device path of the serial port 
    ///        e.g. on Windows, it may be com3 or \\.\com10 
    ///             on Unix-Like OS, it may be /dev/ttyS1, /dev/ttyUSB2, etc
    ///
    /// \param baudrate      the baudrate used
    ///        For most RPLIDAR models, the baudrate should be set to 115200
    ///
    /// \param flag          other flags
    ///        Reserved for future use, always set to Zero
    virtual u_result connect(const char *, _u32, _u32 flag = 0) = 0;


    /// Disconnect with the RPLIDAR and close the serial port
    virtual void disconnect() = 0;

    /// Returns TRUE when the connection has been established
    virtual bool isConnected() = 0;

    /// Ask the RPLIDAR core system to reset it self
    /// The host system can use the Reset operation to help RPLIDAR escape the self-protection mode.
    ///
    ///  \param timeout       The operation timeout value (in millisecond) for the serial port communication                     
    virtual u_result reset(_u32 timeout = DEFAULT_TIMEOUT) = 0;

    // FW1.24
    /// Get all scan modes that supported by lidar
    virtual u_result getAllSupportedScanModes(std::vector<RplidarScanMode>& outModes, _u32 timeoutInMs = DEFAULT_TIMEOUT) = 0;

    /// Get typical scan mode of lidar
    virtual u_result getTypicalScanMode(_u16& outMode, _u32 timeoutInMs = DEFAULT_TIMEOUT) = 0;

    /// Start scan
    ///
    /// \param force            Force the core system to output scan data regardless whether the scanning motor is rotating or not.
    /// \param useTypicalScan   Use lidar's typical scan mode or use the compatibility mode (2k sps)
    /// \param options          Scan options (please use 0)
    /// \param outUsedScanMode  The scan mode selected by lidar
    virtual u_result startScan(bool force, bool useTypicalScan, _u32 options = 0, RplidarScanMode* outUsedScanMode = NULL) = 0;

    /// Start scan in specific mode
    ///
    /// \param force            Force the core system to output scan data regardless whether the scanning motor is rotating or not.
    /// \param scanMode         The scan mode id (use getAllSupportedScanModes to get supported modes)
    /// \param options          Scan options (please use 0)
    /// \param outUsedScanMode  The scan mode selected by lidar
    virtual u_result startScanExpress(bool force, _u16 scanMode, _u32 options = 0, RplidarScanMode* outUsedScanMode = NULL, _u32 timeout = DEFAULT_TIMEOUT) = 0;

    /// Retrieve the health status of the RPLIDAR
    /// The host system can use this operation to check whether RPLIDAR is in the self-protection mode.
    ///
    /// \param health        The health status info returned from the RPLIDAR
    ///
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication     
    virtual u_result getHealth(rplidar_response_device_health_t & health, _u32 timeout = DEFAULT_TIMEOUT) = 0;

    /// Get the device information of the RPLIDAR include the serial number, firmware version, device model etc.
    /// 
    /// \param info          The device information returned from the RPLIDAR
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication  
    virtual u_result getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout = DEFAULT_TIMEOUT) = 0;

    /// Get the sample duration information of the RPLIDAR.
    /// DEPRECATED, please use RplidarScanMode::us_per_sample
    ///
    /// \param rateInfo      The sample duration information returned from the RPLIDAR
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication
    DEPRECATED(virtual u_result getSampleDuration_uS(rplidar_response_sample_rate_t & rateInfo, _u32 timeout = DEFAULT_TIMEOUT)) = 0;
    
    /// Set the RPLIDAR's motor pwm when using accessory board, currently valid for A2 only.
    /// 
    /// \param pwm           The motor pwm value would like to set 
    virtual u_result setMotorPWM(_u16 pwm) = 0;

    /// Start RPLIDAR's motor when using accessory board
    virtual u_result startMotor() = 0;

    /// Stop RPLIDAR's motor when using accessory board
    virtual u_result stopMotor() = 0;

    /// Check whether the device support motor control.
    /// Note: this API will disable grab.
    /// 
    /// \param support       Return the result.
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication. 
    virtual u_result checkMotorCtrlSupport(bool & support, _u32 timeout = DEFAULT_TIMEOUT) = 0;

    /// Calculate RPLIDAR's current scanning frequency from the given scan data
    /// DEPRECATED, please use getFrequency(RplidarScanMode, size_t)
    ///
    /// Please refer to the application note doc for details
    /// Remark: the calcuation will be incorrect if the specified scan data doesn't contains enough data
    ///
    /// \param inExpressMode Indicate whether the RPLIDAR is in express mode
    /// \param count         The number of sample nodes inside the given buffer
    /// \param frequency     The scanning frequency (in HZ) calcuated by the interface.
    /// \param is4kmode      Return whether the RPLIDAR is working on 4k sample rate mode.
    DEPRECATED(virtual u_result getFrequency(bool inExpressMode, size_t count, float & frequency, bool & is4kmode)) = 0;

    /// Calculate RPLIDAR's current scanning frequency from the given scan data
    /// Please refer to the application note doc for details
    /// Remark: the calcuation will be incorrect if the specified scan data doesn't contains enough data
    ///
    /// \param scanMode      Lidar's current scan mode
    /// \param count         The number of sample nodes inside the given buffer
    virtual u_result getFrequency(const RplidarScanMode& scanMode, size_t count, float & frequency) = 0;

    /// Ask the RPLIDAR core system to enter the scan mode(Normal/Express, Express mode is 4k mode)
    /// A background thread will be created by the RPLIDAR driver to fetch the scan data continuously.
    /// User Application can use the grabScanData() interface to retrieved the scan data cached previous by this background thread.
    ///
    /// \param force         Force the core system to output scan data regardless whether the scanning motor is rotating or not.
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication.
    virtual u_result startScanNormal(bool force, _u32 timeout = DEFAULT_TIMEOUT) = 0;
    
    /// Check whether the device support express mode.
    /// DEPRECATED, please use getAllSupportedScanModes
    ///
    /// \param support       Return the result.
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication.
    DEPRECATED(virtual u_result checkExpressScanSupported(bool & support, _u32 timeout = DEFAULT_TIMEOUT)) = 0;

    /// Ask the RPLIDAR core system to stop the current scan operation and enter idle state. The background thread will be terminated
    ///
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication 
    virtual u_result stop(_u32 timeout = DEFAULT_TIMEOUT) = 0;


    /// Wait and grab a complete 0-360 degree scan data previously received. 
    /// NOTE: This method only support distance less than 16.38 meters, for longer distance, please use grabScanDataHq
    /// The grabbed scan data returned by this interface always has the following charactistics:
    ///
    /// 1) The first node of the grabbed data array (nodebuffer[0]) must be the first sample of a scan, i.e. the start_bit == 1
    /// 2) All data nodes are belong to exactly ONE complete 360-degrees's scan
    /// 3) Note, the angle data in one scan may not be ascending. You can use API ascendScanData to reorder the nodebuffer.
    ///
    /// \param nodebuffer     Buffer provided by the caller application to store the scan data
    ///
    /// \param count          The caller must initialize this parameter to set the max data count of the provided buffer (in unit of rplidar_response_measurement_node_t).
    ///                       Once the interface returns, this parameter will store the actual received data count.
    ///
    /// \param timeout        Max duration allowed to wait for a complete scan data, nothing will be stored to the nodebuffer if a complete 360-degrees' scan data cannot to be ready timely.
    ///
    /// The interface will return RESULT_OPERATION_TIMEOUT to indicate that no complete 360-degrees' scan can be retrieved withing the given timeout duration. 
    ///
    /// \The caller application can set the timeout value to Zero(0) to make this interface always returns immediately to achieve non-block operation.
    DEPRECATED(virtual u_result grabScanData(rplidar_response_measurement_node_t * nodebuffer, size_t & count, _u32 timeout = DEFAULT_TIMEOUT)) = 0;

    /// Wait and grab a complete 0-360 degree scan data previously received. 
    /// The grabbed scan data returned by this interface always has the following charactistics:
    ///
    /// 1) The first node of the grabbed data array (nodebuffer[0]) must be the first sample of a scan, i.e. the start_bit == 1
    /// 2) All data nodes are belong to exactly ONE complete 360-degrees's scan
    /// 3) Note, the angle data in one scan may not be ascending. You can use API ascendScanData to reorder the nodebuffer.
    ///
    /// \param nodebuffer     Buffer provided by the caller application to store the scan data
    ///
    /// \param count          The caller must initialize this parameter to set the max data count of the provided buffer (in unit of rplidar_response_measurement_node_t).
    ///                       Once the interface returns, this parameter will store the actual received data count.
    ///
    /// \param timeout        Max duration allowed to wait for a complete scan data, nothing will be stored to the nodebuffer if a complete 360-degrees' scan data cannot to be ready timely.
    ///
    /// The interface will return RESULT_OPERATION_TIMEOUT to indicate that no complete 360-degrees' scan can be retrieved withing the given timeout duration. 
    ///
    /// \The caller application can set the timeout value to Zero(0) to make this interface always returns immediately to achieve non-block operation.
    virtual u_result grabScanDataHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count, _u32 timeout = DEFAULT_TIMEOUT) = 0;

    /// Ascending the scan data according to the angle value in the scan.
    ///
    /// \param nodebuffer     Buffer provided by the caller application to do the reorder. Should be retrived from the grabScanData
    ///
    /// \param count          The caller must initialize this parameter to set the max data count of the provided buffer (in unit of rplidar_response_measurement_node_t).
    ///                       Once the interface returns, this parameter will store the actual received data count.
    /// The interface will return RESULT_OPERATION_FAIL when all the scan data is invalid. 
    DEPRECATED(virtual u_result ascendScanData(rplidar_response_measurement_node_t * nodebuffer, size_t count)) = 0;

    /// Ascending the scan data according to the angle value in the scan.
    ///
    /// \param nodebuffer     Buffer provided by the caller application to do the reorder. Should be retrived from the grabScanData
    ///
    /// \param count          The caller must initialize this parameter to set the max data count of the provided buffer (in unit of rplidar_response_measurement_node_t).
    ///                       Once the interface returns, this parameter will store the actual received data count.
    /// The interface will return RESULT_OPERATION_FAIL when all the scan data is invalid. 
    virtual u_result ascendScanData(rplidar_response_measurement_node_hq_t * nodebuffer, size_t count) = 0;

    /// Return received scan points even if it's not complete scan
    ///
    /// \param nodebuffer     Buffer provided by the caller application to store the scan data
    ///
    /// \param count          Once the interface returns, this parameter will store the actual received data count.
    ///
    /// The interface will return RESULT_OPERATION_TIMEOUT to indicate that not even a single node can be retrieved since last call. 
    DEPRECATED(virtual u_result getScanDataWithInterval(rplidar_response_measurement_node_t * nodebuffer, size_t & count)) = 0;

    /// Return received scan points even if it's not complete scan
    ///
    /// \param nodebuffer     Buffer provided by the caller application to store the scan data
    ///
    /// \param count          Once the interface returns, this parameter will store the actual received data count.
    ///
    /// The interface will return RESULT_OPERATION_TIMEOUT to indicate that not even a single node can be retrieved since last call. 
    virtual u_result getScanDataWithIntervalHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count) = 0;

    virtual ~RPlidarDriver() {}
protected:
    RPlidarDriver(){}

public:
    ChannelDevice* _chanDev;
};




}}}
