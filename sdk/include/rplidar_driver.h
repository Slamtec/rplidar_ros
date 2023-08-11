/*
 *  RPLIDAR SDK
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
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

#ifndef __cplusplus
#error "The RPlidar SDK requires a C++ compiler to be built"
#endif


namespace rp { namespace standalone{ namespace rplidar {
    using namespace sl;
    typedef LidarScanMode RplidarScanMode;

enum {
   DRIVER_TYPE_SERIALPORT = 0x0,
   DRIVER_TYPE_TCP = 0x1,
   DRIVER_TYPE_UDP = 0x2,
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
    static RPlidarDriver * CreateDriver(_u32 drivertype = CHANNEL_TYPE_SERIALPORT);
    

    RPlidarDriver(sl_u32 channelType);

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
    u_result connect(const char *path, _u32 portOrBaud, _u32 flag = 0);
    
    /// Disconnect with the RPLIDAR and close the serial port
    void disconnect();

    /// Returns TRUE when the connection has been established
    bool isConnected(); 

    /// Ask the RPLIDAR core system to reset it self
    /// The host system can use the Reset operation to help RPLIDAR escape the self-protection mode.
    ///
    ///  \param timeout       The operation timeout value (in millisecond) for the serial port communication                     
    u_result reset(_u32 timeout = DEFAULT_TIMEOUT);

    u_result clearNetSerialRxCache() {
        return RESULT_OK;
    }
    // FW1.24
    /// Get all scan modes that supported by lidar
    u_result getAllSupportedScanModes(std::vector<RplidarScanMode>& outModes, _u32 timeoutInMs = DEFAULT_TIMEOUT);
   
    /// Get typical scan mode of lidar
    u_result getTypicalScanMode(_u16& outMode, _u32 timeoutInMs = DEFAULT_TIMEOUT);

    /// Start scan
    ///
    /// \param force            Force the core system to output scan data regardless whether the scanning motor is rotating or not.
    /// \param useTypicalScan   Use lidar's typical scan mode or use the compatibility mode (2k sps)
    /// \param options          Scan options (please use 0)
    /// \param outUsedScanMode  The scan mode selected by lidar
    u_result startScan(bool force, bool useTypicalScan, _u32 options = 0, RplidarScanMode* outUsedScanMode = NULL);

    /// Start scan in specific mode
    ///
    /// \param force            Force the core system to output scan data regardless whether the scanning motor is rotating or not.
    /// \param scanMode         The scan mode id (use getAllSupportedScanModes to get supported modes)
    /// \param options          Scan options (please use 0)
    /// \param outUsedScanMode  The scan mode selected by lidar
    u_result startScanExpress(bool force, _u16 scanMode, _u32 options = 0, RplidarScanMode* outUsedScanMode = NULL, _u32 timeout = DEFAULT_TIMEOUT);

    /// Retrieve the health status of the RPLIDAR
    /// The host system can use this operation to check whether RPLIDAR is in the self-protection mode.
    ///
    /// \param health        The health status info returned from the RPLIDAR
    ///
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication     
    u_result getHealth(rplidar_response_device_health_t & health, _u32 timeout = DEFAULT_TIMEOUT);

    /// Get the device information of the RPLIDAR include the serial number, firmware version, device model etc.
    /// 
    /// \param info          The device information returned from the RPLIDAR
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication  
    u_result getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout = DEFAULT_TIMEOUT);

    /// Set the RPLIDAR's motor pwm when using accessory board, currently valid for A2 only.
    /// 
    /// \param pwm           The motor pwm value would like to set 
    u_result setMotorPWM(_u16 pwm);

    /// Start RPLIDAR's motor when using accessory board
    u_result startMotor();

    /// Stop RPLIDAR's motor when using accessory board
    u_result stopMotor();

    /// Check whether the device support motor control.
    /// Note: this API will disable grab.
    /// 
    /// \param support       Return the result.
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication. 
    u_result checkMotorCtrlSupport(bool & support, _u32 timeout = DEFAULT_TIMEOUT);

	///Set LPX and S2E series lidar's static IP address
	///
	/// \param conf             Network parameter that LPX series lidar owned
	/// \param timeout          The operation timeout value (in millisecond) for the ethernet udp communication
	u_result  setLidarIpConf(const rplidar_ip_conf_t& conf, _u32 timeout = DEFAULT_TIMEOUT);

    ///Get LPX and S2E series lidar's static IP address
    ///
    /// \param conf             Network parameter that LPX series lidar owned
    /// \param timeout          The operation timeout value (in millisecond) for the ethernet udp communication
    u_result  getLidarIpConf(rplidar_ip_conf_t& conf, _u32 timeout = DEFAULT_TIMEOUT);

	///Get LPX and S2E series lidar's MAC address
	///
	/// \param macAddrArray         The device MAC information returned from the LPX series lidar
	u_result getDeviceMacAddr(_u8* macAddrArray, _u32 timeoutInMs = DEFAULT_TIMEOUT);

    /// Ask the RPLIDAR core system to stop the current scan operation and enter idle state. The background thread will be terminated
    ///
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication 
    u_result stop(_u32 timeout = DEFAULT_TIMEOUT);

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
    u_result grabScanDataHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count, _u32 timeout = DEFAULT_TIMEOUT);

    /// Ascending the scan data according to the angle value in the scan.
    ///
    /// \param nodebuffer     Buffer provided by the caller application to do the reorder. Should be retrived from the grabScanData
    ///
    /// \param count          The caller must initialize this parameter to set the max data count of the provided buffer (in unit of rplidar_response_measurement_node_t).
    ///                       Once the interface returns, this parameter will store the actual received data count.
    /// The interface will return RESULT_OPERATION_FAIL when all the scan data is invalid. 
    u_result ascendScanData(rplidar_response_measurement_node_hq_t * nodebuffer, size_t count);

    /// Return received scan points even if it's not complete scan
    ///
    /// \param nodebuffer     Buffer provided by the caller application to store the scan data
    ///
    /// \param count          Once the interface returns, this parameter will store the actual received data count.
    ///
    /// The interface will return RESULT_OPERATION_TIMEOUT to indicate that not even a single node can be retrieved since last call. 
    u_result getScanDataWithInterval(rplidar_response_measurement_node_t * nodebuffer, size_t & count);

    /// Return received scan points even if it's not complete scan
    ///
    /// \param nodebuffer     Buffer provided by the caller application to store the scan data
    ///
    /// \param count          Once the interface returns, this parameter will store the actual received data count.
    ///
    /// The interface will return RESULT_OPERATION_TIMEOUT to indicate that not even a single node can be retrieved since last call. 
    u_result getScanDataWithIntervalHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count);


    virtual ~RPlidarDriver();
protected:
    RPlidarDriver();

private:
    sl_u32 _channelType;
    IChannel* _channel;
    ILidarDriver* _lidarDrv;
    
};




}}}
