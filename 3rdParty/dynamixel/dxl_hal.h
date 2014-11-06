#ifndef _DYNAMIXEL_HAL_HEADER
#define _DYNAMIXEL_HAL_HEADER


//#ifdef __cplusplus
//extern "C" {
//#endif

/**
 * \brief The CDXL_hal class - hardware abstraciton layer for DynamixelSerwo.
 */

class CDXL_hal {
    public :
        /// Constructor.
        CDXL_hal(void);
        /// Destructor.
        ~CDXL_hal(void);
        /// Open connection port, return 1 - succesfull, 0 - error.
        int dxl_hal_open(int deviceIndex, float baudrate);
        /// Close connection port.
        void dxl_hal_close();
        /// Set the baudrate of communication, return 1 - ok, 0 - error.
        int dxl_hal_set_baud( float baudrate );
        /// Clear all received buffers of the communication devices.
        void dxl_hal_clear();
        /// Actualization routine that tranismits packet through the communication devices, return number of data actually transmited.
        int dxl_hal_tx( unsigned char *pPacket, int numPacket );
        /// Actualization routine that received packet from the communication devices, return number of data actually taken out.
        int dxl_hal_rx( unsigned char *pPacket, int numPacket );
        ///  Count response time - stop watch.
        void dxl_hal_set_timeout( int NumRcvByte );
        /// Check whether waiting time is passed or not, return 1 - timeout, 0 - ok.
        int dxl_hal_timeout();

    private:
        int	gSocket_fd;
        long	glStartTime;
        float	gfRcvWaitTime;
        float	gfByteTransTime;

        char	gDeviceName[20];
};


//#ifdef __cplusplus
//}
//#endif

#endif
