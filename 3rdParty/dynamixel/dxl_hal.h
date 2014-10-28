#ifndef _DYNAMIXEL_HAL_HEADER
#define _DYNAMIXEL_HAL_HEADER


//#ifdef __cplusplus
//extern "C" {
//#endif

class CDXL_hal {
    public :
        CDXL_hal(void);
        ~CDXL_hal(void);
        int dxl_hal_open(int deviceIndex, float baudrate);
        void dxl_hal_close();
        int dxl_hal_set_baud( float baudrate );
        void dxl_hal_clear();
        int dxl_hal_tx( unsigned char *pPacket, int numPacket );
        int dxl_hal_rx( unsigned char *pPacket, int numPacket );
        void dxl_hal_set_timeout( int NumRcvByte );
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
