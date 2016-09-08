
#include "localization/localizer.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <cstddef>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <math.h>

#include "../../3rdParty/mti/cmtdef.h"
#include "../../3rdParty/mti/xsens_time.h"
#include "../../3rdParty/mti/xsens_list.h"
#include "../../3rdParty/mti/cmtscan.h"
#include "../../3rdParty/mti/cmt3.h"
#include <array>

using namespace xsens;
using namespace std;

// this macro tests for an error and exits the program with a message if there was one
#define EXIT_ON_ERROR(res,comment) if (res != XRV_OK) { printf("Error %d occurred in " comment ": %s\n",res,xsensResultText(res)); exit(1); }

namespace controller {
    /// create a single Localizer (AHRS)
    Localizer* createLocalizerAHRS(void);
}

using namespace controller;

class AHRS : public Localizer {
    struct SAhrs{
      double rot[3];
      double rot_raw[3];
      double drot[3];
      double dpos[3];
      double ddpos[3];
      double ddpos_raw[3];
      double ang_matrix[9];
      double temp;
      double temp_raw;
      double raw_Mag[3];
      double CMag[3];
      double RawPress;
      double RawPressAge;
      double PressPvt;
      double PressAgePvt;
      double GPSPvt;
      double AnIn1;
      double AnIn2;
      double LLA[3];
      double nano;
      double year;
      double month;
      double rtc;
      double sample_count;
      double status;
      double FPValue;
      std::array<double,9> Ori;
      std::array<double,4> quat;
      std::array<double,3>accG;
    };

	private:
		void doMtSettings(xsens::Cmt3 &, CmtOutputMode &, CmtOutputSettings &, CmtDeviceId []);
		///mti
		void setupMti(CmtOutputMode &mode, int mode_no, CmtOutputSettings &settings, int sett_no);
		/// mti
		void getUserInputs(CmtOutputMode &, CmtOutputSettings &);
		/// mti
		void writeHeaders(unsigned long, CmtOutputMode &, CmtOutputSettings &, int&, int&);

        /// Data structure
        SAhrs ahrsData;

	public:
        /// Pointer
        typedef std::unique_ptr<AHRS> Ptr;
        /// construction/ destruction
        /// overloaded constructor
        AHRS(void);
        ~AHRS(void);

        /// get body orientation
        void getBodyOrientation(double& roll, double& pitch, double& yaw);

		///  odczytuje pozycje i orientacje
        signed char readPositionGiro(double & RotX, double & RotY,double & RotZ);
         char readPosition(std::array<double,3>& tab);
		/// odczytuje orientacje -- macierz
         char readMatrixGiro(std::array<double,9> res);
        char readPositionGiroRaw(double & RotX, double & RotY, double & RotZ);
		///  odczytuje predkosc katowa
        signed char readAccelerationLinear(double & L_AcceloX, double & L_AcceloY, double & L_AcceloZ);
         char readAccelerationLinear(std::array<double,3> tab);

        signed char readAccelerationLinearRaw(double & L_AcceloX, double & L_AcceloY, double & L_AcceloZ);
        char readAccelerationLinearRaw(std::array<double,3> tab);
		///  odczytuje predkosc katowa
        signed char readVelocity(double  & VeloX, double & VeloY, double & VeloZ);
         char readVelocity(std::array<double,3> tab);
		///Temperatura
        signed char readTemperature(double & T_Temp);
        signed char readTemperatureRaw(double & T_Temp);
		///MAgnetometr
        signed char readRawMag(double & MagX, double & MagY, double & MagZ);
        char readRawMag(std::array<double,3> tab);

        signed	char readMag(double & MagX, double & MagY, double & MagZ);
        char readMag(std::array<double,3> tab);

		///ciśnienie
        signed char readRawPress(double & press, double & pressAge);
        char readRawPress(std::array<double,3> tab);
		///GPS
        signed char readGpsPvtData(double & press, double & PressAge, double & GPSAge);
        char readGpsPvtData(std::array<double,3> tab);
		///Analo In
        signed char readAnalogIn1(double & In1);
        signed char readAnalogIn2(double & In2);

		///PosLLa
        signed char readLLaPosition(double & PosLLA_X, double & PosLLA_Y, double & PosLLA_Z);
        char readLLaPosition(std::array<double,3> tab);
		///UTC Time
        signed char readUTCTime(double & nano, double & year, double & month);
        char readUTCTime(std::array<double,3> tab);
		///rtc 
        signed char readRtc(double & rtc);

		///Counter
        signed char readSampleCounter(double &Counter);
		/// Status
        signed char readStatus(double & stat);
		///
        signed char readFPValueSize(double & FP);
        signed char readOriMatrix(std::array<double,9>  res);
        signed char readOriQuat(double & x, double & y, double & z, double & w);
        char readOriQuat(std::array<double,4>  res);
        signed char readAccg(double & power, double & OptEstimation, double & Magsens);
         char readAccg(std::array<double,3>);
		///  reads all values
  /// //
		void procedure(void);


		/// mti
		int doHardwareScan(xsens::Cmt3 &, CmtDeviceId []);
		/// mti
		/// mode 1 - Calibrated data
		/// mode 2 - Orientation data and GPS Position (MTi-G only)
		/// mode 3 - Both Calibrated and Orientation data
		/// mode 4 - Temperature and Calibrated data
		/// mode 5 - Temperature and Orientation data
		/// mode 6 - Temperature, Calibrated and Orientation data
		/// settings 1 - Quaternions
		/// settings 2 - Euler angles
		/// settings 3 - Matrix
		
	int shm_id;
	key_t key;

    float         pos[6];
    float         dpos[6];
    float         ang_matrix[9];
    float         ddpos[6];
    float         ddpos_raw[3];
    xsens::Cmt3 cmt_object;
    /// cmt3
    xsens::Cmt3 * cmt3; // do shm
    /// device id
    CmtDeviceId * deviceIds;
    /// output mode
    CmtOutputMode mode;
    /// output settings
    CmtOutputSettings settings;
    ///
    XsensResultValue res;
    /// nti counter
    unsigned long mtCount;
    ///structs to hold data.
    CmtCalData caldata;
    CmtQuat qat_data;
    CmtEuler euler_data;
    CmtMatrix matrix_data;
    CmtVector acc_vct;
    CmtShortVector acc_vct_raw;
    CmtVector gyr_vct;
    double Temperature;
    double Temperature_raw;
    CmtShortVector MagRaw;
    CmtVector Mag;
    CmtRawPressureData PressureRaw;
    CmtGpsPvtData GPS;
    CmtAnalogInData AnalogInput1;
    CmtAnalogInData AnalogInput2;
    CmtVector LLAPos;
    CmtUtcTime UTCtime;
    TimeStamp RTC;
    uint16_t SampleCounter;
    uint16_t Status;
    uint16_t FPValueSize;
    CmtMatrix OriMatrix;
    CmtQuat OriQuaternion;
    CmtVector AccG;
    Packet* packet; //do shm
};

