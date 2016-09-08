#include "localization/ahrs.h"
#include <iostream>

using namespace xsens;
using namespace std;

//Packet packet_object(0,0); //do shm

AHRS::Ptr localizerAHRS;

controller::Localizer* controller::createLocalizerAHRS(void) {
    localizerAHRS.reset(new AHRS());
    return localizerAHRS.get();
}

AHRS::AHRS(void) : Localizer("AHRS MTi", TYPE_AHRS_XSENSE){
	for(int i=0;i<6;i++){
        ahrsData.rot[i]=0;
        ahrsData.drot[i]=0;
        ahrsData.dpos[i]=0;
        ahrsData.ddpos[i]=0;
        ang_matrix[i]=0;
        ang_matrix[i+3]=0;
        ang_matrix[i+6]=0;
	}
	
	res = XRV_OK;
	mtCount = 0;
    //int temperatureOffset = 0;
    //int screenSensorOffset = 0;
	
	xsens::Cmt3 cmt_object;
	*cmt3 = cmt_object;
	// Perform hardware scan
	mtCount = doHardwareScan(*cmt3, deviceIds);
	packet = new Packet((unsigned short) mtCount, cmt3->isXm());

	setupMti(mode, 3, settings,2);
	// Set device to user input settings
	doMtSettings(*cmt3, mode, settings, deviceIds);
}

AHRS::~AHRS(void){
}

/// get body orientation
void AHRS::getBodyOrientation(double& roll, double& pitch, double& yaw){
    procedure();
    std::array<double,3> angles;
    this->readPosition(angles);
    roll = angles[0]; pitch = angles[1]; yaw = angles[2];
}

///  odczytuje pozycje i orientacje
signed char AHRS::readPositionGiro(double & RotX, double & RotY, double & RotZ){
    RotX=ahrsData.rot[0];
    RotY=ahrsData.rot[1];
    RotZ=ahrsData.rot[2];
}

char AHRS::readPositionGiroRaw(double & RotX, double & RotY, double &RotZ){
    RotX=ahrsData.rot_raw[0];
    RotY=ahrsData.rot_raw[1];
    RotZ=ahrsData.rot_raw[2];

	return 0;
}

char AHRS::readPosition(std::array<double,3>& tab){
    tab[3]=ahrsData.rot[0];
    tab[4]=ahrsData.rot[1];
    tab[5]=ahrsData.rot[2];
}

///  odczytuje macierz rotacji
char AHRS::readMatrixGiro(std::array<double,9> matrix){
	for (int i=0;i<9;i++){
      matrix[i]=ahrsData.ang_matrix[i];
	}
}

///  odczytuje przyspieszenie liniowe
signed char AHRS::readAccelerationLinear(double & L_AcceloX, double & L_AcceloY, double & L_AcceloZ){
    L_AcceloX=ahrsData.ddpos[0];
    L_AcceloY=ahrsData.ddpos[1];
    L_AcceloZ=ahrsData.ddpos[2];
    return 0;
}

char AHRS::readAccelerationLinear(std::array<double,3> tab){
      tab[3]=ahrsData.ddpos[0];
      tab[4]=ahrsData.ddpos[1];
      tab[5]=ahrsData.ddpos[2];
}

signed char AHRS::readAccelerationLinearRaw(double & L_AcceloX, double & L_AcceloY, double &L_AcceloZ){
    L_AcceloX=ahrsData.ddpos_raw[0];
    L_AcceloY=ahrsData.ddpos_raw[1];
    L_AcceloZ=ahrsData.ddpos_raw[2];
}

char AHRS::readAccelerationLinearRaw(std::array<double,3> tab){
    tab[3]=ahrsData.ddpos_raw[0];
    tab[4]=ahrsData.ddpos_raw[1];
    tab[5]=ahrsData.ddpos_raw[2];
}

///  odczytuje predkosc katowa
char AHRS::readVelocity(std::array<double,3> tab){
    tab[3]=ahrsData.dpos[0];
    tab[4]=ahrsData.dpos[1];
    tab[5]=ahrsData.dpos[2];
}

signed char AHRS::readVelocity(double & VeloX, double & VeloY, double & VeloZ){
    VeloX=ahrsData.dpos[0];
    VeloY=ahrsData.dpos[1];
    VeloZ=ahrsData.dpos[2];
}

signed char AHRS::readTemperature(double & T_Temp){
    T_Temp=ahrsData.temp;
}

signed char AHRS::readTemperatureRaw(double & T_Temp){
    T_Temp=ahrsData.temp_raw;
}

///Magnetomer
signed char AHRS:: readRawMag(double & MagX, double & MagY, double & MagZ){
    MagX=ahrsData.raw_Mag[0];
    MagY=ahrsData.raw_Mag[1];
    MagZ=ahrsData.raw_Mag[2];
}

char  AHRS::readRawMag(std::array<double,3> tab){
    tab[3]=ahrsData.raw_Mag[0];
    tab[4]=ahrsData.raw_Mag[1];
    tab[5]=ahrsData.raw_Mag[2];
}

signed char AHRS:: readMag(double & MagX, double & MagY, double & MagZ){
    MagX=ahrsData.CMag[0];
    MagY=ahrsData.CMag[1];
    MagZ=ahrsData.CMag[2];
}
	
char  AHRS::readMag(std::array<double,3> tab){
    tab[3]=ahrsData.CMag[0];
    tab[4]=ahrsData.CMag[1];
    tab[5]=ahrsData.CMag[2];
}

//cisnienie
signed char AHRS::readRawPress(double & press, double & pressAge){
    press=ahrsData.RawPress;
    pressAge=ahrsData.RawPressAge;
}

char  AHRS::readRawPress(std::array<double,3> tab){
    tab[3]=ahrsData.RawPress;
    tab[4]=ahrsData.RawPressAge;
}

///GPS
signed char AHRS :: readGpsPvtData(double & Press, double & PressAge, double & GPSAge){
    Press=ahrsData.PressPvt;
    PressAge=ahrsData.PressAgePvt;
    GPSAge=ahrsData.GPSPvt;
}

char AHRS::readGpsPvtData(std::array<double,3> tab){
    tab[3]=ahrsData.PressPvt;
    tab[4]=ahrsData.PressAgePvt;
    tab[5]=ahrsData.GPSPvt;
}

///Analog In
signed char  AHRS ::readAnalogIn1(double & In1){
    In1=ahrsData.AnIn1;
}

signed char  AHRS ::readAnalogIn2(double & In2){
    In2=ahrsData.AnIn2;
}

///Pos LLA
signed char AHRS::readLLaPosition(double & PosLLA_X, double & PosLLA_Y, double & PosLLA_Z){
    PosLLA_X=ahrsData.LLA[0];
    PosLLA_Y=ahrsData.LLA[1];
    PosLLA_Z=ahrsData.LLA[2];
}

char  AHRS::readLLaPosition(std::array<double,3> tab){
    tab[3]=ahrsData.LLA[0];
    tab[4]=ahrsData.LLA[1];
    tab[5]=ahrsData.LLA[2];
}

///UTC
signed char AHRS:: readUTCTime(double & nano, double & year,double & month){
    nano=ahrsData.nano;
    year=ahrsData.year;
    month=ahrsData.month;
}

char AHRS:: readUTCTime(std::array<double,3> tab){
    tab[3]=ahrsData.nano;
    tab[4]=ahrsData.year;
    tab[5]=ahrsData.month;
}
	
///RTC
signed char AHRS:: readRtc(double & rtc){
    rtc=ahrsData.rtc;
}

///Counter
signed char AHRS::readSampleCounter(double &  Counter){
    Counter=ahrsData.sample_count;
}

//Status
signed char AHRS ::readStatus(double & stat){
    stat=ahrsData.status;
}

///FPValueSize
signed char AHRS::readFPValueSize(double &FP){
    FP=ahrsData.FPValue;
}

signed char AHRS :: readOriMatrix(std::array<double,9> Ori){
    Ori[0]=ahrsData.Ori[0];
    Ori[1]=ahrsData.Ori[1];
    Ori[2]=ahrsData.Ori[2];
    Ori[3]=ahrsData.Ori[3];
    Ori[4]=ahrsData.Ori[4];
    Ori[5]=ahrsData.Ori[5];
    Ori[6]=ahrsData.Ori[6];
    Ori[7]=ahrsData.Ori[7];
    Ori[8]=ahrsData.Ori[8];
}

signed char AHRS ::readOriQuat(double & x, double & y, double & z, double & w){
    x=ahrsData.quat[0];
    y=ahrsData.quat[1];
    z=ahrsData.quat[2];
    w=ahrsData.quat[3];
}

char AHRS ::readOriQuat(std::array<double,4> tab){
    tab[2]=ahrsData.quat[0];
    tab[3]=ahrsData.quat[1];
    tab[4]=ahrsData.quat[2];
    tab[5]=ahrsData.quat[3];
}
	
signed char AHRS :: readAccg(double & power, double & OptEstimation, double & Magsens){
    power=ahrsData.accG[0];
    OptEstimation=ahrsData.accG[1];
    Magsens=ahrsData.accG[2];
}

char AHRS ::readAccg(std::array<double,3>tab){
    tab[3]=ahrsData.accG[0];
    tab[4]=ahrsData.accG[1];
    tab[5]=ahrsData.accG[2];
}

///  reads all values
void AHRS::procedure(void){
    //readVelocity();
    cmt3->waitForDataMessage(packet);
	gyr_vct = packet->getCalGyr(0) ;
    ahrsData.dpos[0] = gyr_vct.m_data[0];
    ahrsData.dpos[1] = gyr_vct.m_data[1];
    ahrsData.dpos[2] = gyr_vct.m_data[2];
      //readAcceleration();
	cmt3->waitForDataMessage(packet);
	acc_vct = packet->getCalAcc(0);
	
    ahrsData.ddpos[0] = acc_vct.m_data[0];
    ahrsData.ddpos[1] = acc_vct.m_data[1];
    ahrsData.ddpos[2] = acc_vct.m_data[2];

	cmt3->waitForDataMessage(packet);
	acc_vct_raw = packet->getRawAcc(0);
    ahrsData.ddpos_raw[0] = acc_vct_raw.m_data[0];
    ahrsData.ddpos_raw[1] = acc_vct_raw.m_data[1];
    ahrsData.ddpos_raw[2] = acc_vct_raw.m_data[2];
	
	//Temperatura
    cmt3->waitForDataMessage(packet);
	Temperature=packet->getTemp(0);
    ahrsData.temp=Temperature;

	cmt3->waitForDataMessage(packet);
	Temperature_raw=packet->getRawTemp(0);
    ahrsData.temp_raw=Temperature_raw;
	
	//Magnetometr
	cmt3->waitForDataMessage(packet);
	MagRaw=packet->getRawMag(0);
    ahrsData.raw_Mag[0]=MagRaw.m_data[0];
    ahrsData.raw_Mag[1]=MagRaw.m_data[1];
    ahrsData.raw_Mag[2]=MagRaw.m_data[2];

	cmt3->waitForDataMessage(packet);
	Mag=packet->getCalMag(0);
    ahrsData.CMag[0]=Mag.m_data[0];
    ahrsData.CMag[1]=Mag.m_data[1];
    ahrsData.CMag[2]=Mag.m_data[2];

	///ciśnienie
	cmt3->waitForDataMessage(packet);
	PressureRaw=packet->getRawPressureData(0);
    ahrsData.RawPress=PressureRaw.m_pressure;
    ahrsData.RawPressAge=PressureRaw.m_pressureAge;

	///GPS
	cmt3->waitForDataMessage(packet);
	GPS=packet->getGpsPvtData(0);
    ahrsData.PressPvt=GPS.m_pressure;
    ahrsData.PressAgePvt=GPS.m_pressureAge;
    ahrsData.GPSPvt=GPS.m_gpsAge;

	///Analog In
	cmt3->waitForDataMessage(packet);
	AnalogInput1=packet->getAnalogIn1(0);
    ahrsData.AnIn1=AnalogInput1.m_data;

 	cmt3->waitForDataMessage(packet);
	AnalogInput2=packet->getAnalogIn2(0);
    ahrsData.AnIn2=AnalogInput2.m_data;

	///Pos LLA
	cmt3->waitForDataMessage(packet);
	LLAPos=packet->getPositionLLA(0);
    ahrsData.LLA[0]=LLAPos.m_data[0];
    ahrsData.LLA[1]=LLAPos.m_data[1];
    ahrsData.LLA[2]=LLAPos.m_data[2];

	///Time
	cmt3->waitForDataMessage(packet);
	UTCtime=packet->getUtcTime(0);
    ahrsData.nano=UTCtime.m_nano;
    ahrsData.year=UTCtime.m_year;
    ahrsData.month=UTCtime.m_month;
	///rtc

	cmt3->waitForDataMessage(packet);
	RTC=packet->getRtc();
    ahrsData.rtc=RTC;

	///SampleCounter
	cmt3->waitForDataMessage(packet);
	SampleCounter=packet->getSampleCounter();
    ahrsData.sample_count=SampleCounter;

	///SampleCounter
	cmt3->waitForDataMessage(packet);
	SampleCounter=packet->getSampleCounter();
    ahrsData.sample_count=SampleCounter;
	
	///FPValueSize
	cmt3->waitForDataMessage(packet);
	FPValueSize=packet->getFPValueSize(0);
    ahrsData.FPValue=FPValueSize;

	cmt3->waitForDataMessage(packet);
	AccG=packet->getAccG(0);
    ahrsData.accG[0]=AccG.m_data[0];
    ahrsData.accG[1]=AccG.m_data[1];

	cmt3->waitForDataMessage(packet);
	OriMatrix=packet->getOriMatrix();

    ahrsData.Ori[0]=OriMatrix.m_data[0][0];
    ahrsData.Ori[1]=OriMatrix.m_data[0][1];
    ahrsData.Ori[2]=OriMatrix.m_data[0][2];
    ahrsData.Ori[3]=OriMatrix.m_data[1][0];
    ahrsData.Ori[4]=OriMatrix.m_data[1][1];
    ahrsData.Ori[5]=OriMatrix.m_data[1][2];
    ahrsData.Ori[6]=OriMatrix.m_data[2][0];
    ahrsData.Ori[7]=OriMatrix.m_data[2][1];
    ahrsData.Ori[8]=OriMatrix.m_data[2][2];
        
	///Quaternions
	cmt3->waitForDataMessage(packet);
	OriQuaternion=packet->getOriQuat(0);
    ahrsData.quat[0]=OriQuaternion.m_data[0];
    ahrsData.quat[1]=OriQuaternion.m_data[1];
    ahrsData.quat[2]=OriQuaternion.m_data[2];
    ahrsData.quat[3]=OriQuaternion.m_data[3];
///accG

    //readMatrix();
	cmt3->waitForDataMessage(packet);
	euler_data = packet->getOriEuler(0);//mamy tylko jednego mti 0
	float ps = euler_data.m_yaw*3.14/180;
	float th = euler_data.m_pitch*3.14/180;
	float fi = euler_data.m_roll*3.14/180;	
	
    ahrsData.ang_matrix[0] = (cos(th)*cos(ps));
    ahrsData.ang_matrix[1] = (sin(fi)*sin(th)*cos(ps)-cos(fi)*sin(ps));
    ahrsData.ang_matrix[2] = (cos(fi)*sin(th)*cos(ps)+sin(fi)*sin(ps));
    ahrsData.ang_matrix[3] = (cos(th)*sin(ps));
    ahrsData.ang_matrix[4] = (sin(fi)*sin(th)*sin(ps)+cos(fi)*cos(ps));
    ahrsData.ang_matrix[5] = (cos(fi)*sin(th)*sin(ps)-sin(fi)*cos(ps));
    ahrsData.ang_matrix[6] = (-sin(th));
    ahrsData.ang_matrix[7] = (sin(fi)*cos(th));
    ahrsData.ang_matrix[8] = (cos(fi)*cos(th));

    //readPosition();
    cmt3->waitForDataMessage(packet);
	euler_data = packet->getOriEuler(0);//mamy tylko jednego mti 0
    ahrsData.rot[0] = euler_data.m_roll*3.14/180;
    ahrsData.rot[1] = euler_data.m_pitch*3.14/180;
    ahrsData.rot[2] = euler_data.m_yaw*3.14/180;
    }
		
    //////////////////////////////////////////////////////////////////////////
    // doHardwareScan
    //
    // Checks available COM ports and scans for MotionTrackers
    int AHRS::doHardwareScan(xsens::Cmt3 &cmt3, CmtDeviceId deviceIds[])
    {
	    XsensResultValue res;
	    List<CmtPortInfo> portInfo;
	    unsigned long portCount = 0;
	    int mtCount;
	    
	    xsens::cmtScanPorts(portInfo);
	    portCount = portInfo.length();
	    
	    if (portCount == 0) {
		    printf("No MotionTrackers found \n\n");
		    return 0; 
	    }

	    for(int p = 0; p < (int)portCount; p++){
		    res = cmt3.openPort(portInfo[p].m_portName, portInfo[p].m_baudrate);
		    EXIT_ON_ERROR(res,"cmtOpenPort");  
	    }

	    //get the Mt sensor count.
	    mtCount = cmt3.getMtCount();

	    // retrieve the device IDs 
	    for(int j = 0; j < mtCount; j++){
		    res = cmt3.getDeviceId((unsigned char)(j+1), deviceIds[j]);
		    EXIT_ON_ERROR(res,"getDeviceId");
	    }
	    
	    return mtCount;
    }

    //////////////////////////////////////////////////////////////////////////
    // getUserInputs
    //
    // Request user for output data
    void AHRS::setupMti(CmtOutputMode &mode, int mode_no, CmtOutputSettings &settings, int sett_no)
    {
	    mode = 0;
	    
	    switch(mode_no)
	    {
	    case 1:
		    mode = CMT_OUTPUTMODE_CALIB;
		    break;
	    case 2:
		    mode = CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_POSITION;
		    break;
	    case 3:
		    mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
		    break;
	    case 4:
		    mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB;
		    break;
	    case 5:
		    mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_ORIENT;
		    break;
	    case 6:
		    mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
		    break;
	    }

	    if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {
		    settings = 0;
		    // Update outputSettings to match data specs of SetOutputSettings
		    switch(sett_no) {
		    case 1:
			    settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
			    break;
		    case 2:
			    settings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER;
			    break;
		    case 3:
			    settings = CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX;
			    break;
		    }
	    } else {
		    settings = 0;
	    }	    	    
	    settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
    }

    //////////////////////////////////////////////////////////////////////////
    // doMTSettings
    //
    // Set user settings in MTi/MTx
    // Assumes initialized global MTComm class
    void AHRS::doMtSettings(xsens::Cmt3 &cmt3, CmtOutputMode &mode,
		    CmtOutputSettings &settings, CmtDeviceId deviceIds[]) 
    {
	    XsensResultValue res;
	    unsigned long mtCount = cmt3.getMtCount();

	    // set sensor to config sate
	    res = cmt3.gotoConfig();
	    EXIT_ON_ERROR(res,"gotoConfig");

	    unsigned short sampleFreq;
	    sampleFreq = cmt3.getSampleFrequency();

	    // set the device output mode for the device(s)

	    for (unsigned int i = 0; i < mtCount; i++) {
		    CmtDeviceMode deviceMode(mode, settings, sampleFreq);
		    if ((deviceIds[i] & 0xFFF00000) != 0x00500000) {
			    // not an MTi-G, remove all GPS related stuff
			    deviceMode.m_outputMode &= 0xFF0F;
		    }
		    res = cmt3.setDeviceMode(deviceMode, true, deviceIds[i]);
		    EXIT_ON_ERROR(res,"setDeviceMode");
	    }

	    // start receiving data
	    res = cmt3.gotoMeasurement();
	    EXIT_ON_ERROR(res,"gotoMeasurement");
    }

    //////////////////////////////////////////////////////////////////////////
    // writeHeaders
    //
    // Write appropriate headers to screen
    void AHRS::writeHeaders(unsigned long mtCount, CmtOutputMode &mode,
		    CmtOutputSettings &settings, int &temperatureOffset, 
		    int &screenSensorOffset)
    {
	    for (unsigned int i = 0; i < mtCount; i++) {
		    printf("MotionTracker %d\n", i + 1);
		    if ((mode & CMT_OUTPUTMODE_TEMP) != 0) {
			    temperatureOffset = 3;
			    printf("Temperature");
			    printf("degrees celcius\n");
		    }

		    if ((mode & CMT_OUTPUTMODE_CALIB) != 0) {
			    printf("Calibrated sensor data");
			    printf(" Acc X\t Acc Y\t Acc Z");
			    printf("(m/s^2)");
			    printf(" Gyr X\t Gyr Y\t Gyr Z");
			    printf("(rad/s)");
			    printf(" Mag X\t Mag Y\t Mag Z");
			    printf("(a.u.)");
		    }

		    if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {
			    printf("Orientation data\n");
			    switch(settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {
			    case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
				    printf("    q0\t    q1\t    q2\t    q3\n");
				    break;
			    case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
				    printf("  Roll\t Pitch\t   Yaw\n");
				    printf("                       degrees\n");
				    break;
			    case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
				    printf(" Matrix\n");
				    break;
			    default:
				    ;
			    }			
		    }

		    if ((mode & CMT_OUTPUTMODE_POSITION) != 0) {
			    printf("\nLongitude\tLatitude\t Altitude\n");
		    }
	    }
    }
