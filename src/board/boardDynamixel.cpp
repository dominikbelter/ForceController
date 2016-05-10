
#include "../include/board/boardDynamixel.h"
#include <iostream>
#include "../3rdParty/dynamixel/dynamixel.h"
#include "../3rdParty/dynamixel/dxl_hal.h"
#include "board/board.h"
#include <fcntl.h>
#include <mutex>

using namespace controller;
using namespace std;
/// A single instance of BoardDynamixel
BoardDynamixel::Ptr boardDynamixel;

BoardDynamixel::BoardDynamixel(void) : Board("Board Dynamixel", TYPE_USB2DYNAMIXEL) {
    //Every operation is executed on two objects in the same time (One object on one side of port)
    for(int i=0 ; i < 2 ; i++){
       int result =  dynamixelMotors[i].dxl_initialize(i+1, DEFAULT_BAUDNUM);
       for (int j=0;j<6;j++){   //deafault values for servos
                  if(result == 1) {
                      for (int k=0;k<3;k++) {
                          sendCommand(WRITE_WORD,i,j*10+k, P_MOVING_SPEED_L,512);
                          sendCommand(WRITE_WORD,i,j*10+k, P_CW_COMPLIANCE_MARGIN, 1);
                          sendCommand(WRITE_WORD,i,j*10+k, P_CCW_COMPLIANCE_MARGIN, 1);
                          sendCommand(WRITE_WORD,i,j*10+k, P_CW_COMPLIANCE_SLOPE, 32);
                          sendCommand(WRITE_WORD,i,j*10+k, P_CCW_COMPLIANCE_SLOPE, 32);
                          sendCommand(WRITE_WORD,i,j*10+k, P_TORQUE_LIMIT_L, 1012);
                          sendCommand(WRITE_WORD,i,j*10+k, P_TEMERATURE_LIMIT_L, 99);
                          /*dynamixelMotors[i].dxl_write_word(j*10+k, P_MOVING_SPEED_L, 512);
                          dynamixelMotors[i].dxl_write_word(j*10+k, P_CW_COMPLIANCE_MARGIN, 1);
                          dynamixelMotors[i].dxl_write_word(j*10+k, P_CCW_COMPLIANCE_MARGIN, 1);
                          dynamixelMotors[i].dxl_write_word(j*10+k, P_CW_COMPLIANCE_SLOPE, 32);
                          dynamixelMotors[i].dxl_write_word(j*10+k, P_CCW_COMPLIANCE_SLOPE, 32);
                          dynamixelMotors[i].dxl_write_word(j*10+k, P_TORQUE_LIMIT_L, 1012);
                          dynamixelMotors[i].dxl_write_word(j*10+k, P_TEMERATURE_LIMIT_L, 99);*/
                      }
                  }
              }
    }

    zero_angle[0]=450; zero_angle[1]=240; zero_angle[2]=1140;
    zero_angle[3]=0; zero_angle[4]=240; zero_angle[5]=1140;
    zero_angle[6]=-480; zero_angle[7]=240; zero_angle[8]=1140;
    zero_angle[9]=420; zero_angle[10]=-240; zero_angle[11]=-1140;
    zero_angle[12]=0; zero_angle[13]=-240; zero_angle[14]=-1140;
    zero_angle[15]=-450; zero_angle[16]=-240; zero_angle[17]=-1140;

    angle_offset[0]=-30; angle_offset[1]=30; angle_offset[2]=80;
    angle_offset[3]=265; angle_offset[4]=20; angle_offset[5]=-30;
    angle_offset[6]=-30; angle_offset[7]=125; angle_offset[8]=0;
    angle_offset[9]=60; angle_offset[10]=40; angle_offset[11]=0;
    angle_offset[12]=-190; angle_offset[13]=45; angle_offset[14]=80;
    angle_offset[15]=10; angle_offset[16]=75; angle_offset[17]=100;
}

BoardDynamixel::~BoardDynamixel(void) {
    for(int i = 0; i < 2;  i++){
        //CDynamixel *pointMotor = &dynamixelMotors[i];
        //pointMotor->dxl_terminate();     //end of transmision
        sendCommand(TERMINATE,i, 0,0,0);
    }
}


/// Set reference position value for servomotor, returns error value
unsigned int BoardDynamixel::setPosition(unsigned char legNo, unsigned char jointNo, float_type angle){
    if(legNo<3 && jointNo==2){
         angle=-angle;
    }


    if(legNo>2 && jointNo==1){
         angle=-angle;
    }

    angle = angle * _DEG2RAD10;


    angle=-(angle+angle_offset[legNo*3+jointNo]-zero_angle[legNo*3+jointNo])* _DEG2DYNAMIXEL + _stalePrzesuniecie;


    //CDynamixel *pointMotor = &dynamixelMotors[ legNo < 3 ?0:1];
    //pointMotor->dxl_write_word(legNo*10+jointNo, MOVE_SERWOMOTOR, angle);
    sendCommand(WRITE_WORD,legNo < 3 ?0:1,legNo*10+jointNo, MOVE_SERWOMOTOR, angle);
    return 0;
}


unsigned int BoardDynamixel::sendCommand(int dynamixelCmd, int usb2dynNo, int servoNo, int command, float value){

    std::mutex mtx;
    //READ_WORD
    if(dynamixelCmd == 0){

        if (usb2dynNo==0){
          mtx.lock();
          dynamixelMotors[0].dxl_read_word(servoNo, command);
          mtx.unlock();
        }
        else if (usb2dynNo==1){
          mtx.lock();
          dynamixelMotors[1].dxl_read_word(servoNo, command);
          mtx.unlock();
        }
    }
    //WRITE_WORD
    else if(dynamixelCmd ==1){

        if (usb2dynNo==0){
          mtx.lock();
          dynamixelMotors[0].dxl_write_word(servoNo, command, value);
          mtx.unlock();
        }
        else if (usb2dynNo==1){
          mtx.lock();
          dynamixelMotors[1].dxl_write_word(servoNo, command, value);
          mtx.unlock();
        }
    }
    //READ_BYTE
    else if(dynamixelCmd ==2){

        if (usb2dynNo==0){
          mtx.lock();
          dynamixelMotors[0].dxl_read_byte(servoNo, command);
          mtx.unlock();
        }
        else if (usb2dynNo==1){
          mtx.lock();
          dynamixelMotors[1].dxl_read_byte(servoNo, command);
          mtx.unlock();
        }
    }
    //WRITE_BYTE
    else if(dynamixelCmd ==3){
        if (usb2dynNo==0){
          mtx.lock();
          dynamixelMotors[0].dxl_write_byte(servoNo, command, value);
          mtx.unlock();
        }
        else if (usb2dynNo==1){
          mtx.lock();
          dynamixelMotors[1].dxl_write_byte(servoNo, command, value);
          mtx.unlock();
        }
    }
    //INITIALIZE
    else if(dynamixelCmd ==4){


        if (usb2dynNo==0){
          mtx.lock();
          dynamixelMotors[0].dxl_initialize(servoNo, command);
          mtx.unlock();
        }
        else if (usb2dynNo==1){
          mtx.lock();
          dynamixelMotors[1].dxl_initialize(servoNo, command);
          mtx.unlock();
        }
    }
    //TERMINATE
    else if(dynamixelCmd ==5){

        if (usb2dynNo==0){
          mtx.lock();
          dynamixelMotors[0].dxl_terminate();
          mtx.unlock();
        }
        else if (usb2dynNo==1){
          mtx.lock();
          dynamixelMotors[1].dxl_terminate();
          mtx.unlock();
        }
    }
 }


/// Set reference position value for servomotors, returns error value
unsigned int BoardDynamixel::setPosition(unsigned char legNo, const std::vector<float_type>& angle){
    vector <float_type> angleLocal;
    // angleLocal(angle.begin(), angle.begin()+3); //nie dziala
    for(int i = 0; i < 3 ; i++){    //typical duplication of vector doesnt work :(
        if (legNo >2)
            angleLocal.push_back( -angle[i] );
        else
            angleLocal.push_back( angle[i] );
    }

    //CDynamixel *pointMotor = &dynamixelMotors[ legNo < 3 ?0:1 ];

    for(int i=0; i<3; i++ ){    // i -> jointNo

        angleLocal[i] = angleLocal[i]*_DEG2RAD10;
        angleLocal[i]=-(angleLocal[i]+angle_offset[legNo*3+i]-zero_angle[legNo*3+i])*0.341333 + 512;

        //pointMotor->dxl_write_word(legNo*10+i, MOVE_SERWOMOTOR, angleLocal[i]);
        sendCommand(WRITE_WORD, legNo < 3 ?0:1, legNo*10+i, MOVE_SERWOMOTOR, angleLocal[i]);
    }
    return 0;
}

/// Set reference position value for servomotors, returns error value
unsigned int BoardDynamixel::setPosition(const std::vector<float_type>& angle){
    //CDynamixel *object1=&dynamixelMotors[0];
    //CDynamixel *object2=&dynamixelMotors[1];
    vector<float_type> angle_tmp;
    for(int i=0; i<18; i++){
        angle_tmp.push_back(1);
    }
    int cnt = 0;
    int tmp = 0;
    for(int i=0;i<18;i++){
        if(i<9){
            tmp=i%3;
            if(!(i%3) && i!=0){
                cnt++;
            }
            angle_tmp[i] = angle[i]*180/M_PI;
            if(i==2||i==5||i==8)
                    angle_tmp[i]*=(-1);
            angle_tmp[i] = angle_tmp[i]*10;
            angle_tmp[i]=-(angle_tmp[i]+angle_offset[cnt*3+tmp]-zero_angle[cnt*3+tmp])*0.341333 + 512;
            //object1->dxl_write_word(cnt*10+tmp,MOVE_SERWOMOTOR,angle_tmp[i]);
            sendCommand(WRITE_WORD, 0, cnt*10+tmp,MOVE_SERWOMOTOR,angle_tmp[i]);
        }
            else{
            tmp=i%3;
            if(!(i%3) && i!=0){
                cnt++;
                }

            angle_tmp[i] = angle[i]*180/M_PI;

            if(i==10||i==13||i==16)
                angle_tmp[i]*=(-1);

            angle_tmp[i] = angle_tmp[i]*10;
            angle_tmp[i]=-(angle_tmp[i]+angle_offset[cnt*3+tmp]-zero_angle[cnt*3+tmp])*0.341333 + 512;
            //object2->dxl_write_word(cnt*10+tmp,MOVE_SERWOMOTOR,angle_tmp[i]);
            sendCommand(WRITE_WORD, 1, cnt*10+tmp,MOVE_SERWOMOTOR,angle_tmp[i]);
            }
        }

    return 0;
}

/// Set reference speed value for servomotor, returns error value
unsigned int BoardDynamixel::setSpeed(unsigned char legNo, unsigned char jointNo, float_type speed){
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
            //object->dxl_write_word(legNo*10 + jointNo, MOVING_SPEED, speed*9);
            sendCommand(WRITE_WORD, legNo <3 ?0:1, legNo*10 + jointNo, MOVING_SPEED, speed*9);
    return 0;
}

/// Set reference speed value for servomotors, returns error value
unsigned int BoardDynamixel::setSpeed(unsigned char legNo, const std::vector<float_type>& speed){
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    for(int i=0;i<3;i++){
        //object->dxl_write_word(legNo*10+i,MOVING_SPEED,speed[i]*9);
        sendCommand(WRITE_WORD, legNo < 3 ?0:1, legNo*10+i,MOVING_SPEED,speed[i]*9);
    }

    return 0;
}

/// Set reference speed value for servomotors, returns error value
unsigned int BoardDynamixel::setSpeed(const std::vector<float_type>& speed){
    //CDynamixel *object1=&dynamixelMotors[0];
    //CDynamixel *object2=&dynamixelMotors[1];
    int cnt = 0;
    int tmp = 0;
    for(int i=0;i<18;i++){
        if(i<9){
            tmp=i%3;
            if(!(i%3) && i!=0){
                cnt++;
            }
            //object1->dxl_write_word(cnt*10+tmp,MOVING_SPEED,speed[i]*9);
            sendCommand(WRITE_WORD, 0, cnt*10+tmp,MOVING_SPEED,speed[i]*9);
        }
            else{
            tmp=i%3;
            if(!(i%3) && i!=0){
                cnt++;
                }
            //object2->dxl_write_word(cnt*10+tmp,MOVING_SPEED,speed[i]*9);
            sendCommand(WRITE_WORD, 1, cnt*10+tmp,MOVING_SPEED,speed[i]*9);
        }
        }
    return 0;
}

/// Set compliance margin [0,254]- dead zone -- for this area the torque is zero, returns error value
unsigned int BoardDynamixel::setComplianceMargin(unsigned char legNo, unsigned char jointNo, float_type margin){
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    //object->dxl_write_byte(legNo*10 + jointNo, P_CCW_COMPLIANCE_MARGIN, margin);
    //object->dxl_write_byte(legNo*10 + jointNo, P_CW_COMPLIANCE_MARGIN, margin);
    sendCommand(WRITE_BYTE, legNo < 3 ?0:1, legNo*10 + jointNo, P_CCW_COMPLIANCE_MARGIN, margin);
    sendCommand(WRITE_BYTE, legNo < 3 ?0:1, legNo*10 + jointNo, P_CW_COMPLIANCE_MARGIN, margin);
    return 0;
}

/// Set compliance margins [0,254]- dead zone -- for this area the torque is zero, returns error value
unsigned int BoardDynamixel::setComplianceMargin(unsigned char legNo, const std::vector<float_type> margin){
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    for(int i=0;i<3;i++)
    {
        //object->dxl_write_byte(legNo*10 + i, P_CCW_COMPLIANCE_MARGIN, margin[i]);
        //object->dxl_write_byte(legNo*10 + i, P_CW_COMPLIANCE_MARGIN, margin[i]);
        sendCommand(WRITE_BYTE, legNo < 3 ?0:1, legNo*10 + i, P_CCW_COMPLIANCE_MARGIN, margin[i]);
        sendCommand(WRITE_BYTE, legNo < 3 ?0:1, legNo*10 + i, P_CW_COMPLIANCE_MARGIN, margin[i]);
    }
    return 0;
}

/// Set compliance margins [0,254]- dead zone -- for this area the torque is zero, returns error value
unsigned int BoardDynamixel::setComplianceMargin(const std::vector<float_type> margin){
    //CDynamixel *object1 = &dynamixelMotors[0];
    //CDynamixel *object2 = &dynamixelMotors[1];
    int tmp = 0;
    int cnt = 0;
    for(int i = 0; i < 18; i++){
        if(i < 9){  //Right side of Mesor
            tmp = i%3;
            if(!(i%3) && i != 0){
                cnt++;
            }
            //object1->dxl_write_byte(cnt*10+tmp, P_CCW_COMPLIANCE_MARGIN, margin[i]);
            //object2->dxl_write_byte(cnt*10+tmp, P_CW_COMPLIANCE_MARGIN, margin[i]);
            sendCommand(WRITE_BYTE, 0, cnt*10+tmp, P_CCW_COMPLIANCE_MARGIN, margin[i]);
            sendCommand(WRITE_BYTE, 0, cnt*10+tmp, P_CW_COMPLIANCE_MARGIN, margin[i]);
        }else{      //Left side of Mesor
            tmp = i%3;
            if(!i%3){
                cnt++;
            }
            //object1->dxl_write_byte(cnt*10+tmp, P_CCW_COMPLIANCE_MARGIN, margin[i]);
            //object2->dxl_write_byte(cnt*10+tmp, P_CW_COMPLIANCE_MARGIN, margin[i]);
            sendCommand(WRITE_BYTE, 1, cnt*10+tmp, P_CCW_COMPLIANCE_MARGIN, margin[i]);
            sendCommand(WRITE_BYTE, 1, cnt*10+tmp, P_CW_COMPLIANCE_MARGIN, margin[i]);
        }

    }
    return 0;
}

/// Set compiance slope [1,254] - the area with the reduced torque, returns error value
unsigned int BoardDynamixel::setComplianceSlope(unsigned char legNo, unsigned char jointNo, float_type slope){
    if(slope < 1){
        slope = 1;
    }else if(slope > 254){
        slope = 254;
    }
    //CDynamixel *pMotor = &dynamixelMotors[ legNo < 3 ?0:1 ];
    //pMotor->dxl_write_byte(legNo*10+jointNo, P_CW_COMPLIANCE_SLOPE, slope);
    //pMotor->dxl_write_byte(legNo*10+jointNo, P_CCW_COMPLIANCE_SLOPE, slope);
    sendCommand(WRITE_BYTE, legNo < 3 ? 0:1, legNo*10+jointNo, P_CW_COMPLIANCE_SLOPE, slope);
    sendCommand(WRITE_BYTE, legNo < 3 ? 0:1, legNo*10+jointNo, P_CW_COMPLIANCE_SLOPE, slope);
    return 0;
}

/// Set compiance slope [1,254] - the area with the reduced torque, returns error value
unsigned int BoardDynamixel::setComplianceSlope(unsigned char legNo, const std::vector<float_type>& slope){

    //CDynamixel *pMotor = &dynamixelMotors[ legNo < 3 ?0:1 ];
    for(int i = 0; i < 3; i++){     // i -> jointNo
        //pMotor->dxl_write_byte(legNo*10+i, P_CW_COMPLIANCE_SLOPE, slope[i]);
        //pMotor->dxl_write_byte(legNo*10+i, P_CCW_COMPLIANCE_SLOPE, slope[i]);
        sendCommand(WRITE_BYTE, legNo < 3 ? 0:1, legNo*10+i, P_CW_COMPLIANCE_SLOPE, slope[i]);
        sendCommand(WRITE_BYTE, legNo < 3 ? 0:1, legNo*10+i, P_CCW_COMPLIANCE_SLOPE, slope[i]);
    }
    return 0;
}

/// Set compiance slope [1,254] - the area with the reduced torque, returns error value
unsigned int BoardDynamixel::setComplianceSlope(const std::vector<float_type>& slope){
    //CDynamixel *pMotorR = &dynamixelMotors[0];
    //CDynamixel *pMotorL = &dynamixelMotors[1];
    int tmp = 0;
    int cnt = 0;
    for(int i = 0; i < 18; i++){
        if(i < 9){  //Right side of Mesor
            tmp = i%3;
            if(!(i%3) && i != 0){
                cnt++;
            }
            //pMotorR->dxl_write_byte(cnt*10+tmp, P_CW_COMPLIANCE_SLOPE, slope[i]);
            //pMotorR->dxl_write_byte(cnt*10+tmp, P_CCW_COMPLIANCE_SLOPE, slope[i]);
            sendCommand(WRITE_BYTE, 0, cnt*10+tmp, P_CW_COMPLIANCE_SLOPE, slope[i]);
            sendCommand(WRITE_BYTE, 0, cnt*10+tmp, P_CCW_COMPLIANCE_SLOPE, slope[i]);
        }else{      //Left side of Mesor
            tmp = i%3;
            if(!i%3){
                cnt++;
            }
            //pMotorL->dxl_write_byte(cnt*10+tmp, P_CW_COMPLIANCE_SLOPE, slope[i]);
            //pMotorL->dxl_write_byte(cnt*10+tmp, P_CCW_COMPLIANCE_SLOPE, slope[i]);
            sendCommand(WRITE_BYTE, 1, cnt*10+tmp, P_CW_COMPLIANCE_SLOPE, slope[i]);
            sendCommand(WRITE_BYTE, 1, cnt*10+tmp, P_CCW_COMPLIANCE_SLOPE, slope[i]);
        }

    }
    return 0;
}

/// Set torque limit torque_limit [0,1] - the torque limit, returns error value
unsigned int BoardDynamixel::setTorqueLimit(unsigned char legNo, unsigned char jointNo, float_type torqueLimit){
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
            //object->dxl_write_word(legNo*10 + jointNo, SET_TORQUE_LIMIT, torqueLimit*1023);
    sendCommand(WRITE_WORD, legNo < 3 ? 0:1, legNo*10 + jointNo, SET_TORQUE_LIMIT, torqueLimit*1023);
    return 0;
}

/// Set torque limit torque_limit [0,1] - the torque limit, returns error value
unsigned int BoardDynamixel::setTorqueLimit(unsigned char legNo, const std::vector<float_type>& torqueLimit){
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    for(int i=0;i<3;i++)
            //object->dxl_write_word(legNo*10 + i, SET_TORQUE_LIMIT, torqueLimit[i]*1023);
            sendCommand(WRITE_WORD, legNo < 3 ? 0:1, legNo*10 + i, SET_TORQUE_LIMIT, torqueLimit[i]*1023);
    return 0;
}

/// Set torque limit torque_limit [0,1] - the torque limit, returns error value
unsigned int BoardDynamixel::setTorqueLimit(const std::vector<float_type>& torqueLimit){
    //CDynamixel *object1 = &dynamixelMotors[0];
    //CDynamixel *object2 = &dynamixelMotors[1];
    int tmp = 0;
    int cnt = 0;
    for(int i = 0; i < 18; i++){
        if(i < 9){  //Right side of Mesor
            tmp = i%3;
            if(!(i%3) && i != 0){
                cnt++;
            }
            //object1->dxl_write_word(cnt*10+tmp, SET_TORQUE_LIMIT, torqueLimit[i]*1023);
            sendCommand(WRITE_WORD, 0, cnt*10 + tmp, SET_TORQUE_LIMIT, torqueLimit[i]*1023);
        }else{      //Left side of Mesor
            tmp = i%3;
            if(!i%3){
                cnt++;
            }
            //object2->dxl_write_word(cnt*10+tmp, SET_TORQUE_LIMIT, torqueLimit[i]*1023);
            sendCommand(WRITE_WORD, 1, cnt*10+tmp, SET_TORQUE_LIMIT, torqueLimit[i]*1023);
        }

    }

    return 0;
}

/// Returns current position of the servomotor, returns error value
unsigned int BoardDynamixel::readPosition(unsigned char legNo, unsigned char jointNo, float_type& angle){
    float_type ang_odt;
    float_type ang;
    CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    //cout << "Wartosc z rejestru readPos: " <<object->dxl_read_word(legNo*10+jointNo, P_PRESENT_POSITION_L) << endl;
    ang_odt = object->dxl_read_word(legNo*10 + jointNo, P_PRESENT_POSITION_L);
    //ang_odt = sendCommand(READ_WORD, legNo < 3 ?0:1, legNo*10 + jointNo, P_PRESENT_POSITION_L, 0);
    ang = ((ang_odt-512)/(-0.341333))-angle_offset[legNo*3+jointNo]+zero_angle[legNo*3+jointNo];
    angle=(ang/10)*(M_PI/180);
    if(legNo < 3 && jointNo == 2){
            angle = -angle;
    }
    if(legNo > 2 && jointNo == 1){
            angle = -angle;
    }
    return 0;
}

/// Returns current position of the servomotor, returns error value
unsigned int BoardDynamixel::readPosition(unsigned char legNo, unsigned char jointNo, float_type& angle, bool reversePos1){
    float_type ang_odt;
    float_type ang;
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    //cout << "Wartosc z rejestru readPos: " <<object->dxl_read_word(legNo*10+jointNo, P_PRESENT_POSITION_L) << endl;
    //ang_odt = object->dxl_read_word(legNo*10 + jointNo, P_PRESENT_POSITION_L);
    ang_odt = sendCommand(READ_WORD, legNo < 3 ?0:1, legNo*10 + jointNo, P_PRESENT_POSITION_L, 0);
    ang = ((ang_odt-512)/(-0.341333))-angle_offset[legNo*3+jointNo]+zero_angle[legNo*3+jointNo];
    angle=(ang/10)*(M_PI/180);
    if(reversePos1)
    {
        if(legNo > 2){
                angle = -angle;
        }
    }
    return 0;
}

/// Returns current position of the servomotors, returns error value
unsigned int BoardDynamixel::readPositions(unsigned char legNo, std::vector<float_type>& angle){
    float_type ang_odt;
    float_type ang;
    float_type angleTmp;
    std::vector<float_type> angVec;
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    for(int i = 0; i<3; i++){
        //ang_odt = object->dxl_read_word(legNo*10 + i, P_PRESENT_POSITION_L);
        sendCommand(READ_WORD, legNo < 3 ?0:1, legNo*10+i,P_PRESENT_POSITION_L, 0);
        ang = ((ang_odt-512)/(-0.341333))-angle_offset[legNo*3+i]+zero_angle[legNo*3+i];
        angleTmp=(ang/10)*(M_PI/180);
    if(legNo < 3 && i == 2){
            angleTmp = -angleTmp;
        }
        if(legNo > 2 && i == 1){
            angleTmp = -angleTmp;
        }
        angVec.push_back(angleTmp);
    }

    angle = angVec;

    return 0;
}

/// Returns current position of the servomotor, returns error value
unsigned int BoardDynamixel::readPosition(std::vector<float_type>& angle){
    float_type ang_odt;
    float_type ang;
    float_type angleTmp;
    std::vector<float_type> angVec;
    int tmp=0;// servo no in leg
    int cnt=0;// leg no
    //CDynamixel *object1 = &dynamixelMotors[0];
    //CDynamixel *object2 = &dynamixelMotors[1];
    for(int i=0;i<18;i++){
        if(i<9){
            tmp=i%3;
            if(!(i%3) && i!=0){
                cnt++;
            }
            //ang_odt = object1->dxl_read_word(cnt*10 + tmp, P_PRESENT_POSITION_L);
            ang_odt = sendCommand(READ_WORD, 0, cnt*10 + tmp, P_PRESENT_POSITION_L, 0);
            ang = ((ang_odt-512)/(-0.341333))-angle_offset[cnt*3+tmp]+zero_angle[cnt*3+tmp];
            angleTmp=(ang/10)*(M_PI/180);
            if(cnt < 3 && tmp == 2){
                    angleTmp = -angleTmp;
            }
            if(cnt > 2 && tmp == 1){
                angleTmp = -angleTmp;
            }
            angVec.push_back(angleTmp);
        }//if
        else{
            tmp=i%3;
            if(!(i%3) && i!=0){
                cnt++;
            }
            //ang_odt = object2->dxl_read_word(cnt*10 + tmp, P_PRESENT_POSITION_L);
            ang_odt = sendCommand(READ_WORD, 1, cnt*10 + tmp, P_PRESENT_POSITION_L, 0);
            ang = ((ang_odt-512)/(-0.341333))-angle_offset[cnt*3+tmp]+zero_angle[cnt*3+tmp];
            angleTmp=(ang/10)*(M_PI/180);
            if(cnt < 3 && tmp == 2){
                angleTmp = -angleTmp;
            }

            if(cnt > 2 && tmp == 1){
                angleTmp = -angleTmp;
            }
            angVec.push_back(angleTmp);
        }//else
    }//for
    angle = angVec;

    return 0;
}

/// Returns contact force from 1-axis force sensor
unsigned int BoardDynamixel::readForce(unsigned char legNo, float_type& contactForce){
    return 0;
}

/// Returns contact force from 1-axis force sensor
unsigned int BoardDynamixel::readForce(const std::vector<float_type>& contactForce){
    return 0;
}

/// Returns contact force from 3-axis torque/force sensor
unsigned int BoardDynamixel::readTorqueForce(unsigned char legNo, TorqueForce& valueTF){
    return 0;
}

/// Returns contact force from 3-axis torque/force sensor
unsigned int BoardDynamixel::readTorqueForce(const std::vector<float_type>& valueTF){
    return 0;
}

/// Returns contact or from microswitch
bool BoardDynamixel::readContact(unsigned char legNo){

    int fileDescriptor[6];
    char buffer;
    bool groundContactValue = false;



//    for(int i=0;i<6;i++){
//         char buff[60];
//         sprintf(buff, "/sys/class/gpio/gpio%d/value",i);
//         if ((fileDescriptor[i] = open(buff, O_RDONLY | O_NDELAY, 0))== 0)
//         {
//            printf("Error: Can't open /sys/class/gpio/gpioX/value.\n");
//            exit(1);
//         }
//         //printf("Value opened for writing.\n");
//    }

             char buff[60];
             sprintf(buff, "/sys/class/gpio/gpio%d/value",legNo);
             if ((fileDescriptor[legNo] = open(buff, O_RDONLY | O_NDELAY, 0))== 0)
             {
                printf("Error: Can't open /sys/class/gpio/gpioX/value.\n");
                exit(1);
             }
             //printf("Value opened for writing.\n");
    lseek(fileDescriptor[legNo], 0, SEEK_SET);
    read( fileDescriptor[legNo], &buffer, 1 );


    if ( buffer == '0')
        groundContactValue = true;
    else
        groundContactValue = false;





    return groundContactValue;
}

/// Returns contact or from microswitches
void BoardDynamixel::readContact(const std::vector<bool> contact){

}

/// Returns current from servo
unsigned int BoardDynamixel::readCurrent(unsigned char legNo, unsigned char jointNo, float_type& servoCurrent){
    //CDynamixel *pMotor = &dynamixelMotors[ legNo < 3 ?0:1 ];
    //servoCurrent = pMotor->dxl_read_word(legNo*10 + jointNo, PRESENT_VOLTAGE);
    servoCurrent = sendCommand(READ_WORD, legNo < 3 ? 0:1, legNo*10 + jointNo, PRESENT_VOLTAGE, 0);
    servoCurrent = servoCurrent/10;

    return 0;
}

/// Returns current from servo
unsigned int BoardDynamixel::readCurrent(unsigned char legNo, std::vector<float_type>& servoCurrent){
    //CDynamixel *pMotor = &dynamixelMotors[ legNo < 3 ?0:1 ];
    for(int i=0; i<3; i++){
        //servoCurrent.push_back( pMotor->dxl_read_word(legNo*10 + i, PRESENT_VOLTAGE) );
        servoCurrent.push_back(sendCommand(READ_WORD, legNo < 3 ? 0:1, legNo*10 + i, PRESENT_VOLTAGE, 0));
        servoCurrent[i] = servoCurrent[i] / 10;
    }
    return 0;
}

/// Returns current from servo
unsigned int BoardDynamixel::readCurrent( std::vector<float_type>& servoCurrent){
    //CDynamixel *pMotorR = &dynamixelMotors[0];
    //CDynamixel *pMotorL = &dynamixelMotors[1];
    int tmp = 0;
    int cnt = 0;
    for(int i = 0; i < 18; i++){
        if(i < 9){  //Right side of Mesor
            tmp = i%3;
            if(!(i%3) && i != 0){
                cnt++;
            }
            //servoCurrent.push_back( pMotorR->dxl_read_word( cnt*10+tmp, PRESENT_VOLTAGE ) );
            servoCurrent.push_back(sendCommand(READ_WORD, 0, cnt*10+tmp, PRESENT_VOLTAGE, 0));
        }else{      //Left side of Mesor
            tmp = i%3;
            if(!i%3){
                cnt++;
            }
            //servoCurrent.push_back( pMotorL->dxl_read_word( cnt*10+tmp, PRESENT_VOLTAGE ) );
            servoCurrent.push_back(sendCommand(READ_WORD, 1, cnt*10+tmp, PRESENT_VOLTAGE, 0));
        }
    servoCurrent[i] = servoCurrent[i]/10;
    }
    return 0;
}

/// Returns torque/load from servo
unsigned int BoardDynamixel::readTorque(unsigned char legNo, unsigned char jointNo, float_type& servoTorque){
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    servoTorque = sendCommand(READ_WORD, legNo < 3 ?0:1,legNo*10 + jointNo, TORQUE, 0) * sendCommand(READ_WORD, legNo < 3 ? 0:1, legNo*10+jointNo, GET_MAX_TORQUE, 0)/1024*28.3;
    //servoTorque = object->dxl_read_word(legNo*10 + jointNo, TORQUE)*object->dxl_read_word(legNo*10+jointNo, GET_MAX_TORQUE)/1024*28.3;
     return 0;
}

/// Returns torque/load from servo
unsigned int BoardDynamixel::readTorque(unsigned char legNo,std::vector<float_type>& servoTorque){
   //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    for(int i=0;i<3;i++)
    {
       servoTorque.push_back(sendCommand(READ_WORD, legNo < 3 ?0:1,legNo*10 + i, TORQUE, 0)*sendCommand(READ_WORD, legNo < 3 ? 0:1,legNo*10+i, GET_MAX_TORQUE, 0)/1024*28.3);
       //servoTorque.push_back(object->dxl_read_word(legNo*10 + i, TORQUE)*object->dxl_read_word(legNo*10+i, GET_MAX_TORQUE)/1024*28.3);
    }
    return 0;
}

/// Returns torque/load from servo
unsigned int BoardDynamixel::readTorque(std::vector<float_type>& servoTorque){
    //CDynamixel *object1 = &dynamixelMotors[0];
    //CDynamixel *object2 = &dynamixelMotors[1];
    int tmp = 0;
    int cnt = 0;
    for(int i = 0; i < 18; i++){
        if(i < 9){  //Right side of Mesor
            tmp = i%3;
            if(!(i%3) && i != 0){
                cnt++;
            }
           //servoTorque.push_back(object1->dxl_read_word(cnt*10+tmp, TORQUE)*object1->dxl_read_word(cnt*10+tmp, GET_MAX_TORQUE)/1024*28.3);
           servoTorque.push_back(sendCommand(READ_WORD, 0, cnt*10+tmp, TORQUE, 0)*sendCommand(READ_WORD, 0, cnt*10+tmp, GET_MAX_TORQUE, 0)/1024*28.3);
        }else{      //Left side of Mesor
            tmp = i%3;
            if(!i%3){
                cnt++;
            }
            servoTorque.push_back(sendCommand(READ_WORD, 1, cnt*10+tmp, TORQUE, 0)*sendCommand(READ_WORD, 1, cnt*10+tmp, GET_MAX_TORQUE, 0)/1024*28.3);
            //servoTorque.push_back(object2->dxl_read_word(cnt*10+tmp, TORQUE)*object2->dxl_read_word(cnt*10+tmp, GET_MAX_TORQUE)/1024*28.3);
        }
    }
    return 0;
}

/// Set servo Offset
void BoardDynamixel::setOffset(unsigned char legNo, unsigned char jointNo, float_type offset){
    float_type localOffset;

    localOffset = offset;
    localOffset = localOffset * (180/M_PI);

    angle_offset[legNo*10+jointNo] += localOffset ;

}

/// Set servo Offset
void BoardDynamixel::setOffset(unsigned char legNo, const std::vector<float_type> offset){
    vector <float_type> localOffset;
    for(int i=0; i<3; i++){
       localOffset.push_back( offset[i] );
       localOffset[i] = localOffset[i] * (180/M_PI);
    }

    for(int i=0; i<3; i++){
         angle_offset[legNo*10+i] += localOffset[i];
    }
}

/// Set servo Offset
void BoardDynamixel::setOffset(const std::vector<float_type> offset){
    vector <float_type> localOffset;
    for(int i=0; i<18; i++){
       localOffset.push_back( offset[i] );
       localOffset[i] = localOffset[i] * (180/M_PI);
    }

    int tmp = 0;
    int cnt = 0;
    for(int i = 0; i < 18; i++){
        if(i < 9){  //Right side of Mesor
            tmp = i%3;
            if(!(i%3) && i != 0){
                cnt++;
            }
            angle_offset[cnt*10+tmp] += localOffset[i];
        }else{      //Left side of Mesor
            tmp = i%3;
            if(!i%3){
                cnt++;
            }
            angle_offset[cnt*10+tmp] += localOffset[i];
        }

    }
}

/// Board configuration -- set default value
void BoardDynamixel::setDefault(void){ }

controller::Board* controller::createBoardDynamixel(void) {
    boardDynamixel.reset(new BoardDynamixel());
    return boardDynamixel.get();
}
