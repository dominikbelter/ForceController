#include "include/defs/defs.h"
#include "include/board/board.h"
#include "include/board/boardDynamixel.h"
#include "3rdParty/dynamixel/dynamixel.h"
#include "3rdParty/dynamixel/dxl_hal.h"
#include <iostream>
#include <stdio.h>
using namespace std;
using namespace controller;
int main( int argc, const char** argv )
{

    int position = 0;
    float_type readAngle[3];

//    float_type readkat = 0;
//    vector<float_type> readkatLeg;

//    float_type momenty[] = {0, 0, 0};
    vector <float_type> motorSpeed;
    for(int i = 0; i < 18; i++ ){
        motorSpeed.push_back(5);
    }

    Board *robot = createBoardDynamixel();

    robot->setSpeed(motorSpeed);




    std::vector<float_type> position1_0;
    std::vector<float_type> position1_1;
    std::vector<float_type> position1_2;

    std::vector<float_type> position2_0;
    std::vector<float_type> position2_1;
    std::vector<float_type> position2_2;

    std::vector<float_type> position3_0;
    std::vector<float_type> position3_1;
    std::vector<float_type> position3_2;

    //INITIAL POSITION//
    position1_0.push_back((30*M_PI)/180);
    position1_0.push_back((24*M_PI)/180);
    position1_0.push_back((114*M_PI)/180);

    position1_1.push_back((0*M_PI)/180);
    position1_1.push_back((24*M_PI)/180);
    position1_1.push_back((114*M_PI)/180);

    position1_2.push_back((-30*M_PI)/180);
    position1_2.push_back((24*M_PI)/180);
    position1_2.push_back((114*M_PI)/180);

    //3 LEGS UP//
    position2_0.push_back((30*M_PI)/180);
    position2_0.push_back((44*M_PI)/180);
    position2_0.push_back((114*M_PI)/180);

    position2_1.push_back((0*M_PI)/180);
    position2_1.push_back((44*M_PI)/180);
    position2_1.push_back((114*M_PI)/180);

    position2_2.push_back((-30*M_PI)/180);
    position2_2.push_back((44*M_PI)/180);
    position2_2.push_back((114*M_PI)/180);

    //2 LEGS BACK (BOARD FORWARD)//
    position3_0.push_back((10*M_PI)/180);
    position3_0.push_back((24*M_PI)/180);
    position3_0.push_back((114*M_PI)/180);

    position3_1.push_back((-20*M_PI)/180);
    position3_1.push_back((24*M_PI)/180);
    position3_1.push_back((114*M_PI)/180);

    position3_2.push_back((-50*M_PI)/180);
    position3_2.push_back((24*M_PI)/180);
    position3_2.push_back((114*M_PI)/180);


    robot->setPosition(0, position1_0);
    robot->setPosition(1, position1_1);
    robot->setPosition(2, position1_2);
    robot->setPosition(3, position1_2);
    robot->setPosition(4, position1_1);
    robot->setPosition(5, position1_0);

    usleep(3000000);



    usleep(3000000);



    usleep(3000000);



    usleep(3000000);



    usleep(3000000);



    usleep(3000000);




    while(true)
        {


    while(position == 0)
    {
        robot->readPosition(5, 0, readAngle[0], true);
        robot->readPosition(5, 1, readAngle[1], true);
        robot->readPosition(5, 2, readAngle[2], true);

//        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
//        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
//        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;
	
        if((abs(readAngle[0] - position1_0[0]) < 0.04) && (abs(readAngle[1] - position1_0[1]) < 0.04) && (abs(readAngle[2] - position1_0[2]) < 0.04) )
        {
            position = 1;
            robot->setPosition(0, position2_0);
            robot->setPosition(4, position2_1);
            robot->setPosition(2, position2_2);
        }
    }



    while(position == 1)
    {
        robot->readPosition(0, 0, readAngle[0], true);
        robot->readPosition(0, 1, readAngle[1], true);
        robot->readPosition(0, 2, readAngle[2], true);

//        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
//        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
//        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;

        if((abs(readAngle[0] - position2_0[0]) < 0.04) && (abs(readAngle[1] - position2_0[1]) < 0.04) && (abs(readAngle[2] - position2_0[2]) < 0.04) )
        {
            position = 2;
            robot->setPosition(5, position3_0);
            robot->setPosition(1, position3_1);
            robot->setPosition(3, position3_2);
        }
    }

    while(position == 2)
    {
        robot->readPosition(5, 0, readAngle[0], true);
        robot->readPosition(5, 1, readAngle[1], true);
        robot->readPosition(5, 2, readAngle[2], true);

//        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
//        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
//        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;

        if((abs(readAngle[0] - position3_0[0]) < 0.04) && (abs(readAngle[1] - position3_0[1]) < 0.04) && (abs(readAngle[2] - position3_0[2]) < 0.04) )
        {
            position = 3;
            robot->setPosition(0, position1_0);
            robot->setPosition(4, position1_1);
            robot->setPosition(2, position1_2);
        }
    }

    while(position == 3)
    {
        robot->readPosition(0, 0, readAngle[0], true);
        robot->readPosition(0, 1, readAngle[1], true);
        robot->readPosition(0, 2, readAngle[2], true);

//        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
//        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
//        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;

        if((abs(readAngle[0] - position1_0[0]) < 0.04) && (abs(readAngle[1] - position1_0[1]) < 0.04) && (abs(readAngle[2] - position1_0[2]) < 0.04) )
        {
            position = 4;
            robot->setPosition(5, position2_0);
            robot->setPosition(1, position2_1);
            robot->setPosition(3, position2_2);
        }
    }

    while(position == 4)
    {
        robot->readPosition(5, 0, readAngle[0], true);
        robot->readPosition(5, 1, readAngle[1], true);
        robot->readPosition(5, 2, readAngle[2], true);

//        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
//        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
//        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;

        if((abs(readAngle[0] - position2_0[0]) < 0.04) && (abs(readAngle[1] - position2_0[1]) < 0.04) && (abs(readAngle[2] - position2_0[2]) < 0.04) )
        {
            position = 5;
            robot->setPosition(0, position3_0);
            robot->setPosition(4, position3_1);
            robot->setPosition(2, position3_2);
        }
    }

    while(position == 5)
    {
        robot->readPosition(0, 0, readAngle[0], true);
        robot->readPosition(0, 1, readAngle[1], true);
        robot->readPosition(0, 2, readAngle[2], true);

//        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
//        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
//        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;

        if((abs(readAngle[0] - position3_0[0]) < 0.04) && (abs(readAngle[1] - position3_0[1]) < 0.04) && (abs(readAngle[2] - position3_0[2]) < 0.04) )
        {
            position = 0;
            robot->setPosition(5, position1_0);
            robot->setPosition(1, position1_1);
            robot->setPosition(3, position1_2);
        }
    }



    }

    return 0;
}
