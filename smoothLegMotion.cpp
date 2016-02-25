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




    std::vector<float_type> position1;
    std::vector<float_type> position2;


    position1.push_back((0*M_PI)/180);
    position1.push_back((24*M_PI)/180);
    position1.push_back((114*M_PI)/180);

    position2.push_back((30*M_PI)/180);
    position2.push_back((14*M_PI)/180);
    position2.push_back((80*M_PI)/180);

    while(true)
    {
    robot->setPosition(0, position1);
    robot->setPosition(1, position1);
    robot->setPosition(2, position1);
    robot->setPosition(3, position1);
    robot->setPosition(4, position1);
    robot->setPosition(5, position1);

    while(position == 0)
    {
        robot->readPosition(5, 0, readAngle[0], true);
        robot->readPosition(5, 1, readAngle[1], true);
        robot->readPosition(5, 2, readAngle[2], true);

//        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
//        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
//        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;
	
        if((abs(readAngle[0] - position1[0]) < 0.04) && (abs(readAngle[1] - position1[1]) < 0.04) && (abs(readAngle[2] - position1[2]) < 0.04) )
        {
            position = 1;
        }
    }

    robot->setPosition(0, position2);
    robot->setPosition(1, position2);
    robot->setPosition(2, position2);
    robot->setPosition(3, position2);
    robot->setPosition(4, position2);
    robot->setPosition(5, position2);

    while(position == 1)
    {
        robot->readPosition(5, 0, readAngle[0], true);
        robot->readPosition(5, 1, readAngle[1], true);
        robot->readPosition(5, 2, readAngle[2], true);

//        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
//        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
//        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;

        if((abs(readAngle[0] - position2[0]) < 0.04) && (abs(readAngle[1] - position2[1]) < 0.04) && (abs(readAngle[2] - position2[2]) < 0.04) )
        {
            position = 0;
        }
    }
    }

//    while (true){
//        cout << "Noga = ";
//        cin >> noga;
//        cout << "Wezel = ";
//        cin >> wezel;
//        cout << "Podatnosc = ";
//        cin >> podatnosc;
//        robot->setTorqueLimit(noga, wezel, podatnosc);
//        robot->readPosition(noga,wezel,readkat);
//        robot->readPositions(noga,readkatLeg);
//        cout<<"Odczytany kat: "<<readkat*180/M_PI<<endl;
//        cout<<"Odczytane katy w nodze: "<<readkatLeg[0]*180/M_PI<<" "<<readkatLeg[1]*180/M_PI<<" "<<readkatLeg[2]*180/M_PI<<endl;
//    }
    return 0;
}
