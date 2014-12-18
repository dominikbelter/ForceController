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
    int deg;
    float_type kat0 = 0;
    float_type kat1 = 0;
    float_type kat2 = 0;

    float_type moment0 = 0;
    float_type moment1 = 0;
    float_type moment2 = 0;

    vector <float_type> motorSpeed;
    for(int i = 0; i < 18; i++ ){
        motorSpeed.push_back(5);
    }
    vector <float_type> motorOffset;
    for(int i = 0; i < 18; i++ ){
        motorOffset.push_back(0);
    }
    vector <float_type> motorComplianceSlope;
    for(int i = 0; i < 18; i++ ){
        motorComplianceSlope.push_back(120);
    }

    Board *demo = createBoardDynamixel();
    demo->readPosition( LEG_1, JOINT_0, kat0);
    demo->readPosition( LEG_1, JOINT_1, kat1);
    demo->readPosition( LEG_1, JOINT_2, kat2);
    demo->setSpeed( motorSpeed );
    demo->setComplianceSlope( motorComplianceSlope );
    demo->setOffset( motorOffset );


    while (true){

     /*   cout<<"Enter degree"<<endl;
        cin>>deg;
        if (deg>100){
            break;
        }
        demo->setPosition(1, 0, (M_PI*deg)/180);   //move 1st leg, 0 joint, about 45deg
        //demo->setPosition(4, 0, (M_PI*deg)/180);   //move 1st leg, 0 joint, about 45deg


        cout << "Kat: "<< (kat*180)/M_PI << endl
              << "Moment: " << moment << endl;
       */
    }

    return 0;
}
