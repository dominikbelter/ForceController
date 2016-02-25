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

//    float_type readkat = 0;
//    vector<float_type> readkatLeg;

//    float_type momenty[] = {0, 0, 0};
    vector <float_type> motorSpeed;
    for(int i = 0; i < 18; i++ ){
        motorSpeed.push_back(50);
    }

    Board *robot = createBoardDynamixel();

    robot->setSpeed(motorSpeed);




    std::vector<float_type> wektorTestowy18;

    for(int i=0; i<6;i++){
        wektorTestowy18.push_back((0*M_PI)/180);
        wektorTestowy18.push_back((24*M_PI)/180);
        wektorTestowy18.push_back((114*M_PI)/180);
    }

    robot->setPosition(wektorTestowy18);


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
