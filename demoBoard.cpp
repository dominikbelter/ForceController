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
    float_type kat;
    float_type noga;
    float_type wezel;
    float_type podatnosc;
    float_type kat0 = 0;
    float_type kat1 = 0;
    float_type kat2 = 0;
    float_type readkat = 0;
    vector<float_type> readkatLeg;
    float_type moment0 = 0;
    float_type moment1 = 0;
    float_type moment2 = 0;
    vector <float_type> motorSpeed;
    for(int i = 0; i < 18; i++ ){
        motorSpeed.push_back(50);
    }
    vector <float_type> motorOffset;
    for(int i = 0; i < 18; i++ ){
        motorOffset.push_back(0);
    }
    vector <float_type> motorComplianceSlope;
    for(int i = 0; i < 18; i++ ){
        motorComplianceSlope.push_back(60);
    }
    vector <float_type> polozeniePoczatkowe;
    polozeniePoczatkowe.push_back( 0 );
    polozeniePoczatkowe.push_back( 0 );
    polozeniePoczatkowe.push_back( 0 );
    Board *demo = createBoardDynamixel();
    /* demo->readPosition( LEG_1, JOINT_0, kat0);
    demo->readPosition( LEG_1, JOINT_1, kat1);
    demo->readPosition( LEG_1, JOINT_2, kat2);
    demo->setSpeed( motorSpeed );
    demo->setComplianceSlope( motorComplianceSlope );
    demo->setOffset( motorOffset );
    demo->setPosition(LEG_0, polozeniePoczatkowe );
    //demo->setPosition(LEG_1, polozeniePoczatkowe );
    //demo->setPosition(LEG_2, polozeniePoczatkowe );
    //demo->setPosition(LEG_3, polozeniePoczatkowe );
    //demo->setPosition(LEG_4, polozeniePoczatkowe );
    //demo->setPosition(LEG_5, polozeniePoczatkowe );
    demo->readTorque(LEG_0, JOINT_0, moment0 );
    cout << "Katy: " << endl;
    cout << (kat0,*180)/M_PI << endl << (kat1*180)/M_PI << endl << (kat2*180)/M_PI << endl;
    cout << "Momenty: " << endl;
    cout << moment0 << endl << moment1 << endl << moment2 << endl;
    */

    /***** pelka *****/

    //demo->setOffset(5,0,(-20*M_PI)/180);
    //demo->setComplianceSlope( 1, 1, 0x40 );

    /*****************/

    /*** Pawcio *****/
    //demo->setComplianceMargin( 1, 1, 0x40 );
    /**************/

    std::vector<float_type> wektorTestowy3;
    wektorTestowy3.push_back((0*M_PI)/180);
    wektorTestowy3.push_back((24*M_PI)/180);
    wektorTestowy3.push_back((-114*M_PI)/180);

    std::vector<float_type> wektorTestowy18;
    wektorTestowy18.push_back((0*M_PI)/180);
    wektorTestowy18.push_back((24*M_PI)/180);
    wektorTestowy18.push_back((-114*M_PI)/180);
    wektorTestowy18.push_back((0*M_PI)/180);
    wektorTestowy18.push_back((24*M_PI)/180);
    wektorTestowy18.push_back((-114*M_PI)/180);
    wektorTestowy18.push_back((0*M_PI)/180);
    wektorTestowy18.push_back((24*M_PI)/180);
    wektorTestowy18.push_back((-114*M_PI)/180);
    wektorTestowy18.push_back((0*M_PI)/180);
    wektorTestowy18.push_back((-24*M_PI)/180);
    wektorTestowy18.push_back((114*M_PI)/180);
    wektorTestowy18.push_back((0*M_PI)/180);
    wektorTestowy18.push_back((-24*M_PI)/180);
    wektorTestowy18.push_back((114*M_PI)/180);
    wektorTestowy18.push_back((0*M_PI)/180);
    wektorTestowy18.push_back((-24*M_PI)/180);
    wektorTestowy18.push_back((114*M_PI)/180);

    for(int i=0;i<6;i++)
    {
        /*demo->setPosition(i, 0, (0*M_PI)/180 );
        demo->setPosition(i, 1, (24*M_PI)/180 );
        demo->setPosition(i, 2, (-114*M_PI)/180 );*/

        demo->setPosition(i,wektorTestowy3);
    }
    //demo->setPosition(wektorTestowy18);

    while (true){
        cout << "Noga = ";
        cin >> noga;
        cout << "Wezel = ";
        cin >> wezel;
        cout << "Podatnosc = ";
        cin >> podatnosc;
        demo->setTorqueLimit(noga, wezel, podatnosc);
        demo->readPosition(noga,wezel,readkat);
        demo->readPositions(noga,readkatLeg);
        cout<<"Odczytany kat: "<<readkat*180/M_PI<<endl;
        cout<<"Odczytane katy w nodze: "<<readkatLeg[0]*180/M_PI<<" "<<readkatLeg[1]*180/M_PI<<" "<<readkatLeg[2]*180/M_PI<<endl;
    }
    return 0;
}
