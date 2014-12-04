#include "include/defs/defs.h"
#include "include/board/board.h"
#include "include/board/boardDynamixel.h"
#include "3rdParty/dynamixel/dynamixel.h"
#include "3rdParty/dynamixel/dxl_hal.h"
#include <iostream>
#include <stdio.h>
// test Ubuntu Limek
// Limek - Michał Limański 123

// Limek
//pelka
// pawcio

using namespace std;
using namespace controller;

int main( int argc, const char** argv )
{
    int deg;

    Board *demo = createBoardDynamixel();
    while (true){

        cout<<"Enter degree"<<endl;
        cin>>deg;
        if (deg>100){
            break;
        }
        demo->setPosition(1, 0, (M_PI*deg)/180);   //move 1st leg, 0 joint, about 45deg
    }


    return 0;
}
