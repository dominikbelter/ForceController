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
    try {
        Board* board = createBoardDynamixel();
         //Grabber* grabber;
         //grabber = createFileGrabber();
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    BoardDynamixel *demo = new BoardDynamixel();
    demo->setPosition(1, 10, M_PI/4);   //move 1st leg, 10 joint, about 45deg
    delete demo;

    return 0;
}
