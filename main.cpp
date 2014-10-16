#include "include/defs/defs.h"
#include "include/board/boardDynamixel.h"
#include "include/kinematic/kinematicLie.h"
#include "include/legModel/insectLeg.h"
#include "include/robotModel/robotMessor2.h"
#include "include/visualization/visualizerGL.h"
#include <iostream>
#include <stdio.h>

using namespace std;

int main( int argc, const char** argv )
{
    try {
         Board* board;
         board = createBoardDynamixel();
         std::cout << "Board type: " << board->getName() << "\n";

         Kinematic* kinematicModel;
         kinematicModel = createKinematicLie("../resources/legModel.xml");
         std::cout << "Kinematic type: " << kinematicModel->getName() << "\n";
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
