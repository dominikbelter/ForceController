#include "include/defs/defs.h"
#include "include/kinematic/kinematic.h"
#include "../include/kinematic/kinematicLie.h"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace tinyxml2;//--------------------

int main( int argc, const char** argv )
{
    try {
         //Grabber* grabber;
         //grabber = createFileGrabber();
		controller::Kinematic* demoKine;
		demoKine = createKinematicLie("legModel.xml");
		std::cout << "Kinematic type: " << demoKine->getName() << "\n";
		
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
