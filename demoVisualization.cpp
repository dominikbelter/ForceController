#include "include/defs/defs.h"
#include "include/visualization/visualizer.h"
#include <iostream>
#include <stdio.h>
#include <irrlicht.h>


// Irrlicht installation
//
// sudo apt-get install libIrrlicht1.8 libIrrlicht-dev

using namespace std;


using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;



int main( int argc, const char** argv )
{

    IrrlichtDevice *device =
        createDevice( video::EDT_SOFTWARE, dimension2d<u32>(640, 480), 16,
            false, false, false, 0);

    if (!device)
        return 1;

    getchar();

    try {
         //Grabber* grabber;
         //grabber = createFileGrabber();
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
