#include "include/defs/defs.h"
#include "include/visualization/visualizerIrrlicht.h"
#include <iostream>
#include <stdio.h>
#include <irrlicht.h>


// Irrlicht installation
//
// sudo apt-get install libIrrlicht1.8 libIrrlicht-dev

using namespace controller;

using namespace std;
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;





int main(int argc, const char** argv)
{
    Visualizer* visualizer;

    try {
        visualizer= createVisualizerIrrlicht("Test", 1920, 1024);
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    Mat34 robotPose;
    std::vector<float_type> configuration;

    for(int i=0; i<18 ; i++) {
        configuration.push_back(PI/10);
    }



    visualizer->drawRobot(robotPose, configuration);

    getchar();

    return 0;

}



