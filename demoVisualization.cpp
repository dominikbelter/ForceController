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
      //  visualizer= createVisualizerIrrlicht("Test", 1920, 1024, 0.01, true);
           visualizer= createVisualizerIrrlicht("configVisualization.xml", "TEST");
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }



    Mat34 robotPose;
    std::vector<float_type> configuration;

    for(int i=0; i<18 ; i++) {
        configuration.push_back(PI/8);
    }

 //   (static_cast<VisualizerIrrlicht*>(visualizer))->setDebugMode(true);

    /*
               // Z

             robotPoseTest(0,0) = sqrt2;
             robotPoseTest(0,2) = sqrt2;
             robotPoseTest(2,0) = -sqrt2;
             robotPoseTest(2,2) = sqrt2;
             robotPoseTest(1,1) = 1;
             robotPoseTest(3,3) = 1;

             // X

             robotPoseTest2(0,0) = 1;
             robotPoseTest2(1,1) = sqrt2;
             robotPoseTest2(1,2) = -sqrt2;
             robotPoseTest2(2,1) = sqrt2;
             robotPoseTest2(2,2) = sqrt2;
             robotPoseTest2(3,3) = 1;
    */

/*
               float sqrt2 = sqrt(2)/2;

             robotPose(0,0) = sqrt2;
             robotPose(0,1) = -1/2;
             robotPose(0,2) = 1/2;
             robotPose(1,0) = sqrt2;
             robotPose(1,1) = 1/2;
             robotPose(1,2) = -1/2;
             robotPose(2,1) = sqrt2;
             robotPose(2,2) = sqrt2;

   //          robotPose.Identity();

*/

// setIdentity w bledny sposob tworzy macierz jednostkowa
//DB u mnie działa dobrze: robotPose.setIdentity();

    for(int i=0 ; i<4 ; i++) {
        for(int j=0 ; j<4 ; j++) {
            robotPose(i,j) = 0;
        }
    }

    robotPose(0,0) = 1;
    robotPose(1,1) = 1;
    robotPose(2,2) = 1;
    robotPose(3,3) = 1;


    visualizer->drawRobot(robotPose, configuration);



    return 0;

}



