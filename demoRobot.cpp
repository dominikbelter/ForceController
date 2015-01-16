#include "include/defs/defs.h"
#include "include/robotModel/robot.h"
#include "include/robotModel/robotMessor2.h"
#include "include/legModel/insectLeg.h"
#include <iostream>
#include <stdio.h>
#include "include/visualization/visualizerIrrlicht.h"
#include <irrlicht.h>
/*
Joregus
Emil
*/
using namespace std;
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

int main( int argc, const char** argv )
{
    try {


        Mat34 testmoveplatform;

		testmoveplatform.setIdentity();
		testmoveplatform(0, 3) = 0;
        testmoveplatform(1, 3) = 0;
        testmoveplatform(2, 3) = 0.12;


        Robot* Rob;
        Rob = createRobotMessor("../resources/robotModel.xml");
        Visualizer* visualizer;
        visualizer = createVisualizerIrrlicht("configVisualization.xml", "TEST");


		std::vector<float_type> configuration, configuration2, Fz;
        // configuration2.push_back(18);






		// setIdentity w bledny sposob tworzy macierz jednostkowa

		Mat34 robotPose;
        robotPose.setIdentity();


        // tutaj macie katy 0,24,-114 dla kazdej nogi na sztywno wrzucone
       for (int i = 0; i<6; i++)
      {
            configuration.push_back(0);
            configuration.push_back(24*3.14/180);
            configuration.push_back(-114*3.14/180);
      }

        //visualizer->drawRobot(robotPose, configuration);


        configuration2 = Rob->movePlatform(testmoveplatform);
        //getchar();

       visualizer->drawRobot(robotPose, configuration2);

        return 0;

   }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
