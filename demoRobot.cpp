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


        Mat34 testmoveplatform, testmoveplatform2;

		testmoveplatform.setIdentity();
		testmoveplatform(0, 3) = 0;
        testmoveplatform(1, 3) = 0.14;
		testmoveplatform(2, 3) = 0;

		testmoveplatform2.setIdentity();
		testmoveplatform2(0, 3) = 0;
		testmoveplatform2(1, 3) = 0;
		testmoveplatform2(2, 3) = 0;

        Robot* Rob;
        Rob = createRobotMessor("../resources/robotModel.xml");
        Visualizer* visualizer;
        visualizer = createVisualizerIrrlicht("configVisualization.xml", "TEST");


		std::vector<float_type> configuration, configuration2, Fz;
        configuration.push_back(18);
       configuration = Rob->movePlatform(testmoveplatform);
       //cout << configuration[0] << endl << configuration[1] << endl << configuration[2] << endl;
        //cout << configuration[3] << endl << configuration[4] << endl << configuration[5] << endl;
        //cout << configuration[12] << endl << configuration[13] << endl << configuration[14] << endl;
        //cout << configuration[15] << endl << configuration[16] << endl << configuration[17] << endl;




		// setIdentity w bledny sposob tworzy macierz jednostkowa

		Mat34 robotPose;

		for (int i = 0; i<4; i++) {
			for (int j = 0; j<4; j++) {
				robotPose(i, j) = 0;
			}
		}

		robotPose(0, 0) = 1;
		robotPose(1, 1) = 1;
		robotPose(2, 2) = 1;
		robotPose(3, 3) = 1;


/*
        // tutaj macie katy 0,24,-114 dla kazdej nogi na sztywno wrzucone
       for (int i = 0; i<6; i++)
      {
           configuration.push_back(0);
            configuration.push_back(0.4189);
            configuration.push_back(-1.1989);
    }
*/
        visualizer->drawRobot(robotPose, configuration);
		//visualizer->drawRobot(robotPose, configuration2);
        return 0;

   }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
