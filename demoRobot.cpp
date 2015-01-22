#include "include/defs/defs.h"
#include "include/robotModel/robot.h"
#include "include/robotModel/robotMessor2.h"
#include "include/legModel/insectLeg.h"
#include <iostream>
#include <stdio.h>
#include "include/visualization/visualizerIrrlicht.h"
#include <irrlicht.h>
#include "include/board/board.h"
#include "include/board/boardDynamixel.h"
#include "3rdParty/dynamixel/dynamixel.h"
#include "3rdParty/dynamixel/dxl_hal.h"

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


 //******************************
        Mat34 move1;
        move1.setIdentity();
        move1(0, 3) = 0;
        move1(1, 3) = 0;
        move1(2, 3) = 0.12;

        Mat34 move2;
        move2.setIdentity();
        move2(0, 3) = 0;
        move2(1, 3) = 0.1;
        move2(2, 3) = 0;

        Mat34 move3;
        move3.setIdentity();
        move3(0, 3) = 0.1;
        move3(1, 3) = 0;
        move3(2, 3) = 0;

        Mat34 move4;
        move4.setIdentity();
        move4(0, 3) = 0;
        move4(1, 3) = 0;
        move4(2, 3) = 0.12;

        Mat34 move6;
        move6.setIdentity();
        move6(0, 3) = 0;
        move6(1, 3) = 0;
        move6(2, 3) = 0.12;

//*******************************

        Robot* Rob;
        Rob = createRobotMessor("../resources/robotModel.xml");
        //Visualizer* visualizer;
        //visualizer = createVisualizerIrrlicht("configVisualization.xml", "TEST");

        Board *demo = createBoardDynamixel();

        std::vector<float_type> configuration, configuration2, Fz, configurationmove1, configurationmove2, configurationmove3;
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

        configurationmove1 = Rob->movePlatform(move1);


       // configurationmove4 = Rob->movePlatform(move4);
       // configurationmove5 = Rob->movePlatform(move5);
       // configurationmove6 = Rob->movePlatform(move6);
        //getchar();

       //visualizer->drawRobot(robotPose, configuration2);

        /*
        for(int i = 0; i < 6; i++)
        {
            int j = 3 * i;
                demo->setPosition(i, 0, configuration2[j] );
                demo->setPosition(i, 1, configuration2[j+1] );
                demo->setPosition(i, 2, configuration2[j+2] );
        }
        */
        for(int i = 0; i < 6; i++)
        {
            int j = 3 * i;
                demo->setPosition(i, 0, configurationmove1[j] );
                demo->setPosition(i, 1, configurationmove1[j+1] );
                demo->setPosition(i, 2, configurationmove1[j+2] );
        }

        configurationmove2 = Rob->movePlatform(move2);

        usleep(2000000);

        for(int i = 0; i < 6; i++)
        {
            int j = 3 * i;
                demo->setPosition(i, 0, configurationmove2[j] );
                demo->setPosition(i, 1, configurationmove2[j+1] );
                demo->setPosition(i, 2, configurationmove2[j+2] );
        }

        usleep(2000000);

        configurationmove3 = Rob->movePlatform(move3);

        for(int i = 0; i < 6; i++)
        {
            int j = 3 * i;
                demo->setPosition(i, 0, configurationmove3[j] );
                demo->setPosition(i, 1, configurationmove3[j+1] );
                demo->setPosition(i, 2, configurationmove3[j+2] );
        }


        /*
       //leg 0
            demo->setPosition(0, 0, configuration2[0] );
            demo->setPosition(0, 1, configuration2[1] );
            demo->setPosition(0, 2, configuration2[2] );
       //leg 1
            demo->setPosition(1, 0, configuration2[3] );
            demo->setPosition(1, 1, configuration2[4] );
            demo->setPosition(1, 2, configuration2[5] );
       //leg 2
            demo->setPosition(2, 0, configuration2[6] );
            demo->setPosition(2, 1, configuration2[7] );
            demo->setPosition(2, 2, configuration2[8] );
       //leg 3
            demo->setPosition(3, 0, configuration2[9] );
            demo->setPosition(3, 1, configuration2[10] );
            demo->setPosition(3, 2, configuration2[11] );
        // leg 4
            demo->setPosition(4, 0, configuration2[12] );
            demo->setPosition(4, 1, configuration2[13] );
            demo->setPosition(4, 2, configuration2[14] );
        //leg 5
            demo->setPosition(5, 0, configuration2[15] );
            demo->setPosition(5, 1, configuration2[16] );
            demo->setPosition(5, 2, configuration2[17] );

        */
        return 0;

   }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
