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

        Mat34 moveneutral;
        moveneutral.setIdentity();

        std::vector<Mat34> move;

        Mat34 move1;
        move1.setIdentity();
        move1(2, 3) = -0.05;
        move.push_back(move1);

        Mat34 move2;
        move2.setIdentity();
        move2(1, 3) = 0.05;
        move.push_back(move2);

        Mat34 move3;
        move3.setIdentity();
        move3(1, 3) = -0.05;
        move.push_back(move3);

        Mat34 move4;
        move4.setIdentity();
        move4(0, 3) = 0.05;
        move.push_back(move4);

        Mat34 move5;
        move5.setIdentity();
        move5(0, 3) = -0.05;
        move.push_back(move5);

        Mat34 move6;
        move6.setIdentity();
        move6(2, 3) = 0.12;
        move.push_back(move6);

        Robot* Rob;
        Rob = createRobotMessor("../resources/robotModel.xml");

        Board *demo = createBoardDynamixel();
        std::vector<float_type> configuration, configuration2, configurationneutral;
        std::vector<std::vector<float_type>> configurationtest;
		Mat34 robotPose;
        robotPose.setIdentity();

        vector <float_type> motorSpeed(18,15.0);//set default speed
        demo->setSpeed(motorSpeed);

        // tutaj macie katy 0,24,-114 dla kazdej nogi na sztywno wrzucone
       for (int i = 0; i<6; i++)
      {
            configuration.push_back(0);
            configuration.push_back(24*3.14/180);
            configuration.push_back(-114*3.14/180);
      }


   for (int j=0;j<move.size();j++)
{
   configurationtest.push_back(Rob->movePlatform(move[j]));
         for (int i=0;i<18;i++)
        {//pierwsze serwo powinno otrzymywac wartosci w oklicach zera (niezgodnosc kinematyki robota i sterownika)
             if (configurationtest[j][i]>3.14)
                 configurationtest[j][i]-=6.28;
             else if (configurationtest[j][i]<-3.14)
                 configurationtest[j][i]=+6.28;
        }
   }

         configuration2 = Rob->movePlatform(testmoveplatform);
         configurationneutral = Rob->movePlatform(moveneutral);

        for(int i = 0; i < 6; i++)
        {
            int j = 3 * i;
                demo->setPosition(i, 0, configurationneutral[j] );
                demo->setPosition(i, 1, configurationneutral[j+1] );
                demo->setPosition(i, 2, configurationneutral[j+2] );
        }
        usleep(2000000);

        for (int z;z<configurationtest.size();z++)
        {
        for(int i = 0; i < 6; i++)
        {
            int j = 3 * i;
                demo->setPosition(i, 0, configurationtest[z][j] );
                demo->setPosition(i, 1, configurationtest[z][j+1] );
                demo->setPosition(i, 2, configurationtest[z][j+2] );
        }
                usleep(2000000);
        }

        return 0;
   }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
