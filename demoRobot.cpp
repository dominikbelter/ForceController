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
#include <thread>

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

Visualizer* visualizer;

void drawRobot(){
    std::vector<double> configuration(18,0);
    for (int i=0;i<6;i++){
        configuration.push_back(0);
        configuration.push_back(24*3.14/180);
        configuration.push_back(-114*3.14/180);
    }
    visualizer->drawRobot(Mat34::Identity(), configuration);
}

int main( int argc, const char** argv ){
    try {
        Robot* robot;
        robot = createRobotMessor("../resources/robotModel.xml");
        visualizer = createVisualizerIrrlicht("VisualizerWindow", 1024, 768, 0.01, false);

        Board *demo = createBoardDynamixel();
        std::vector<float_type> configuration;
        Mat34 robotPose(Mat34::Identity());

        //vector<float_type> motorSpeed(18,15.0);//set default speed
        //demo->setSpeed(motorSpeed);

        // tutaj macie katy 0,24,-114 dla kazdej nogi na sztywno wrzucone
        for (int i = 0; i<6; i++){
            configuration.push_back(0);
            configuration.push_back(24*3.14/180);
            configuration.push_back(-114*3.14/180);
        }
        visualizer->setPosition(configuration);

        std::thread visuThr(drawRobot);
        /*for (int i=0;i<45;i++){
            usleep(2000000);
            configuration[15]=i*3.14/180;
            visualizer->setPosition(configuration);
        }*/
        usleep(2000000);
        Mat34 motion(Mat34::Identity());
        motion(2,3)=0.07;
        configuration = robot->movePlatform(motion);
        visualizer->setPosition(configuration);

        /*for (int j=0;j<move.size();j++){
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
*/
        visuThr.join();
        return 0;
   }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
