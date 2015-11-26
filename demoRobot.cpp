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
        std::string filename = "../../resources/configGlobal.xml";
        tinyxml2::XMLDocument config;
        config.LoadFile(filename.c_str());
        bool useVisualization;
        if (config.FirstChildElement() == nullptr)
            std::cout << "unable to load config global file.\n";
        else {
            tinyxml2::XMLElement * pRoot = config.FirstChildElement("parameters");
            pRoot->QueryBoolAttribute("useVisualization", &useVisualization);
        }
        std::cout << useVisualization << "\n";

        Robot* robot;
        robot = createRobotMessor("../resources/robotModel.xml");
        if (useVisualization)
            visualizer = createVisualizerIrrlicht("VisualizerWindow", 1024, 768, 0.01, false);
        Board *demo;
        if (!useVisualization)
            demo = createBoardDynamixel();
        std::vector<float_type> configuration;
        Mat34 robotPose(Mat34::Identity());

        if (!useVisualization){
            vector<float_type> motorSpeed(18,15.0);//set default speed
            demo->setSpeed(motorSpeed);
        }

        // tutaj macie katy 0,24,-114 dla kazdej nogi na sztywno wrzucone
        for (int i = 0; i<6; i++){
            configuration.push_back(0);
            configuration.push_back(24*3.14/180);
            configuration.push_back(-114*3.14/180);
        }
        std::unique_ptr<std::thread> visuThr;
	std::cout << "output ref: ";
	for (auto& val : configuration){
            if (val>6.14) val-=6.283;
            else if (val<-6.14) val=+6.283;
            std::cout << val << ", ";
        }
	std::cout << "\n";
        if (useVisualization){
            //std::thread visuThr(drawRobot);
            visuThr = std::unique_ptr<std::thread>(new std::thread(drawRobot));
            visualizer->setPosition(configuration);
        }
        else
            demo->setPosition(configuration);

        /*for (int i=0;i<45;i++){
            usleep(2000000);
            configuration[15]=i*3.14/180;
            visualizer->setPosition(configuration);
        }*/
        usleep(2000000);
        Mat34 motion(Mat34::Identity());
        motion(0,3)=0.07;
        configuration = robot->movePlatform(motion);
	std::cout << "output: ";
	for (auto& val : configuration){
            if (val>6.14) val-=6.283;
            else if (val<-6.14) val=+6.283;
            std::cout << val << ", ";
        }
	std::cout << "\n";
	configuration[2]=-0.5;
        if (useVisualization)
            visualizer->setPosition(configuration);
        else
            demo->setPosition(configuration);
        std::cout << "\n";
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
        if (useVisualization)
            visuThr->join();
        return 0;
   }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
