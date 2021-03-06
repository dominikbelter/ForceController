#include "include/defs/defs.h"
#include "include/board/boardDynamixel.h"
#include "include/kinematic/kinematicLie.h"
#include "include/legModel/insectLeg.h"
#include "include/robotModel/robotMessor2.h"
#include "include/visualization/visualizerGL.h"
#include "include/visualization/visualizerIrrlicht.h"
#include <iostream>
#include <stdio.h>
#include <thread>
#include <time.h>

#define __DO_NOT_USE_VISUALIZER__


/*
 Paulina Jankowska
 Tomasz Chrosniak
 */

using namespace std;

#ifndef __DO_NOT_USE_VISUALIZER__

struct visualizationPointers
{
	Board* board;
	Visualizer* visualizer;
};

void updatePlatformPosition(visualizationPointers* args)
{
	Mat34 robotPose;
	visualizationPointers *ptrs = args;
	Board* board = ptrs->board;
	timespec requestedTime;
	requestedTime.tv_nsec = 200000000;
	requestedTime.tv_sec = 0;
	Visualizer* visualizer = ptrs->visualizer;
	std::vector<float_type> configuration;
	std::vector<float_type> configSingleLeg;
	while(1)
	{
		configuration.clear();
		for(int i=0;i<6;++i)
		{
			configSingleLeg.clear();
			board->readPositions(i,configSingleLeg);
			for(int j=0;j<configSingleLeg.size();++j)
			{
				configuration.push_back(configSingleLeg[j]);
			}
		}
		visualizer->drawRobot(robotPose, configuration);
		nanosleep(&requestedTime,NULL);
	}
}

#endif


int main( int argc, const char** argv )
{
    try {

         cout<<"przed stworzeniem"<<endl;
         Board* board;
         board = createBoardDynamixel();
         cout<<"po stworzeniu"<<endl;
         //std::cout << "Board type: " << board->getName() << "\n";

         vector<float_type> complianceSlopeVector = {120,120,120};
         vector<float_type> zera = {0.0,((24.0/180.0)*PI),((-114.0/180.0)*PI)};
         vector<float_type> speeds = {25,25,25};
         for(int i=0;i<6;i++)
         {
             board->setSpeed(i,speeds);
             board->setComplianceSlope(i,complianceSlopeVector);
             board->setPosition(i,zera);
         }

         getchar();

         char filename[100] = "../resources/robotModel.xml";
         cout<<"Przed stworzeniem robota:"<<filename<<endl;
         Robot* robot;
         robot = createRobotMessor("../resources/robotModel.xml");
         cout<<"Po stworzeniu."<<endl;

         Mat34 mat;
         cout<<"Stworzenie macierzy jednostkowej"<<endl;
         mat.setIdentity();
         mat(0, 3) = 0.0;
         mat(1, 3) = 0.0;
         mat(2, 3) = 0.05;
         cout<<"Wypełnienie macierzy"<<endl;

         board->setPosition(robot->movePlatform(mat));
         //std::cout << "Robot name: " << robot->getName() << "\n";

         /*Kinematic* kinematicModel;
         kinematicModel = createKinematicLie("../resources/legModel.xml");
         std::cout << "Kinematic type: " << kinematicModel->getName() << "\n";
*/

         Mat34 destinationMatrix; // where do the values come from?
         std::vector<float_type> destinationConfiguration = robot->movePlatform(mat);
         board->setPosition(destinationConfiguration);

         std::vector<float_type> destinationCompliance = robot->computeCompliance(destinationConfiguration);
         board->setTorqueLimit(destinationCompliance);

#ifndef __DO_NOT_USE_VISUALIZER__

         Visualizer* visualizer;

         visualizationPointers ptrs;

		 try {
             visualizer = createVisualizerIrrlicht("VisualizerWindow", 1920, 1024, 0.01, false);
	         ptrs.board = board;
	         ptrs.visualizer = visualizer;
	         std::thread visualizerThread(updatePlatformPosition,&ptrs);
		 }
		 catch (const std::exception& ex) {
			 std::cerr << ex.what() << std::endl;
			 return 1;
		 }
#endif

    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
