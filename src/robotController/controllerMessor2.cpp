/** @file controllerMessor2.cpp
*
* @author Dominik Belter
*/

#include "../include/robotController/controllerMessor2.h"
#include <iostream>

using namespace controller;

/// A single instance of Controller Messor2
ControllerMessor2::Ptr controllerMessor2;

ControllerMessor2::ControllerMessor2(void) : RobotController("Controller Messor2", TYPE_MESSOR2) {
    robot = createRobotMessor("../resources/robotModel.xml");
    board = createBoardDynamixel();
    visualizer = createVisualizerIrrlicht("VisualizerWindow", 1024, 768, 0.01, false);
    if (config.useVisualizer) {
        continueVisualizer=true;
        visualizerThr.reset(new std::thread(&ControllerMessor2::drawRobot, this));
    }
}

ControllerMessor2::Config::Config(std::string configFilename){
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID()) {
        std::cout << "unable to load Kinematic config file.\n";
    }
    else {
        //tinyxml2::XMLElement * parameters;
        config.FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
        config.FirstChildElement( "parameters" )->QueryBoolAttribute("useVisualizer", &useVisualizer);
        if (verbose){
            std::cout << "verbose: " << verbose << "\n";
            std::cout << "useVisualizer: " << useVisualizer << "\n";
        }
    }
}

ControllerMessor2::ControllerMessor2(std::string configFilename) : config(configFilename), RobotController("Controller Messor2", TYPE_MESSOR2){
    robot = createRobotMessor("../resources/robotModel.xml");
    board = createBoardDynamixel();
    visualizer = createVisualizerIrrlicht("VisualizerWindow", 1024, 768, 0.01, false);
    if (config.useVisualizer) {
        continueVisualizer=true;
        visualizerThr.reset(new std::thread(&ControllerMessor2::drawRobot, this));
    }
}

void ControllerMessor2::drawRobot(){
    std::vector<double> configuration(18,0);
    for (int i=0;i<6;i++){
        configuration.push_back(0);
        configuration.push_back(24*3.14/180);
        configuration.push_back(-114*3.14/180);
    }
    visualizer->drawRobot(Mat34::Identity(), configuration);
}

/// Wait for visualization thread to finish
void ControllerMessor2::finishVisualizer(void){
    continueVisualizer = false;
    visualizerThr->join();
}

///Move platform
void ControllerMessor2::movePlatform(Mat34& motion, double speed){
    std::vector<double> configuration = robot->movePlatform(motion);
    if (config.useVisualizer)
        visualizer->setPosition(configuration);
    else
        board->setPosition(configuration);
}

ControllerMessor2::~ControllerMessor2(void) {
}

/// Compliant tripod step
void ControllerMessor2::tripodStepCompliant(Mat34& motion, double speed){

}

/// create single ControllerMessor2 object
controller::RobotController* controller::createControllerMessor2(void)
{
    controllerMessor2.reset(new ControllerMessor2());
    return controllerMessor2.get();
}

/// create single ControllerMessor2 object
controller::RobotController* controller::createControllerMessor2(std::string filename) {
    controllerMessor2.reset(new ControllerMessor2(filename));
    return controllerMessor2.get();
}

void ControllerMessor2::moveLeg(unsigned char legNo, const Mat34& trajectory)
{


    std::vector<float_type> configuration;


    configuration = robot->moveLeg(legNo, trajectory);


    if (config.useVisualizer)
    {
        std::vector<float_type> currentConfiguration = visualizer->getPosition(legNo);

        float_type step = 0.05;
        int n = 1/step;
        //std::cout << n << "    " << step << endl;
        std::vector<float_type> diff;
        for(int s=0; s<configuration.size(); s++)
        {
            if(s == 0)
            {
                if(configuration[s] > 3.14)
                {
                    configuration[s] -= 6.28;
                }
            }
            diff.push_back(configuration[s] - currentConfiguration[s]);
            diff[s] = diff[s]*step;
        }

        int i = 0;
        while(i<=n)
        {
            //cout << visualizer->getPosition(legNo)[0] << endl;
            for(int s=0; s<configuration.size(); s++)
            {
                configuration[s] = currentConfiguration[s] + diff[s]*i;
            }
            visualizer->setPosition(legNo,configuration);
            i++;
            usleep(50000);
        }
    }
    else
    {
        vector<float_type> readAngle(3);
        bool motionFinished = false;
        float_type offset = 0.10;
        board->setPosition(legNo, configuration);
        while(!motionFinished)
        {
            board->readPosition(legNo, 0, readAngle[0]);
            board->readPosition(legNo, 1, readAngle[1]);
            board->readPosition(legNo, 2, readAngle[2]);

            if((abs(readAngle[0] - configuration[0]) < offset) && (abs(readAngle[1] - configuration[1]) < offset) && (abs(readAngle[2] - configuration[2]) < offset) )
            {
                motionFinished = true;
            }
        }

    }

}

void ControllerMessor2::moveLeg(unsigned char legNo, const std::vector<Mat34>& trajectory)
{

    for(int i=0; i<trajectory.size(); i++)
    {
        this->moveLeg(legNo, trajectory[i]);
    }

}
