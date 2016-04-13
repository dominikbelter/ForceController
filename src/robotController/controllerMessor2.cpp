/** @file controllerMessor2.cpp
*
* @author Dominik Belter
*/

#include "../include/robotController/controllerMessor2.h"
#include <iostream>
#include <thread>
#include <mutex>

using namespace controller;

std::mutex mtx;

/// A single instance of Controller Messor2
ControllerMessor2::Ptr controllerMessor2;

ControllerMessor2::ControllerMessor2(void) : RobotController("Controller Messor2", TYPE_MESSOR2) {
    robot = createRobotMessor("../resources/robotModel.xml");
    board = createBoardDynamixel();

    if (config.useVisualizer) {
        visualizer = createVisualizerIrrlicht("VisualizerWindow", 1024, 768, 0.01, false);
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

    if (config.useVisualizer) {
        visualizer = createVisualizerIrrlicht("VisualizerWindow", 1024, 768, 0.01, false);
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

void ControllerMessor2::moveLegSingle(unsigned char legNo, const Mat34& trajectory, float_type speed)
{


    std::vector<float_type> configuration;
    std::vector<float_type> diff(3);


    configuration = robot->moveLeg(legNo, trajectory);


    if (config.useVisualizer)
    {
        std::vector<float_type> currentConfiguration = visualizer->getPosition(legNo);

        float_type step = 0.04;
        int n = 1/step;
        //std::cout << n << "    " << step << endl;
        for(int s=0; s<configuration.size(); s++)
        {
            if(s == 0)
            {
                if(configuration[s] > 3.14)
                {
                    configuration[s] -= 6.28;
                }
            }
            diff[s]=(configuration[s] - currentConfiguration[s])*step;
        }

        int i = 0;
        while(i<=n)
        {

            //cout << visualizer->getPosition(legNo)[0] << endl;
            for(int s=0; s<configuration.size(); s++)
            {
                configuration[s] = currentConfiguration[s] + diff[s]*i;
            }
            mtx.lock();
            visualizer->setPosition(legNo,configuration);
            mtx.unlock();
            i++;
            usleep(50000);


        }
    }
    else
    {
        vector<float_type> readAngle(3);
        vector<float_type> speedScale(3);
        float_type longestJourney = 0;

        for(int i=0; i<configuration.size(); i++)
        {
            //mtx.lock();
            board->readPosition(legNo, i, readAngle[i]);
            //mtx.unlock();
            diff[i] = abs(readAngle[i] - configuration[i]);
            if(diff[i] > longestJourney)
            {
                longestJourney = diff[i];
            }
        }

        for(int i=0; i<configuration.size(); i++)
        {
            speedScale[i] = diff[i] / longestJourney;
            //mtx.lock();
            board->setSpeed(legNo, i, speed);
            //mtx.unlock();

        }


        bool motionFinished = false;
        float_type offset = 0.10;
        //mtx.lock();
        board->setPosition(legNo, configuration);
        //mtx.unlock();

        while(!motionFinished)
        {
            //mtx.lock();
            board->readPosition(legNo, 0, readAngle[0]);
            board->readPosition(legNo, 1, readAngle[1]);
            board->readPosition(legNo, 2, readAngle[2]);
            //mtx.unlock();


            cout << "s0 " << readAngle[0] << " s1 " << readAngle[1] << " s2 " << readAngle[2] << endl;

            if((abs(readAngle[0] - configuration[0]) < offset) && (abs(readAngle[1] - configuration[1]) < offset) && (abs(readAngle[2] - configuration[2]) < offset) )
            {
                motionFinished = true;
                cout << "move finished";
            }
        }

    }

}

void ControllerMessor2::moveLeg(unsigned char legNo, const std::vector<Mat34>& trajectory, float_type speed)
{

    for(int i=0; i<trajectory.size(); i++)
    {
        this->moveLegSingle(legNo, trajectory[i], speed);
    }

}

void ControllerMessor2::moveLegs(std::vector<unsigned char> legNo, const std::vector<std::vector<Mat34> >& trajectory, float_type speed)
{

    if(legNo.size() == 1)
    {
        std::thread first(&ControllerMessor2::moveLeg,this,legNo[0],trajectory[0], speed);
        first.join();
    }

    else if(legNo.size() == 2)
    {
        std::thread first(&ControllerMessor2::moveLeg,this,legNo[0],trajectory[0], speed);
        std::thread second(&ControllerMessor2::moveLeg,this,legNo[1],trajectory[1], speed);
        first.join();
        second.join();
    }

    else if(legNo.size() == 3)
    {
        std::thread first(&ControllerMessor2::moveLeg,this,legNo[0],trajectory[0], speed);
        std::thread second(&ControllerMessor2::moveLeg,this,legNo[1],trajectory[1], speed);
        std::thread third(&ControllerMessor2::moveLeg,this,legNo[2],trajectory[2], speed);
        first.join();
        second.join();
        third.join();
    }

    else if(legNo.size() == 4)
    {
        std::thread first(&ControllerMessor2::moveLeg,this,legNo[0],trajectory[0], speed);
        std::thread second(&ControllerMessor2::moveLeg,this,legNo[1],trajectory[1], speed);
        std::thread third(&ControllerMessor2::moveLeg,this,legNo[2],trajectory[2], speed);
        std::thread fourth(&ControllerMessor2::moveLeg,this,legNo[3],trajectory[3], speed);
        first.join();
        second.join();
        third.join();
        fourth.join();
    }

    else if(legNo.size() == 5)
    {
        std::thread first(&ControllerMessor2::moveLeg,this,legNo[0],trajectory[0], speed);
        std::thread second(&ControllerMessor2::moveLeg,this,legNo[1],trajectory[1], speed);
        std::thread third(&ControllerMessor2::moveLeg,this,legNo[2],trajectory[2], speed);
        std::thread fourth(&ControllerMessor2::moveLeg,this,legNo[3],trajectory[3], speed);
        std::thread fifth(&ControllerMessor2::moveLeg,this,legNo[4],trajectory[4], speed);
        first.join();
        second.join();
        third.join();
        fourth.join();
        fifth.join();
    }

    else if(legNo.size() == 6)
    {
        std::thread first(&ControllerMessor2::moveLeg,this,legNo[0],trajectory[0], speed);
        std::thread second(&ControllerMessor2::moveLeg,this,legNo[1],trajectory[1], speed);
        std::thread third(&ControllerMessor2::moveLeg,this,legNo[2],trajectory[2], speed);
        std::thread fourth(&ControllerMessor2::moveLeg,this,legNo[3],trajectory[3], speed);
        std::thread fifth(&ControllerMessor2::moveLeg,this,legNo[4],trajectory[4], speed);
        std::thread sixth(&ControllerMessor2::moveLeg,this,legNo[5],trajectory[5], speed);
        first.join();
        second.join();
        third.join();
        fourth.join();
        fifth.join();
        sixth.join();
    }



//    std::vector<std::thread> legsThreads;

//    for(int i=0; i<legNo.size(); i++){
//        legsThreads.emplace_back(&ControllerMessor2::moveLeg,this,legNo[i],trajectory[i], speed);
//    }

//    // later
//    for (int i=0; i<legNo.size(); i++) {
//        legsThreads.at(i).join();
//    }

//    // OK to destroy now
//    legsThreads.clear();
}
