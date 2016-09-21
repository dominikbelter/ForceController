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
        config.FirstChildElement( "parameters" )->QueryDoubleAttribute("offsetPajak", &offsetPajak);
        config.FirstChildElement( "parameters" )->QueryIntAttribute("speedPajak", &speedPajak);


        if (verbose){
            std::cout << "verbose: " << verbose << "\n";
            std::cout << "useVisualizer: " << useVisualizer << "\n";
            std::cout << "offsetPajak: " << offsetPajak << "\n";

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
    std::vector<unsigned char> legsNo;
    std::vector<float_type> conf;
    std::vector<std::vector<float_type> > configs;
    std::vector<std::vector<std::vector<float_type> > > confTotal;
    for (int i=0; i<6; i++)
    {
        legsNo.push_back(i);
        for (int j=0; j<3; j++)
        {
            conf.push_back(configuration[i*3+j]);
        }

        configs.push_back(conf);
        confTotal.push_back(configs);
        conf.clear();
        configs.clear();
    }

    moveLegs(legsNo, confTotal, speed, 0);

}

void ControllerMessor2::movePlatform(std::vector<Mat34>& motion, double speed)
{
    for(int i=0; i<motion.size(); i++)
    {
        this->movePlatform(motion[i], speed);
    }
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

void ControllerMessor2::moveLegSingle(unsigned char legNo, const Mat34& trajectory, float_type speed, bool lastMove, int smartMotionMode, int inputCoordinateSystem )
{
    std::vector<float_type> configuration;
    std::vector<float_type> diff(3);
    std::vector<float_type> newConf(3);

    if(inputCoordinateSystem == 1)
    {
        Mat34 trajectoryFromRobotToLeg;
        trajectoryFromRobotToLeg = robot->robotToFootTransformation(trajectory, legNo);
        configuration = robot->moveLeg(legNo, trajectoryFromRobotToLeg);
    }
    else
        configuration = robot->moveLeg(legNo, trajectory);

    for(int s=0; s<configuration.size(); s++)
    {
        if(s == 0)
        {
            if(configuration[s] > 3.14)
            {
                configuration[s] -= 6.28;
            }
            else if(configuration[s] < -3.14)
            {
                configuration[s] += 6.28;
            }
        }
    }

    if (config.useVisualizer)
    {
        bool wychodze = false;

        std::vector<float_type> currentConfiguration = visualizer->getPosition(legNo);
        mtx.lock();
        mtx.unlock();

        float_type step = 0.04;
        int n = 1/step;
        for(int s=0; s<configuration.size(); s++)
        {
            diff[s]=(configuration[s] - currentConfiguration[s])*step;
        }

        int i = 0;
        while(i<=n)
        {
            std::vector<float_type> currentConfigurationNow = visualizer->getPosition(legNo);

            for(int s=0; s<configuration.size(); s++)
            {

               if(abs(configuration[s] - currentConfigurationNow[s]) < 0.0001)
                {
                    wychodze = true;
                }
                newConf[s] = currentConfiguration[s] + diff[s]*i;
            }

            visualizer->setPosition(legNo,newConf);

            i++;
            usleep(50000);


        }
    }
    else
    {
        vector<Mat34> result;
        vector<Mat34> current;
        vector<float_type> readAngle(3);
        vector<float_type> speedScale(3);
        float_type longestJourney = 0;

        for(int i=0; i<configuration.size(); i++)
        {

            board->readPosition(legNo, i, readAngle[i]);
            diff[i] = abs(readAngle[i] - configuration[i]);
            if(diff[i] > longestJourney)
            {
                longestJourney = diff[i];
            }
        }

        for(int i=0; i<configuration.size(); i++)
        {
            speedScale[i] = diff[i] / longestJourney;
            board->setSpeed(legNo, i, speed*speedScale[i]);

        }

        bool motionFinished = false;
//        float_type offsetConf = 0.20;
        float_type offset;
        if(!lastMove)
            offset = 0.020;
        else
            offset = 0.018;
        board->setPosition(legNo, configuration);

        bool isContactDetected = false;
        bool startReadingContact = false;
        bool doOnce = false;
        while(!motionFinished)
        {
            //usleep(200000);
            isContactDetected = board->readContact(legNo);
            if(!isContactDetected && !startReadingContact)
                startReadingContact = true;

            board->readPosition(legNo, 0, readAngle[0]);
            board->readPosition(legNo, 1, readAngle[1]);
            board->readPosition(legNo, 2, readAngle[2]);

            current = robot->conputeLinksPosition(readAngle);
            result = robot->conputeLinksPosition(configuration);

            //smartMotionMode = 1 for reading contact state
            if(smartMotionMode == 1)
            {
                if(isContactDetected || (abs(result[3](0,3)-current[3](0,3)) < offset && abs(result[3](1,3)-current[3](1,3)) < offset && abs(result[3](2,3)-current[3](2,3)) < offset))
                //if(isContactDetected || (abs(readAngle[0] - configuration[0]) < offsetConf) && (abs(readAngle[1] - configuration[1]) < offsetConf) && (abs(readAngle[2] - configuration[2]) < offsetConf))
                {
                    if(startReadingContact)
                    {
                        motionFinished=true;
                        if(isContactDetected && !doOnce)
                        {
                            newConf[0] = readAngle[0];
                            newConf[1] = readAngle[1]-config.offsetPajak;
                            newConf[2] = readAngle[2];
                            board->setPosition(legNo, newConf);

                            doOnce = true;
                        }
                    }                  
                }

            }
            else if(smartMotionMode == 5) //mdoe for tripod step back movement
            {
                vector<float_type> newConf(3);
                newConf[0] = configuration[0];
                newConf[1] = readAngle[1];
                newConf[2] = readAngle[2];
                motionFinished=true;
                if(isContactDetected && !doOnce)
                {
                    board->setPosition(legNo, newConf);
                    doOnce = true;
                }
            }
            //stardard smartMotionMode for reading servo position only
            else
            {
                if(abs(result[3](0,3)-current[3](0,3)) < offset && abs(result[3](1,3)-current[3](1,3)) < offset && abs(result[3](2,3)-current[3](2,3)) < offset)
                //if((abs(readAngle[0] - configuration[0]) < offsetConf) && (abs(readAngle[1] - configuration[1]) < offsetConf) && (abs(readAngle[2] - configuration[2]) < offsetConf))
                {
                    motionFinished=true;
                }
            }

        }

    }

}

void ControllerMessor2::moveLegSingle(unsigned char legNo, const std::vector<float_type>& configuration1, float_type speed, bool lastMove, int smartMotionMode)
{

    std::vector<float_type> configuration(3);
    std::vector<float_type> diff(3);
    mtx.lock();
    configuration[0] = configuration1[0];
    configuration[1] = configuration1[1];
    configuration[2] = configuration1[2];
    mtx.unlock();
    for(int s=0; s<configuration.size(); s++)
    {
        if(s == 0)
        {
            if(configuration[s] > 3.14)
            {
                configuration[s] -= 6.28;
            }
            else if(configuration[s] < -3.14)
            {
                configuration[s] += 6.28;
            }
        }
    }
    mtx.lock();
    mtx.unlock();
    if (config.useVisualizer)
    {
        std::vector<float_type> currentConfiguration = visualizer->getPosition(legNo);

        float_type step = 0.04;
        int n = 1/step;
        for(int s=0; s<configuration.size(); s++)
        {
            diff[s]=(configuration[s] - currentConfiguration[s])*step;
        }

        int i = 0;
        while(i<=n)
        {
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
        vector<float_type> speedScale(3);
        vector<Mat34> result;
        vector<Mat34> current;
        float_type longestJourney = 0;

        for(int i=0; i<configuration.size(); i++)
        {

            board->readPosition(legNo, i, readAngle[i]);
            diff[i] = abs(readAngle[i] - configuration[i]);
            if(diff[i] > longestJourney)
            {
                longestJourney = diff[i];
            }
        }

        for(int i=0; i<configuration.size(); i++)
        {
            speedScale[i] = diff[i] / longestJourney;
            board->setSpeed(legNo, i, speed*speedScale[i]);

        }


        bool motionFinished = false;
        float_type offset = 0.20;

        board->setPosition(legNo, configuration);

        while(!motionFinished)
        {
            usleep(90000);
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


void ControllerMessor2::moveLeg(unsigned char legNo, const std::vector<Mat34>& trajectory, float_type speed, int smartMotionMode, int inputCoordinateSystem)
{
    for(int i=0; i<trajectory.size(); i++)
    {
        if(smartMotionMode == 1)
        {
            if(trajectory.size() == 1)
            {
                this->moveLegSingle(legNo, trajectory[i], speed, true, 5, inputCoordinateSystem);
            }
            else
            {
                if(i==0)
                    this->moveLegSingle(legNo, trajectory[i], speed, false, 0, inputCoordinateSystem);
                else if(i == trajectory.size()-1)
                    this->moveLegSingle(legNo, trajectory[i], speed, true, 1, inputCoordinateSystem);
                else
                    this->moveLegSingle(legNo, trajectory[i], speed, false, 1, inputCoordinateSystem);
            }
        }
        else
        {
            if(i==trajectory.size()-1)
                this->moveLegSingle(legNo, trajectory[i], speed, true, smartMotionMode, inputCoordinateSystem);
            else
                this->moveLegSingle(legNo, trajectory[i], speed, false, smartMotionMode, inputCoordinateSystem);
        }

    }
}

void ControllerMessor2::moveLegConf(unsigned char legNo,const std::vector<std::vector<float_type> >& configuration, float_type speed, int smartMotionMode)
{   
    for(int i=0; i<configuration.size(); i++)
    {
        if(smartMotionMode == 1)
        {
            if(configuration.size() == 1)
            {
                this->moveLegSingle(legNo, configuration[i], speed, true, 0);
            }
            else
            {
                if(i==0)
                    this->moveLegSingle(legNo, configuration[i], speed, false, 0);
                else if(i == configuration.size()-1)
                    this->moveLegSingle(legNo, configuration[i], speed, true, 1);
                else
                    this->moveLegSingle(legNo, configuration[i], speed, false, 1);
            }
        }
        else
        {
            if(i==configuration.size()-1)
                this->moveLegSingle(legNo, configuration[i], speed, true, smartMotionMode);
            else
                this->moveLegSingle(legNo, configuration[i], speed, false, smartMotionMode);
        }
    }
}

void ControllerMessor2::moveLegs(std::vector<unsigned char> legNo, const std::vector<std::vector<Mat34> >& trajectory, float_type speed, int smartMotionMode, int inputCoordinateSystem)
{

    std::vector<float_type> speedScale;
    speedScale = scaleSpeedBytrajectory(legNo, trajectory);

    if(legNo.size() == 1)
    {
        std::thread first(&ControllerMessor2::moveLeg,this,legNo[0],trajectory[0], speed*speedScale[0], smartMotionMode, inputCoordinateSystem);
        first.join();
    }

    else if(legNo.size() == 2)
    {
        std::thread first(&ControllerMessor2::moveLeg,this,legNo[0],trajectory[0], speed*speedScale[0], smartMotionMode, inputCoordinateSystem);
        std::thread second(&ControllerMessor2::moveLeg,this,legNo[1],trajectory[1], speed*speedScale[1], smartMotionMode, inputCoordinateSystem);
        first.join();
        second.join();
    }

    else if(legNo.size() == 3)
    {
        std::thread first(&ControllerMessor2::moveLeg,this,legNo[0],trajectory[0], speed*speedScale[0], smartMotionMode, inputCoordinateSystem);
        std::thread second(&ControllerMessor2::moveLeg,this,legNo[1],trajectory[1], speed*speedScale[1], smartMotionMode, inputCoordinateSystem);
        std::thread third(&ControllerMessor2::moveLeg,this,legNo[2],trajectory[2], speed*speedScale[2], smartMotionMode, inputCoordinateSystem);
        first.join();
        second.join();
        third.join();
    }

    else if(legNo.size() == 4)
    {
        std::thread first(&ControllerMessor2::moveLeg,this,legNo[0],trajectory[0], speed*speedScale[0], smartMotionMode, inputCoordinateSystem);
        std::thread second(&ControllerMessor2::moveLeg,this,legNo[1],trajectory[1], speed*speedScale[1], smartMotionMode, inputCoordinateSystem);
        std::thread third(&ControllerMessor2::moveLeg,this,legNo[2],trajectory[2], speed*speedScale[2], smartMotionMode, inputCoordinateSystem);
        std::thread fourth(&ControllerMessor2::moveLeg,this,legNo[3],trajectory[3], speed*speedScale[3], smartMotionMode, inputCoordinateSystem);
        first.join();
        second.join();
        third.join();
        fourth.join();
    }

    else if(legNo.size() == 5)
    {
        std::thread first(&ControllerMessor2::moveLeg,this,legNo[0],trajectory[0], speed*speedScale[0], smartMotionMode, inputCoordinateSystem);
        std::thread second(&ControllerMessor2::moveLeg,this,legNo[1],trajectory[1], speed*speedScale[1], smartMotionMode, inputCoordinateSystem);
        std::thread third(&ControllerMessor2::moveLeg,this,legNo[2],trajectory[2], speed*speedScale[2], smartMotionMode, inputCoordinateSystem);
        std::thread fourth(&ControllerMessor2::moveLeg,this,legNo[3],trajectory[3], speed*speedScale[3], smartMotionMode, inputCoordinateSystem);
        std::thread fifth(&ControllerMessor2::moveLeg,this,legNo[4],trajectory[4], speed*speedScale[4], smartMotionMode, inputCoordinateSystem);
        first.join();
        second.join();
        third.join();
        fourth.join();
        fifth.join();
    }

    else if(legNo.size() == 6)
    {
        std::thread first(&ControllerMessor2::moveLeg,this,legNo[0],trajectory[0], speed*speedScale[0], smartMotionMode, inputCoordinateSystem);
        std::thread second(&ControllerMessor2::moveLeg,this,legNo[1],trajectory[1], speed*speedScale[1], smartMotionMode, inputCoordinateSystem);
        std::thread third(&ControllerMessor2::moveLeg,this,legNo[2],trajectory[2], speed*speedScale[2], smartMotionMode, inputCoordinateSystem);
        std::thread fourth(&ControllerMessor2::moveLeg,this,legNo[3],trajectory[3], speed*speedScale[3], smartMotionMode, inputCoordinateSystem);
        std::thread fifth(&ControllerMessor2::moveLeg,this,legNo[4],trajectory[4], speed*speedScale[4], smartMotionMode, inputCoordinateSystem);
        std::thread sixth(&ControllerMessor2::moveLeg,this,legNo[5],trajectory[5], speed*speedScale[5], smartMotionMode, inputCoordinateSystem);
        first.join();
        second.join();
        third.join();
        fourth.join();
        fifth.join();
        sixth.join();
    }
}

void ControllerMessor2::moveLegs(std::vector<unsigned char> legNo,const std::vector<std::vector<std::vector<float_type> > >& configuration, float_type speed, int smartMotionMode)
{   
    if(legNo.size() == 1)
    {
        std::thread first(&ControllerMessor2::moveLegConf,this,legNo[0],configuration[0], speed, smartMotionMode);
        first.join();
    }

    else if(legNo.size() == 2)
    {
        std::thread first(&ControllerMessor2::moveLegConf,this,legNo[0],configuration[0], speed, smartMotionMode);
        std::thread second(&ControllerMessor2::moveLegConf,this,legNo[1],configuration[1], speed, smartMotionMode);
        first.join();
        second.join();
    }

    else if(legNo.size() == 3)
    {
        std::thread first(&ControllerMessor2::moveLegConf,this,legNo[0],configuration[0], speed, smartMotionMode);
        std::thread second(&ControllerMessor2::moveLegConf,this,legNo[1],configuration[1], speed, smartMotionMode);
        std::thread third(&ControllerMessor2::moveLegConf,this,legNo[2],configuration[2], speed, smartMotionMode);
        first.join();
        second.join();
        third.join();
    }

    else if(legNo.size() == 4)
    {
        std::thread first(&ControllerMessor2::moveLegConf,this,legNo[0],configuration[0], speed, smartMotionMode);
        std::thread second(&ControllerMessor2::moveLegConf,this,legNo[1],configuration[1], speed, smartMotionMode);
        std::thread third(&ControllerMessor2::moveLegConf,this,legNo[2],configuration[2], speed, smartMotionMode);
        std::thread fourth(&ControllerMessor2::moveLegConf,this,legNo[3],configuration[3], speed, smartMotionMode);
        first.join();
        second.join();
        third.join();
        fourth.join();
    }

    else if(legNo.size() == 5)
    {
        std::thread first(&ControllerMessor2::moveLegConf,this,legNo[0],configuration[0], speed, smartMotionMode);
        std::thread second(&ControllerMessor2::moveLegConf,this,legNo[1],configuration[1], speed, smartMotionMode);
        std::thread third(&ControllerMessor2::moveLegConf,this,legNo[2],configuration[2], speed, smartMotionMode);
        std::thread fourth(&ControllerMessor2::moveLegConf,this,legNo[3],configuration[3], speed, smartMotionMode);
        std::thread fifth(&ControllerMessor2::moveLegConf,this,legNo[4],configuration[4], speed, smartMotionMode);
        first.join();
        second.join();
        third.join();
        fourth.join();
        fifth.join();
    }

    else if(legNo.size() == 6)
    {

        std::thread first(&ControllerMessor2::moveLegConf,this,legNo[0],configuration[0], speed, smartMotionMode);
        std::thread second(&ControllerMessor2::moveLegConf,this,legNo[1],configuration[1], speed, smartMotionMode);
        std::thread third(&ControllerMessor2::moveLegConf,this,legNo[2],configuration[2], speed, smartMotionMode);
        std::thread fourth(&ControllerMessor2::moveLegConf,this,legNo[3],configuration[3], speed, smartMotionMode);
        std::thread fifth(&ControllerMessor2::moveLegConf,this,legNo[4],configuration[4], speed, smartMotionMode);
        std::thread sixth(&ControllerMessor2::moveLegConf,this,legNo[5],configuration[5], speed, smartMotionMode);
        first.join();
        second.join();
        third.join();
        fourth.join();
        fifth.join();
        sixth.join();
    }
}

std::vector<float_type> ControllerMessor2::scaleSpeedBytrajectory (std::vector<unsigned char> legNo, const std::vector<std::vector<Mat34> >& trajectory)
{
    std::vector<float_type> longest(trajectory.size());
    float_type longestJourney;
    float_type longestestJourney;
    std::vector<float_type> configuration;
    std::vector<float_type> diff(3);
    std::vector<float_type> currentAngle(3);
    std::vector<float_type> speedScale(trajectory.size());

    for(int i=0; i < trajectory.size(); i++)
    {
        longest[i]=0;
        longestJourney = 0;
        longestestJourney = 0;
        for(int j=0; j<trajectory[i].size(); j++)
        {
//            cout << "RUCH " << j << endl;
            configuration = robot->moveLeg(legNo[i], trajectory[i][j]);

            for(int k=0; k<configuration.size(); k++)
            {
                if(configuration[k] > 3.14)
                {
                    configuration[k]-=6.28;
                }
                if(configuration[k] < -3.14)
                {
                    configuration[k]+=6.28;
                }

                if(j==0)
                    board->readPosition(legNo[i], k, currentAngle[k]);

                //cout << "CZYTAM NOGE: " << (int)legNo[i] << " SERWO: " << k << "WYNIK: " << configuration[k] << endl;
                diff[k] = abs(currentAngle[k] - configuration[k]);
                //cout << "CZYTAM NOGE: " << (int)legNo[i] << " SERWO: " << k << "WYNIK DIFF: " << diff[k] << endl;
                currentAngle[k] = configuration[k];


                if(diff[k] > longestJourney)
                {
                    longestJourney = diff[k];
                }
            }
            longest[i] += longestJourney;
        }
        //cout << "LONGEST NOGA: " << i << " = " << longest[i] << endl;

    }

    for(int i=0; i<trajectory.size(); i++)
    {
        if(longest[i] > longestestJourney)
        {
            longestestJourney = longest[i];
        }
    }

    for(int i=0; i<trajectory.size(); i++)
    {
        speedScale[i] = longest[i] / longestestJourney;
        //cout << " SPEED " << i << " WYNOSI: " << speedScale[i] << endl;
    }

    return speedScale;
}

