/**
* @author Maciej Okoniewski
* @author Emil Waledziak
* @author Jerzy Wiatrow
* @author Marcin Zgolinski
* 
*/

#ifndef _ROBOTMESSOR_H_INCLUDED
#define _ROBOTMESSOR_H_INCLUDED

#include "robot.h"
#include <iostream>
#include "../include/legModel/insectLeg.h"


namespace controller {
                    Robot* createRobotMessor(void);
                    Robot* createRobotMessor(std::string configFilename);
                     }

using namespace controller;

///Class RobotMessor includes all functions required for creating robot model

class RobotMessor: public Robot
{
public:
    /// Pointer
    typedef std::unique_ptr<Robot> Ptr;
    RobotMessor(void);
    RobotMessor(std::string configFilename):Robot(configFilename, "RobotMessor", TYPE_MESSOR2)
    {
        tinyxml2::XMLDocument config;
        std::string filename = "../../resources/" + configFilename;
        config.LoadFile(filename.c_str());
        if (config.ErrorID())
        {
            std::cout << "unable to load Robot config file.\n";
        }
        else
        {
            legModel = createInsectLeg("../resources/legModel.xml");

            std::cout << "Load robot parameters\n";
            tinyxml2::XMLElement * parameters;
            parameters = config.FirstChildElement("parameters");
            parameters->QueryIntAttribute("legsNo", &legsNo);
            std::cout << "Load leg parameters\n";

            for (int i=0;i<legsNo;i++){
                std::string legName = "leg" + std::to_string(i+1);
                tinyxml2::XMLElement * legParams = config.FirstChildElement(legName.c_str());
                Mat34 position(Mat34::Identity());
                legParams = legParams->FirstChildElement("position");
                for (int row=1;row<4;row++){
                    for (int col=1;col<5;col++){
                        std::string elementName = "a" + std::to_string(row) + std::to_string(col);
                        double element;
                        legParams->QueryDoubleAttribute(elementName.c_str(), &element);
                        position(row-1,col-1) = element;
                    }
                }
                legMountPoints.push_back(position);
            }
        }

        neutralMotion.setIdentity();
        neutralMotion(2, 3) = 0.12;

       for (int i = 0; i<6; i++){
                configurationStart.push_back(0);
                configurationStart.push_back(24*3.14/180);
                configurationStart.push_back(-114*3.14/180);
        }
        configurationCurr=configurationStart;
    };


    /// Name of the robot model
    const std::string& getName() const { return name; }

    ///Compute configuration of the robot for the reference motion
    /**
    * @param motion - standard robotic matrix 3x4, which includes robot Rotation and Translation
    * @return tmp
    */
    std::vector<float_type> movePlatform(const Mat34& motion);

    ///Compute configuration of the leg for the reference motion of the platform
    /**
    * @param motion - specified motion
    * @return reference values for servomotors
    */
    std::vector<float_type> computeLegConfiguration(int legNo, const Mat34 bodyMotion, std::vector<float_type> startConfiguration);

    ///Compute configuration of the robot for the reference motion (in relation to neutral pose)
    /**
    * @param motion - standard robotic matrix 3x4, which includes robot Rotation and Translation to neutral pose
    * @return tmp
    */

    ///Compute configuration of the robot for the reference motion (each foot generates separate motion)
    std::vector<float_type> movePlatform(const std::vector<Mat34>& motion);

    std::vector<float_type> moveLeg(unsigned char legNo, const Mat34& trajectory);


    std::vector<float_type> movePlatformNeutral(const Mat34 motion);

    /// new method: computes forward kinematics for each leg and returns position of each link of the robot (body is [0,0,0]^T)
	/**
	* @param configuration - configuration off all actual positions of servomechanisms in robot legs
	* @return name - return position of all servomechanisms in legs and end of the legs
	*/
    std::vector<Mat34> conputeLinksPosition(std::vector<float_type> configuration);


    /** Compute force in each joint of the legs, input configuration of the robot
    * @param [in] configuration of all servomechanism
    * @return std::vector<float_type> compliance of all servomechanism
    */
    std::vector<float_type> computeCompliance(const std::vector<float_type> configuration);

    ~RobotMessor(void);

private:

    Mat34 neutralMotion; /// neutral position of the robot

    Leg* legModel;

    std::vector<float_type> configurationStart;
    std::vector<float_type> configurationCurr;
};

#endif


