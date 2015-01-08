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
            Leg0 = createInsectLeg("../resources/legModel.xml");

            tinyxml2::XMLElement * parameters;
            float_type paramf;
            parameters = config.FirstChildElement("width1");
            parameters = parameters->FirstChildElement( "parameters" );
            parameters->QueryDoubleAttribute("width_max", &paramf); width_max = paramf;
              parameters = config.FirstChildElement("width2");
            parameters = parameters->FirstChildElement( "parameters" );
            parameters->QueryDoubleAttribute("width_min", &paramf); width_min = paramf;
              parameters = config.FirstChildElement("length1");
            parameters = parameters->FirstChildElement( "parameters" );
            parameters->QueryDoubleAttribute("length", &paramf); length = paramf;

           // std::cout << "links no: " << linksNo << " joints no: " << jointsNo << "\n";
            std::cout<< width_max << std::endl;
            //std::cout << "Lenght2: " << lengths[1] << std::endl;
            //std::cout << "Lenght3: " << lengths[2] << std::endl;
        }
        L0.setIdentity();
        L0(0, 3) = width_min;
        L0(1, 3) = length;
        L0(2, 3) = 0;

        L1.setIdentity();
        L1(0, 3) = width_max;
        L1(1, 3) = 0;
        L1(2, 3) = 0;

        L2.setIdentity();
        L2(0, 3) = width_min;
        L2(1, 3) = -length;
        L2(2, 3) = 0;

        L3.setIdentity();
        L3(0, 3) = -width_min;
        L3(1, 3) = -length;
        L3(2, 3) = 0;

        L4.setIdentity();
        L4(0, 3) = -width_max;
        L4(1, 3) = 0;
        L4(2, 3) = 0;

        L5.setIdentity();
        L5(0, 3) = -width_min;
        L5(1, 3) = length;
        L5(2, 3) = 0;

        L_all.push_back(L0);
        L_all.push_back(L1);
        L_all.push_back(L2);
        L_all.push_back(L3);
        L_all.push_back(L4);
        L_all.push_back(L5);

        OldMotion.setIdentity();
        OldMotion(0, 3) = 0;
        OldMotion(1, 3) = 0;
        OldMotion(2, 3) = 0;

        NeutralMotion.setIdentity();
        NeutralMotion(0, 3) = 0;
        NeutralMotion(1, 3) = 0;
        NeutralMotion(2, 3) = 0.12;


    };





    /// Name of the robot model
    const std::string& getName() const { return name; }

    ///Compute configuration of the robot for the reference motion
    /**
    * @param motion - standard robotic matrix 3x4, which includes robot Rotation and Translation
    * @return tmp
    */
    std::vector<float_type> movePlatform(const Mat34& motion);

    ///Compute configuration of the robot for the reference motion (in relation to neutral pose)
    /**
    * @param motion - standard robotic matrix 3x4, which includes robot Rotation and Translation to neutral pose
    * @return tmp
    */
    std::vector<float_type> movePlatformNeutral(const Mat34 motion);

    /// new method: computes forward kinematics for each leg and returns position of each link of the robot (body is [0,0,0]^T)
	/**
	* @param configuration - configuration off all actual positions of servomechanisms in robot legs
	* @return name - return position of all servomechanisms in legs and end of the legs
	*/
    std::vector<Mat34> conputeLinksPosition(std::vector<float_type> configuration);

    ///Compute force in each joint of the legs, input configuration of the robot
    /**
    * @param vector<float_type>
    * @return TF.susceptibility - return susceptibility in each servomotor
    */
    std::vector<float_type> computeCompliance(const std::vector<float_type> configuration);

    ~RobotMessor(void);

private:

	float_type width_max; ///distance from center to middle leg
	float_type width_min; /// distance from x to front leg
	float_type length; ///distance front legs from x


	Mat34 L0;
	Mat34 L1;
	Mat34 L2;
	Mat34 L3;
	Mat34 L4;
	Mat34 L5;

	Mat34 OldMotion; ///matrix include oldmotion robot, start with...
    Mat34 NeutralMotion; ///matrix include robots neutral position

	std::vector<Mat34> L_all; ///vector include translations for all legs from robot center
	Leg* Leg0;


};

#endif


