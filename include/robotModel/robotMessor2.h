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
    RobotMessor(std::string configFilename):Robot(configFilename, "RobotMessor", TYPE_MESSOR2){};

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

	std::vector<Mat34> L_all; ///vector include translations for all legs from robot center
	Leg* Leg0;


};

#endif


