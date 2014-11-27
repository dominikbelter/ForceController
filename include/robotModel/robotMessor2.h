/**
* @author Maciej Okoniewski
* @author Emil Waledziak
* @author Jerzy Wiatrow
* @author Marcin Zgolinski
* @mainpage
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
    * @param motion - specified motion
    * @return tmp
    */
    std::vector<float_type> movePlatform(const Mat34& motion);

    ///Compute configuration of the robot for the reference motion (in relation to neutral pose)
    /**
    * @param motion - specified motion
    * @return tmp
    */
    std::vector<float_type> movePlatformNeutral(const Mat34 motion);

    /// new method: computes forward kinematics for each leg and returns position of each link of the robot (body is [0,0,0]^T)
    std::vector<Mat34> conputeLinksPosition(std::vector<float_type> configuration);

    ///Compute force in each joint of the legs, input configuration of the robot
    /**
    * @param vector<float_type>
    * @return torque
    */
    std::vector<float_type> computeCompliance(std::vector<float_type>);

    ~RobotMessor(void);

private:

    float_type width_max;
    float_type width_min;
    float_type length;

    Mat34 Ps0;
    Mat34 Ps1;
    Mat34 Ps2;
    Mat34 Ps3;
    Mat34 Ps4;
    Mat34 Ps5;

    Mat34 L0;
    Mat34 L1;
    Mat34 L2;
    Mat34 L3;
    Mat34 L4;
    Mat34 L5;
};

#endif


