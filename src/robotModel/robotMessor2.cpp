#include "../include/robotModel/robotMessor2.h"
#include <iostream>

using namespace controller;




RobotMessor::Ptr robotmessor;

RobotMessor::RobotMessor(void) : Robot("Type Messor", TYPE_MESSOR2)
{

}


RobotMessor::~RobotMessor(void)
{

}

///Compute configuration of the robot for the reference motion
std::vector<float_type> RobotMessor::movePlatform(const Mat34& motion)
{
    std::vector<float_type> tmp;
     return tmp;
}
		
///Compute configuration of the robot for the reference motion (in relation to neutral pose)
 std::vector<float_type> RobotMessor::movePlatformNeutral(const Mat34 motion)
{
     std::vector<float_type> tmp;
      return tmp;
}

 /// new method: computes forward kinematics for each leg and returns position of each link of the robot (body is [0,0,0]^T)
std::vector<Mat34> RobotMessor::conputeLinksPosition(std::vector<float_type> configuration){
     std::vector<Mat34> tmp;
     return tmp;
 }

///Compute force in each joint of the legs, input configuration of the robot
 std::vector<float_type> RobotMessor::computeCompliance(std::vector<float_type>)
{
   
    std::vector<float_type> torque;
      return torque;

}

 controller::Robot* controller::createRobotMessor(void) {
     robotmessor.reset(new RobotMessor());
     return robotmessor.get();
 }

 controller::Robot* controller::createRobotMessor(std::string filename) {
     robotmessor.reset(new RobotMessor(filename));
     return robotmessor.get();
 }
