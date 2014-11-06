#include "../include/robotModel/robotMessor2.h"
#include <iostream>

using namespace controller;





///Compute configuration of the robot for the reference motion
std::vector<float_type> movePlatform(const Mat34& motion)
{
    std::vector<float_type> tmp;
     return tmp;
}
		
///Compute configuration of the robot for the reference motion (in relation to neutral pose)
 std::vector<float_type> movePlatformNeutral(const Mat34 motion)
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
 std::vector<float_type> computeCompliance(std::vector<float_type>)
{
   
     std::vector<float_type> tmp;
      return tmp;

}
