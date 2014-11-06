#include "../include/robotModel/robot.h"
#include <iostream>

using namespace controller;





///Compute configuration of the robot for the reference motion
/**
* @param motion
* @return tmp
*/
std::vector<float_type> movePlatform(const Mat34& motion)
{
    std::vector<float_type> tmp;
     return tmp;
}
		
///Compute configuration of the robot for the reference motion (in relation to neutral pose)
/**
* @param motion
* @return tmp
*/
 std::vector<float_type> movePlatformNeutral(const Mat34 motion)
{
     std::vector<float_type> tmp;
      return tmp;
}

///Compute force in each joint of the legs, input configuration of the robot
 /**
 * @param vector<float_type>
 * @return tmp
 */
 std::vector<float_type> computeCompliance(std::vector<float_type>)
{
   
     std::vector<float_type> tmp;
      return tmp;

}
