#include "robot.h"


using namespace controller





///Compute configuration of the robot for the reference motion
virtual std::vector<float_type> movePlatform(const Mat34& motion)
{
	return 0;
}
		
///Compute configuration of the robot for the reference motion (in relation to neutral pose)
virtual std::vector<float_type> movePlatformNeutral(const Mat34 motion)
{
	return 0;
{