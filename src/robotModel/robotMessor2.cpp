#include "../include/robotModel/robotMessor2.h"
#include <iostream>
#include "../include/legModel/insectLeg.h"

using namespace controller;




RobotMessor::Ptr robotmessor;

RobotMessor::RobotMessor(void) : Robot("Type Messor", TYPE_MESSOR2)
{
	width_max = 0.1025; ///distance from center to middle leg
	width_min = 0.052; /// distance from x to front leg
	length = 0.12; ///distance front legs from x




	L0(0, 0) = 1;
	L0(0, 1) = 0;
	L0(0, 2) = 0;
	L0(0, 3) = width_min;
	L0(1, 0) = 0;
	L0(1, 1) = 1;
	L0(1, 2) = 0;
	L0(1, 3) = length;
	L0(2, 0) = 0;
	L0(2, 1) = 0;
	L0(2, 2) = 1;
	L0(2, 3) = 0;

	L1(0, 0) = 1;
	L1(0, 1) = 0;
	L1(0, 2) = 0;
	L1(0, 3) = width_max;
	L1(1, 0) = 0;
	L1(1, 1) = 1;
	L1(1, 2) = 0;
	L1(1, 3) = 0;
	L1(2, 0) = 0;
	L1(2, 1) = 0;
	L1(2, 2) = 1;
	L1(2, 3) = 0;

	L2(0, 0) = 1;
	L2(0, 1) = 0;
	L2(0, 2) = 0;
	L2(0, 3) = width_min;
	L2(1, 0) = 0;
	L2(1, 1) = 1;
	L2(1, 2) = 0;
	L2(1, 3) = -length;
	L2(2, 0) = 0;
	L2(2, 1) = 0;
	L2(2, 2) = 1;
	L2(2, 3) = 0;

	L3(0, 0) = 1;
	L3(0, 1) = 0;
	L3(0, 2) = 0;
	L3(0, 3) = -width_min;
	L3(1, 0) = 0;
	L3(1, 1) = 1;
	L3(1, 2) = 0;
	L3(1, 3) = -length;
	L3(2, 0) = 0;
	L3(2, 1) = 0;
	L3(2, 2) = 1;
	L3(2, 3) = 0;

	L4(0, 0) = 1;
	L4(0, 1) = 0;
	L4(0, 2) = 0;
	L4(0, 3) = -width_max;
	L4(1, 0) = 0;
	L4(1, 1) = 1;
	L4(1, 2) = 0;
	L4(1, 3) = 0;
	L4(2, 0) = 0;
	L4(2, 1) = 0;
	L4(2, 2) = 1;
	L4(2, 3) = 0;

	L5(0, 0) = 1;
	L5(0, 1) = 0;
	L5(0, 2) = 0;
	L5(0, 3) = -width_min;
	L5(1, 0) = 0;
	L5(1, 1) = 1;
	L5(1, 2) = 0;
	L5(1, 3) = length;
	L5(2, 0) = 0;
	L5(2, 1) = 0;
	L5(2, 2) = 1;
	L5(2, 3) = 0;
	/*


	//   1.0,,0,width_min,
	//    0,1.0,0,length,
	//    0,0,1.0,0;

	L1<<1,0,0,0,width_max,
	0,1,0,0,
	0,0,1,0;

	L2<<1,0,0,width_min,
	0,1,0,-length,
	0,0,1,0;

	L3<<1,0,0,-width_min,
	0,1,0,-length,
	0,0,1,0;

	L4<<1,0,0,-width_max,
	0,1,0,0,
	0,0,1,0;

	L5<<1,0,0,-width_min,
	0,1,0,length,
	0,0,1,0;

	*/
	L_all.push_back(L0);
	L_all.push_back(L1);
	L_all.push_back(L2);
	L_all.push_back(L3);
	L_all.push_back(L4);
	L_all.push_back(L5);


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

	std::vector<Mat34> linksPos;
	std::vector<float_type> conf;






	Leg* Leg0;
	Leg0 = createInsectLeg();
	//Robot* Rob;
	//Rob=createRobotMessor();


	//-----------------------------------------
	for (int h = 0; h<18; h + 3)
	{
		for (int j = 0; j<3; j++)
		{
			conf.push_back(configuration[j + h]);
		}

		for (int i = 0; i<3; i++)
		{
			//     linksPos.push_back(L_all[h/3]*forwardKinematic(conf, 0));
		}

		// linksPos.push_back(L0*forwardKinematic(conf, -1));

		for (int k = 0; k>3; k++)
		{
			conf.pop_back();
		}
	}
	//-------------------------------------------
	//return Leg0;
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
