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

	//Translation for each Leg

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


}



RobotMessor::~RobotMessor(void)
{

}

///Compute configuration of the robot for the reference motion
std::vector<float_type> RobotMessor::movePlatform(const Mat34& motion)
{
	std::vector<float_type> conf, conf2;


	Leg* Leg0;
	Leg0 = createInsectLeg();
	Robot* Rob;
	Rob = createRobotMessor();
	Mat34 tmp;

	//-----------------------------------------

	for (int i = 0; i<6; i++)
	{

		tmp = motion*L_all[i];
		conf2 = Leg0->inverseKinematic(tmp);
		conf.push_back(conf2[0]);
		conf.push_back(conf2[1]);
		conf.push_back(conf2[2]);
	}

	//-------------------------------------------


	return conf;
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
	Robot* Rob;
	Rob = createRobotMessor();


	//-----------------------------------------
	for (int h = 0; h<18; h + 3)
	{
		for (int j = 0; j<3; j++)
		{
			conf.push_back(configuration[j + h]);
		}

		for (int i = 0; i<3; i++)
		{
			linksPos.push_back(L_all[h / 3] * Leg0->forwardKinematic(conf, 0));
		}

		linksPos.push_back(L_all[h / 3] * Leg0->forwardKinematic(conf, -1));

		for (int k = 0; k>3; k++)
		{
			conf.pop_back();
		}
	}
	//-------------------------------------------

	return linksPos;
 }

///Compute force in each joint of the legs, input configuration of the robot
 std::vector<float_type> RobotMessor::computeCompliance(const std::vector<float_type> configuration)
{
     TorqueForce TF;

      return TF.susceptibility;

}

 controller::Robot* controller::createRobotMessor(void) {
     robotmessor.reset(new RobotMessor());
     return robotmessor.get();
 }

 controller::Robot* controller::createRobotMessor(std::string filename) {
     robotmessor.reset(new RobotMessor(filename));
     return robotmessor.get();
 }
