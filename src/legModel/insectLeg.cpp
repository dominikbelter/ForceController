/** @file insectLeg.cpp 
*
* @author Lukasz Mejlun
* @author Wojciech Nowakowski
*/

#include "../include/legModel/insectLeg.h"
#include "../include/kinematic/kinematicLie.h"
#include "../include/board/board.h"
#include "../include/board/boardDynamixel.h"
#include <iostream>

using namespace controller;

/// A single instance of insect leg
InsectLeg::Ptr insectLeg;
//KinematicLie::Ptr kinematicLie;

InsectLeg::InsectLeg(void) : Leg("Insect Leg", TYPE_INSECT) 
{

}

InsectLeg::~InsectLeg(void) 
{

}

/** Compute torque in each joint for given the force applied in the foot
* @param [in] force Indicator to the force vector which works in x,y,z axis
* @return std::vector<float_type> load vector in individual nodes
*/
std::vector<float_type> InsectLeg::computLoad(Vec3& force, std::vector<float_type> config)
{
	using namespace Eigen;
	std::vector<float_type> result;
	float_type temp;
	Mat33 jacobian;

	jacobian = computeJacobian_transposed(config);

	for (int i = 0; i < 3; ++i)
	{
		temp = 0;
		temp += -jacobian(i, 0) * force.x();
		temp += -jacobian(i, 1) * force.y();
		temp += -jacobian(i, 2) * force.z();
		result.push_back(temp);
	}

  return result;
}

/** Compute forward kinematic, default (-1) -- the last joint
* @param [in] configuration configuration variables legs
* @param [in] linkNo the number of nodes kinematic
* @return Mat34 homogeneous matrix legs
*/
Mat34 InsectLeg::forwardKinematic(std::vector<float_type> configuration, int linkNo)
{
	Kinematic* demoKine;
	demoKine = createKinematicLie("../resources/legModel.xml");
	return demoKine->forwardKinematic(configuration, linkNo);
	//return kinematicLie->forwardKinematic(configuration, linkNo);
}

/** Compute inverse kinematic, default (-1) -- the last joint
* @param [in] linkPose homogeneous matrix legs
* @param [in] linkNo the number of nodes kinematic
* @return std::vector<float_type> configuration variables legs
*/
std::vector<float_type> InsectLeg::inverseKinematic(Mat34 linkPose, int linkNo)
{
	Kinematic* demoKine;
	demoKine = createKinematicLie("../resources/legModel.xml");
	return demoKine->inverseKinematic(linkPose, linkNo);
	//return kinematicLie->inverseKinematic(linkPose, linkNo);
}

/** Bezargumentowy konstruktor obiektu typu Leg*
 * @return controller::Leg* wskaznik na obiekt typu Leg
 */
controller::Leg* controller::createInsectLeg(void) 
{
  insectLeg.reset(new InsectLeg());
  return insectLeg.get();
}

/** Konstruktor obiektu typu Leg* przyjmujacy za argument polozenie pliku konfiguracyjnego typu xml
 * @param [in] filename wzgledna sciezka dostepu do pliku
 * @return controller::Leg* wskaznik na obiekt typu Leg
 */
controller::Leg* controller::createInsectLeg(std::string filename) 
{
  insectLeg.reset(new InsectLeg(filename));
  return insectLeg.get();
}
