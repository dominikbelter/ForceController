/** @file insectLeg.cpp 
*
* @author Lukasz Mejlun
* @author Wojciech Nowakowski
*/

#include "../include/legModel/insectLeg.h"
#include "../include/kinematic/kinematicLie.h"
#include <iostream>

using namespace controller;

/// A single instance of insect leg
InsectLeg::Ptr insectLeg;

InsectLeg::InsectLeg(void) : Leg("Insect Leg", TYPE_INSECT) 
{
	legKine = createKinematicLie();
}

InsectLeg::InsectLeg(std::string configFilename) : Leg(configFilename, "Insect Leg", TYPE_INSECT)
{
	tinyxml2::XMLDocument config;
	std::string filename = "../../resources/" + configFilename;
	config.LoadFile(filename.c_str());
	if (config.ErrorID())
	{
		std::cout << "unable to load Kinematic config file.\n";
	}
	else
	{
		linksNo = std::stoi(config.FirstChildElement("conf")->FirstChildElement("linksNo")->GetText());
		jointsNo = std::stoi(config.FirstChildElement("conf")->FirstChildElement("jointsNo")->GetText());

		tinyxml2::XMLElement * parameters;
		float_type paramf;
		parameters = config.FirstChildElement("Link0");
		parameters = parameters->FirstChildElement( "parameters" );
		parameters->QueryDoubleAttribute("length", &paramf); lengths[0] = paramf;
		parameters = config.FirstChildElement("Link1");
		parameters = parameters->FirstChildElement( "parameters" );
		parameters->QueryDoubleAttribute("length", &paramf); lengths[1] = paramf;
		parameters = config.FirstChildElement("Link2");
		parameters = parameters->FirstChildElement( "parameters" );
		parameters->QueryDoubleAttribute("length", &paramf); lengths[2] = paramf;

		std::cout << "links no: " << linksNo << " joints no: " << jointsNo << "\n";
		std::cout << "Lenght1: " << lengths[0] << std::endl;
		std::cout << "Lenght2: " << lengths[1] << std::endl;
		std::cout << "Lenght3: " << lengths[2] << std::endl;
	}
	legKine = createKinematicLie("../resources/legModel.xml");
}

InsectLeg::~InsectLeg(void) 
{
	legKine->~Kinematic();
}

/** Compute torque in each joint for given the force applied in the foot
* @param [in] force Indicator to the force vector which works in x,y,z axis
* @param [in] config vector of joints parameters of leg
* @return std::vector<float_type> load vector in individual nodes
*/
std::vector<float_type> InsectLeg::computLoad(Vec3& force, std::vector<float_type> config)
{
	std::vector<float_type> result;
	Vec3 torque;
	torque.vector() = -computeJacobian_transposed(config) * force.vector();

	for (int i = 0; i < 3; i++)
	{
		result.push_back(torque.vector().data()[i]);
	}

	return result;
}

/** Compute forward kinematic, default (-1) -- the last joint
* @param [in] configuration configuration variables legs
* @param [in] linkNo the number of nodes kinematic
* @return Mat34 homogeneous matrix legs
*/
Mat34 InsectLeg::forwardKinematic(std::vector<float_type> configuration, int linkNo, bool is_leg_left)
{
	if(is_leg_left)
	{
		Mat34 temp = legKine->forwardKinematic(configuration, linkNo);
				temp(0, 3) = -temp(0, 3);
				temp(1, 3) = -temp(1, 3);
					//float_type tmpf = temp(0,3);
				//temp(0,3)=-temp(1,3);
				//temp(1,3) = tmpf;
		return temp;
	}
	else
	{
        Mat34 temp = legKine->forwardKinematic(configuration, linkNo);
				//float_type tmpf = temp(0,3);
				//temp(0,3)=temp(1,3);
				//temp(1,3) = tmpf;
        return temp;
	}
}

/** Compute inverse kinematic, default (-1) -- the last joint
* @param [in] linkPose homogeneous matrix legs
* @param [in] linkNo the number of nodes kinematic
* @return std::vector<float_type> configuration variables legs
*/
std::vector<float_type> InsectLeg::inverseKinematic(Mat34 linkPose, int linkNo, bool is_leg_left)
{
	if(is_leg_left)
	{
		linkPose(2, 3) += M_PI / 2.0;

		return legKine->inverseKinematic(linkPose, linkNo);
	}
	else
	{
		return legKine->inverseKinematic(linkPose, linkNo);
	}
}

/// Jacobian of Messor leg
Mat33 InsectLeg::computeJacobian_transposed(std::vector<float_type> config)
{
	Mat33 temp;

	temp(0, 0) = -lengths[2] * sin(config[0]) * cos(config[1]) - lengths[1] * sin(config[0]);
	temp(1, 0) = -lengths[2] * cos(config[0]) * sin(config[1]);
	temp(2, 0) = 0;
	temp(0, 1) = lengths[2] * cos(config[0]) * cos(config[1]) + lengths[1] * cos(config[0]);
	temp(1, 1) = -lengths[2] * sin(config[0]) * sin(config[1]);
	temp(2, 1) = 0;
	temp(0, 2) = 0;
	temp(1, 2) = 0;
	temp(2, 2) = 0;

	return temp;
}

/** Constructor without arguments of Leg object*
 * @return controller::Leg* indicator to the Leg object
 */
controller::Leg* controller::createInsectLeg(void) 
{
	insectLeg.reset(new InsectLeg());
	return insectLeg.get();
}

/** Constructor of Leg object which argument is location of configuration file
 * @param [in] filename relative path to acces the file
 * @return controller::Leg* indicator to the Leg object
 */
controller::Leg* controller::createInsectLeg(std::string filename) 
{
	insectLeg.reset(new InsectLeg(filename));
  return insectLeg.get();
}
