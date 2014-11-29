/** @file insectLeg.h
*
* Leg interface for insects
*
* @author Lukasz Mejlun
* @author Wojciech Nowakowski
*/

#ifndef _INSECTLEG_H_INCLUDED
#define _INSECTLEG_H_INCLUDED

#include "leg.h"
#include <iostream>
#include <chrono>
#include <memory>
#include <mutex>

namespace controller 
{
	/** Constructor without arguments of Leg object*
	 * @return controller::Leg* indicator to the Leg object
	 */
	Leg* createInsectLeg(void);
	/** Constructor of Leg object which argument is location of configuration file
	 * @param [in] filename relative path to acces the file
	 * @return controller::Leg* indicator to the Leg object
	 */
	Leg* createInsectLeg(std::string filename);
}

using namespace controller;

/// Insect Leg implementation
class InsectLeg : public Leg 
{
	public:
		/// Pointer
		typedef std::unique_ptr<InsectLeg> Ptr;
	
		/** Constructor without arguments of Leg object*
		 * @return controller::Leg* indicator to the Leg object
		 */
		InsectLeg(void);
			
		/** Constructor of Leg object which argument is location of configuration file
		 * @param [in] configFilename relative path to acces the file
		 * @return controller::Leg* indicator to the Leg object
		 */
		InsectLeg(std::string configFilename) : Leg(configFilename, "Insect Leg", TYPE_INSECT)
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
		}
	
		/// Destructor
		~InsectLeg(void);
	
		/** Returns name of the leg model
		 * @return (const std::string&) name of the leg model
		 */
		const std::string& getName() const { return name; }

		/** Compute torque in each joint for given the force applied in the foot
		* @param [in] force Indicator to the force vector which works in x,y,z axis
		* @param [in] config vector of joints parameters of leg
		* @return std::vector<float_type> load vector in individual nodes
		*/
		std::vector<float_type> computLoad(Vec3& force, std::vector<float_type> config);

		/** Compute forward kinematic, default (-1) -- the last joint
		* @param [in] configuration configuration variables legs
		* @param [in] linkNo the number of nodes kinematic
		* @return Mat34 homogeneous matrix legs
		*/
		Mat34 forwardKinematic(std::vector<float_type> configuration, int linkNo = -1);

		/** Compute inverse kinematic, default (-1) -- the last joint
		* @param [in] linkPose homogeneous matrix legs
		* @param [in] linkNo the number of nodes kinematic
		* @return std::vector<float_type> configuration variables legs
		*/
		std::vector<float_type> inverseKinematic(Mat34 linkPose, int linkNo = -1);

		/** Returns number of links in leg
		* @return int number of links
		*/
		int getLinksNo() const { return linksNo; }

	private:
		/// number of joints
		int jointsNo;

		/// number of links
		int linksNo;

		/// lengths of legs
		float_type lengths[3];

		/// Jacobian of Messor leg
		Mat33 computeJacobian_transposed(std::vector<float_type> config)
		{
			Mat33 temp;
			float_type element;

			element = -lengths[2] * sin(config[0]) * cos(config[1]) - lengths[1] * sin(config[0]);
			temp(0, 0) = element;
			element = -lengths[2] * cos(config[0]) * sin(config[1]);
			temp(1, 0) = element;
			temp(2, 0) = 0;
			element = lengths[2] * cos(config[0]) * cos(config[1]) + lengths[1] * cos(config[0]);
			temp(0, 1) = element;
			element = -lengths[2] * sin(config[0]) * sin(config[1]);
			temp(1, 1) = element;
			temp(2, 1) = 0;
			temp(0, 2) = 0;
			temp(1, 2) = 0;
			temp(2, 2) = 0;

			return temp;
		}

};


#endif	// _INSECTLEG_H_INCLUDED
