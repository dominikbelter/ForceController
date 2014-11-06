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
	/// create a single leg model (insect type)
	Leg* createInsectLeg(void);
	/// create a single leg model (insect type) and load configuration from file
	Leg* createInsectLeg(std::string filename);
};

using namespace controller;

/// Insect Leg implementation
class InsectLeg : public Leg 
{
	public:
		/// Pointer
		typedef std::unique_ptr<InsectLeg> Ptr;
	
		/// Construction
		InsectLeg(void);
			
		/// Construction
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
		      tinyxml2::XMLElement * parameters = config.FirstChildElement( "parameters" );
		      int param;
		      parameters->QueryIntAttribute("linksNo", &param); linksNo = param;
		      parameters->QueryIntAttribute("jointsNo", &param); jointsNo = param;

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
	
		/// Name of the leg model
		const std::string& getName() const { return name; }

		/// Compute torque in each joint for given the force applied in the foot
		std::vector<float_type> computLoad(Vec3& force);

		/// Compute forward kinematic, default (-1) -- the last joint
		Mat34 forwardKinematic(std::vector<float_type> configuration, unsigned int linkNo = -1);

		/// Compute inverse kinematic, default (-1) -- the last joint
		std::vector<float_type> inverseKinematic(Mat34 linkPose, unsigned int linkNo = -1);

	private:
		/// number of joints
		unsigned int jointsNo;

                /// Number of links
                unsigned int linksNo;

                ///
                float_type lengths[3];

};


#endif	// _INSECTLEG_H_INCLUDED
