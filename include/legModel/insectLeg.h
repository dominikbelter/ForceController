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

/// Kinematic implementation
class InsectLeg : public Leg 
{
	public:
		/// Pointer
		typedef std::unique_ptr<InsectLeg> Ptr;
	
		/// Construction
		InsectLeg(void);
			
		/// Construction
		InsectLeg(std::string configFilename) : Leg(configFilename, "Insect Leg", TYPE_INSECT){};
	
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


};


#endif	// _INSECTLEG_H_INCLUDED
