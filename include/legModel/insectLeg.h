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
		InsectLeg(std::string configFilename);
	
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

		/** Compute torque in each joint for given the force applied in the foot
		* @param [in] force Indicator to the force vector which works in x,y,z axis
		* @param [in] config vector of joints parameters of leg
		* @param [in] is_leg_left is the leg on the left side of robot
		* @return std::vector<float_type> load vector in individual nodes
		*/
		std::vector<float_type> computLoad(Vec3& force, std::vector<float_type> config, bool is_leg_left);

		/** Compute forward kinematic, default (-1) -- the last joint
		* @param [in] configuration configuration variables legs
		* @param [in] linkNo the number of nodes kinematic
		* @return Mat34 homogeneous matrix legs
		*/
		Mat34 forwardKinematic(std::vector<float_type> configuration, int linkNo = -1, bool is_leg_left = false);

		/** Compute inverse kinematic, default (-1) -- the last joint
		* @param [in] linkPose homogeneous matrix legs
		* @param [in] linkNo the number of nodes kinematic
		* @return std::vector<float_type> configuration variables legs
		*/
        std::vector<float_type> inverseKinematic(Mat34& linkPose, int linkNo = -1, bool is_leg_left = false);

		/** Returns number of links in leg
		* @return int number of links
		*/
		int getLinksNo() const { return linksNo; }

	private:
		/// number of joints
		int jointsNo;

		/// number of links
		int linksNo;

		/// kinematic model of leg
		Kinematic* legKine;

		/// lengths of legs
		float_type lengths[3];

		/// Jacobian of Messor leg
		Mat33 computeJacobian_transposed(std::vector<float_type> config);

};


#endif	// _INSECTLEG_H_INCLUDED
