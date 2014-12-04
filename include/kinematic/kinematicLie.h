/** @file kinematicLie.h
 *
 * implementation - Kinematic model which uses Lie's groups
 *
 * @author Adam Czeszejkowski
 * @author Norbert Werblinski
 *
 */
 

#ifndef KINEMATIC_LIE_H_INCLUDED
#define KINEMATIC_LIE_H_INCLUDED

#include "kinematic.h"
#include <iostream>
#include <chrono>
#include <memory>
#include <mutex>

namespace controller {
     /** create a single kinematic model (Lie's groups)
     */
    Kinematic* createKinematicLie(void);
    
     /** create a single kinematic model (Lie's groups) and load configuration from file
    */
    Kinematic* createKinematicLie(std::string filename);
};

using namespace controller;

/// Kinematic implementation
class KinematicLie : public Kinematic {
    public:
           
        /** Type Ptr definition
        */   
        
        typedef std::unique_ptr<KinematicLie> Ptr;

        /// Construction
        KinematicLie(void);

        /// Construction
		KinematicLie(std::string configFilename);

        /// Destructor
        ~KinematicLie(void);

        /// Name of the kienematic model
        const std::string& getName() const { return name; }
        
         /** Compute forward kinematic, default (-1) -- the last joint
         *
         * 
         *@param [in] configuration Vector of displacements and joint angles
         *@param [in] linkNo Link number
         *@return Matrix of the position, orientation
         *
         */
        Mat34 forwardKinematic(const std::vector<float_type>& configuration, unsigned int linkNo=-1);
        
          /** Compute inverse kinematic, default (-1) -- the last joint
         *
         * 
         *@param [in] linkPose Matrix of the position, orientation
         *@param [in] linkNo Link number
         *@return Vector of displacements and joint angles
         *
         */
        std::vector<float_type> inverseKinematic(const Mat34& linkPose, unsigned int linkNo=-1);
        
          /** Return set of link's poses
         *
         * 
         *@param [in] configuration Vector of displacements and joint angles
         *@return Vector of Matrix of the position, orientation for each joint
         *
         */
        std::vector<Mat34> getState(const std::vector<float_type>& configuration);

    private:
		Eigen::Matrix4d createEMatrix(const std::vector<float_type>& E, float_type angle)
		{
			Eigen::Matrix4d e;
			Eigen::Matrix3d w_, ewO, I;
			Eigen::Vector3d w(3), v(3), T(3);
			w << E[3], E[4], E[5];
			v << E[0], E[1], E[2];
			w_ << 0, -E[5], E[4], E[5], 0, -E[3], -E[4], E[3], 0;
			I.setIdentity();
			ewO = I + w_*sin(angle) + w_*w_*(1 - cos(angle));
			T = (I - ewO)*w.cross(v);
			e.setIdentity();
			e.topLeftCorner(3, 3) = ewO;
			e.col(3) << T[0], T[1], T[2],1;
			return e;
		}
		Eigen::Matrix4d createGMatrix(const std::vector<float_type>& g)
		{
			Eigen::Matrix4d e;
			e.setIdentity();
			e.col(3) << g[0], g[1], g[2], 1;
			return e;
		}
		//ksi tables
		std::vector<std::vector<float_type>> ksi;
		//g0 table
		std::vector<float_type> g0;
		/// number of joints
		unsigned int jointsNo;
		/// Number of links
		unsigned int linksNo;
		tinyxml2::XMLError parameters;
		tinyxml2::XMLElement * pElement;
		tinyxml2::XMLElement * pListElement;
		tinyxml2::XMLDocument conf;

};

#endif // BOARDDYNAMIXEL_H_INCLUDED
