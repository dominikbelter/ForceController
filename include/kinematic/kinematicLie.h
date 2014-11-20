/** @file kinematicLie.h
 *
 * implementation - Kinematic model which uses Lie's groups
 *
 */
 
 /**
* @author Adam Czeszejkowski
* @author Norbert Werbliñski
* @mainpage
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
           
        /** Definicja typu pointer
        */   
        
        typedef std::unique_ptr<KinematicLie> Ptr;

        /// Construction
        KinematicLie(void);

        /// Construction
        KinematicLie(std::string configFilename) : Kinematic(configFilename, "Kienamtic Lie", TYPE_LIE){};

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


};

#endif // BOARDDYNAMIXEL_H_INCLUDED
