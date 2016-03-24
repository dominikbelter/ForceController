/** @file robot.h
 *
 * Robot interface
 * Emil Waledziak
 * Jerzy Wiatrow
 * Maciej Okoniewski
 * Marcin Zgolinski	
 */

#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "../defs/defs.h"
#include "../include/legModel/leg.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <string>
#include <vector>

namespace controller {
    /// Kinematic interface
    class Robot {
        public:

            /// Robot type
            enum Type {
                /// Messor Robot
                TYPE_MESSOR,
                /// Messor2 Robot
                TYPE_MESSOR2,
                /// StarlETH robot
                TYPE_STARLETH,
            };

            /// overloaded constructor
            Robot(const std::string _name, Type _type) : name(_name), type(_type) {};

            Robot(std::string configFilename, const std::string _name, Type _type) : name(_name), type(_type){

            }

            /// Name of the robot model
            virtual const std::string& getName() const { return name; }

            ///Compute configuration of the leg for the reference motion of the platform
            /**
            * @param motion - specified motion
            * @return reference values for servomotors
            */
            virtual std::vector<float_type> computeLegConfiguration(int legNo, const Mat34 bodyMotion, std::vector<float_type> startConfiguration) = 0;

            ///Compute configuration of the robot for the reference motion
            /**
            * @param motion - specified motion
            * @return reference values for servomotors
            */
            virtual std::vector<float_type> movePlatform(const Mat34& motion) = 0;

            virtual std::vector<float_type> moveLeg(unsigned char legNo, const Mat34& trajectory) = 0;

            ///Compute configuration of the robot for the reference motion (each foot generates separate motion)
            std::vector<float_type> movePlatform(const std::vector<Mat34>& motion);

            ///Compute configuration of the robot for the reference motion (in relation to neutral pose)
            /**
            * @param motion - specified motion
            * @return reference values for servomotors
            */
            virtual std::vector<float_type> movePlatformNeutral(const Mat34 motion) = 0;

            /// new method: computes forward kinematics for each leg and returns position of each link of the robot (body is [0,0,0]^T)
            virtual std::vector<Mat34> conputeLinksPosition(std::vector<float_type> configuration) = 0;

            ///Compute force in each joint of the legs, input configuration of the robot
            /**
            * @param vector<float_type>
            * @return tmp
            */
            virtual std::vector<float_type> computeCompliance(const std::vector<float_type> configuration) = 0;

            /// Virtual descrutor
            virtual ~Robot() {}

        protected:
            /// Robot type
            Type type;

            /// Robot name
            const std::string name;

            /// legs number
            int legsNo;

            /// Mounting points of legs
            std::vector<Mat34> legMountPoints;
    };
};

#endif // _ROBOT_H_
