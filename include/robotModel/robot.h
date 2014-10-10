/** @file robot.h
 *
 * Robot interface
 *
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

            ///Compute configuration of the robot for the reference motion
            virtual std::vector<float_type> movePlatform(const Mat34& motion) = 0;

            ///Compute configuration of the robot for the reference motion (in relation to neutral pose)
            virtual std::vector<float_type> movePlatformNeutral(const Mat34 motion) = 0;

            ///Compute force in each joint of the legs, input configuration of the robot
            virtual std::vector<float_type> computeCompliance(std::vector<float_type>) = 0;

            /// Virtual descrutor
            virtual ~Robot() {}

        protected:
            /// Robot type
            Type type;

            /// Robot name
            const std::string name;
    };
};

#endif // _ROBOT_H_
