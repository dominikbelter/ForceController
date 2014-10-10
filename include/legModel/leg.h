/** @file leg.h
 *
 * Leg interface
 *
 */

#ifndef _LEG_H_
#define _LEG_H_

#include "../defs/defs.h"
#include "../include/kinematic/kinematic.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <string>
#include <vector>

namespace controller {
    /// Kinematic interface
    class Leg {
        public:

            /// Leg type
            enum Type {
                /// Insect
                TYPE_INSECT,
                /// Mammal
                TYPE_MAMMAL,
            };

            /// overloaded constructor
            Leg(const std::string _name, Type _type) : name(_name), type(_type) {};

            Leg(std::string configFilename, const std::string _name, Type _type) : name(_name), type(_type){

            }

            /// Name of the leg model
            virtual const std::string& getName() const { return name; }

            ///Compute torque in each joint for given the force applied in the foot
            std::vector<float_type> computLoad(Vec3& force);

            /// Compute forward kinematic, default (-1) -- the last joint
            virtual Mat34 forwardKinematic(std::vector<float_type> configuration, unsigned int linkNo=-1) = 0;

            /// Compute forward kinematic, default (-1) -- the last joint
            virtual std::vector<float_type> inverseKinematic(Mat34 linkPose, unsigned int linkNo=-1) = 0;

            /// Virtual descrutor
            virtual ~Leg() {}

        protected:
            /// Leg type
            Type type;

            /// Leg name
            const std::string name;
    };
};

#endif // _LEG_H_
