/** @file visualizer.h
 *
 * Visualizer interface
 * Tomasz Walczewski
 * Sebastian Drogowski
 */

#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

#include "../defs/defs.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <string>
#include <vector>

namespace controller {
    /// Visualizer interface
    class Visualizer {
        public:

            /// Visualizer type
            enum Type {
                /// Insect
                TYPE_GL,
            };

            /// overloaded constructor
            Visualizer(const std::string _name, Type _type) : name(_name), type(_type) {}

            Visualizer(std::string configFilename, const std::string _name, Type _type) : name(_name), type(_type){

            }

            /// Name of the Visualizer
            virtual const std::string& getName() const { return name; }

            ///Compute torque in each joint for given the force applied in the foot
            virtual void drawRobot(const Mat34& robotPose, std::vector<float_type> configuration) = 0;

            /// Virtual descrutor
            virtual ~Visualizer() {}

        protected:
            /// Visualizer type
            Type type;

            /// Visualizer name
            const std::string name;
    };
};

#endif // _VISUALIZER_H_
