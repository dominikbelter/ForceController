/** @file kinematic.h
 *
 * Kinematic interface
 *
 */

#ifndef _KINEMATIC_H_
#define _KINEMATIC_H_

#include "../defs/defs.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <string>
#include <vector>

namespace controller {
    /// Kinematic interface
    class Kinematic {
        public:

            /// Kinematic type
            enum Type {
                    /// Lie groups
                    TYPE_LIE,
                    /// Denavit-Hartenberg
                    TYPE_DENAVIT_HARTNBERG,
            };

            /// overloaded constructor
            Kinematic(const std::string _name, Type _type) : name(_name), type(_type) {};

            Kinematic(std::string configFilename, const std::string _name, Type _type) : name(_name), type(_type){
                tinyxml2::XMLDocument config;
                std::string filename = "../../resources/" + configFilename;
                config.LoadFile(filename.c_str());
                if (config.ErrorID())
                    std::cout << "unable to load Kinematic config file.\n";
                else {
                    tinyxml2::XMLElement * parameters = config.FirstChildElement( "parameters" );
                    int param;
                    parameters->QueryIntAttribute("linksNo", &param); linksNo = param;
                    parameters->QueryIntAttribute("jointsNo", &param); jointsNo = param;
                    std::cout << "links no: " << linksNo << " joints no: " << jointsNo << "\n";
                }
            }

            /// Name of the kienematic model
            virtual const std::string& getName() const { return name; }

            /// Compute forward kinematic, default (-1) -- the last joint
            virtual Mat34 forwardKinematic(const std::vector<float_type>& configuration, unsigned int linkNo=-1) = 0;

            /// Compute forward kinematic, default (-1) -- the last joint
            virtual std::vector<float_type> inverseKinematic(const Mat34& linkPose, unsigned int linkNo=-1) = 0;

            /// Return set of link's poses
            virtual std::vector<Mat34> getState(const std::vector<float_type>& configuration) = 0;

            /// Virtual descrutor
            virtual ~Kinematic() {}

        protected:
            /// Board type
            Type type;

            /// Board name
            const std::string name;

            /// number of joints
            unsigned int jointsNo;

            /// Number of links
            unsigned int linksNo;
    };
};

#endif // _BOARD_H_
