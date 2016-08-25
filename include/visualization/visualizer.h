/** @file visualizer.h
 *
 * Visualizer interface
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

            ///@param Visualizer type
            enum Type {
                TYPE_GL,
                TYPE_IRRLICHT,
            };

            /// overloaded constructor
            Visualizer(const std::string _name, Type _type) : name(_name), type(_type) {}

            Visualizer(std::string configFilename, const std::string _name, Type _type) : name(_name), type(_type){

            }


			/** Name of the Robot
            * @return name

            /** Name of the Visualizer
            * @return string name
			*/
            virtual const std::string& getName() const { return name; }

			/** Drawing a robot
            * @param robotPose - position of the robot
            * @param configuration - configuration of the robot
			*/
            virtual void drawRobot(const Mat34& robotPose, std::vector<float_type> configuration) = 0;

            virtual void setPosition(std::vector<float_type> configuration) = 0;

            virtual void setPosition(unsigned char legNo, std::vector<float_type> configuration) = 0;

            virtual std::vector<float_type> getPosition(int legNo) = 0;

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
