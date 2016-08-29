/** @file robotController.h
 *
 * robotController interface
 *
 */

#ifndef _ROBOTCONTROLLER_H_
#define _ROBOTCONTROLLER_H_

#include "../defs/defs.h"
#include "../include/kinematic/kinematic.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <string>
#include <vector>

namespace controller {
    /// Robot controller interface
    class RobotController {
        public:

            /// controller type
            enum Type {
                /// Messor2
                TYPE_MESSOR2,
            };

            /// overloaded constructor
            RobotController(const std::string _name, Type _type) : name(_name), type(_type) {};

            RobotController(std::string configFilename, const std::string _name, Type _type) : name(_name), type(_type){

            }

            /// Name of the leg model
            virtual const std::string& getName() const { return name; }

            ///Move platform
            virtual void movePlatform(Mat34& motion, double speed) = 0;

            ///Compliant tripod step
            virtual void tripodStepCompliant(Mat34& motion, double speed) = 0;

            virtual void moveLegSingleLin(unsigned char legNo, const Mat34& trajectory, float_type speed) = 0;

            //methods for moving robot legs by passing trajectory in the leg coordinate system
            virtual void moveLegSingle(unsigned char legNo, const Mat34& trajectory, float_type speed) = 0;

            virtual void moveLeg(unsigned char legNo, const std::vector<Mat34>& trajectory, float_type speed) = 0;

            virtual void moveLegs(std::vector<unsigned char> legNo, const std::vector<std::vector<Mat34> >& trajectory, float_type speed) = 0;

            //methods for moving robot legs by passing trajectory in the robot coordinate system
            virtual void moveLegSingleRobot(unsigned char legNo, const Mat34& trajectory, float_type speed) = 0;

            virtual void moveLegRobot(unsigned char legNo, const std::vector<Mat34>& trajectory, float_type speed) = 0;

            virtual void moveLegsRobot(std::vector<unsigned char> legNo, const std::vector<std::vector<Mat34> >& trajectory, float_type speed) = 0;

            virtual void movePlatform(std::vector<Mat34>& motion, double speed) = 0;

            //methods for moving robot legs by passing motors configuration
            virtual void moveLegSingle(unsigned char legNo,const std::vector<float_type>& configuration1, float_type speed) = 0;

            virtual void moveLegConf(unsigned char legNo,const std::vector<std::vector<float_type> >& configuration, float_type speed) = 0;

            virtual void moveLegs(std::vector<unsigned char> legNo,const  std::vector<std::vector<std::vector<float_type> > >& configuration, float_type speed) = 0;


            /// Virtual descrutor
            virtual ~RobotController() {}

        protected:
            /// Robot Controller type
            Type type;

            /// Robot Controller name
            const std::string name;
    };
}

#endif // _ROBOTCONTROLLER_H_
