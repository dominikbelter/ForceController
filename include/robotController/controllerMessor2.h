/** @file controllerMessor2.h
*
* controller interface for Messor 2 robot
*
* @author Dominik Belter
*/

#ifndef _CONTROLLERMESSOR2_H_INCLUDED
#define _CONTROLLERMESSOR2_H_INCLUDED

#include "robotController.h"
#include "../robotModel/robotMessor2.h"
#include "../board/boardDynamixel.h"
#include "visualization/visualizerIrrlicht.h"
#include <thread>
#include <iostream>

namespace controller
{
    RobotController* createControllerMessor2(void);
    RobotController* createControllerMessor2(std::string filename);
}

using namespace controller;

/// Robot Messor2 Controller implementation
class ControllerMessor2 : public RobotController
{
    public:
        /// Pointer
        typedef std::unique_ptr<ControllerMessor2> Ptr;

        /// Constructor
        ControllerMessor2(void);

        /// Constructor of ControllerMessor2 object which argument is location of configuration file
        ControllerMessor2(std::string configFilename);

        /// Destructor
        ~ControllerMessor2(void);

        class Config{
          public:
            Config() : verbose(0){
            }

            Config(std::string configFilename);
            public:
                /// Verbose
                int verbose;
                /// use visualizer
                bool useVisualizer;
        };

        /** Returns name of the ControllerMessor2
         * @return (const std::string&) name of the ControllerMessor2
         */
        const std::string& getName() const { return name; }

        ///Move platform
        void movePlatform(Mat34& motion, double speed);

        ///Compliant tripod step
        void tripodStepCompliant(Mat34& motion, double speed);

        void moveLegSingle(unsigned char legNo, const Mat34& trajectory, float_type speed);

        void moveLegSingleLin(unsigned char legNo, const Mat34& trajectory, float_type speed);

        void moveLeg(unsigned char legNo, const std::vector<Mat34>& trajectory, float_type speed);

        void moveLegs(std::vector<unsigned char> legNo, const std::vector<std::vector<Mat34> >& trajectory, float_type speed);

        /// use visualizer?
        inline bool useVisualizer(void) {return config.useVisualizer;}




        /// Wait for visualization thread to finish
        void finishVisualizer(void);

    private:
        ///configuration parameters
        Config config;
        ///Robot model
        Robot* robot;
        /// Board controller
        Board* board;
        /// Visualizer
        Visualizer* visualizer;

        /// visualizer thread
        void drawRobot(void);

        /// Visualizer thread
        std::unique_ptr<std::thread> visualizerThr;

        /// Visualizer thread
        bool continueVisualizer;
};

#endif	// _CONTROLLERMESSOR2_H_INCLUDED
