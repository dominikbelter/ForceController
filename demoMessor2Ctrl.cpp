#include "include/defs/defs.h"
#include "include/robotController/controllerMessor2.h"
#include <iostream>

int main( int argc, const char** argv ) {
    try {
        controller::RobotController* controller = createControllerMessor2("controllerMessor2.xml");
        usleep(1000000);
        Mat34 motion(Mat34::Identity());
        motion(1,3)=0.07;
        controller->movePlatform(motion,1.0);
        if (((ControllerMessor2*)controller)->useVisualizer()){
            ((ControllerMessor2*)controller)->finishVisualizer();
        }
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

