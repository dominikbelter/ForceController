#include "include/defs/defs.h"
#include "include/localization/ahrs.h"
#include <iostream>
#include <stdio.h>
using namespace std;
using namespace controller;


int main( int argc, const char** argv ) {
    try {
        Localizer * ahrs = createLocalizerAHRS();
        while (1){
            double roll, pitch, yaw;
            ahrs->getBodyOrientation(roll, pitch, yaw);
            std::cout << "rol, pitch, yaw: " << roll << ", " << pitch << ", " << yaw << "\n";
        }
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }


    return 0;
}
