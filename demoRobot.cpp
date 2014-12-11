#include "include/defs/defs.h"
#include "include/robotModel/robot.h"
#include "include/robotModel/robotMessor2.h"
#include <iostream>
#include <stdio.h>
/*
Maciej Okoniewski
Joregus
Emil
*/
using namespace std;

int main( int argc, const char** argv )
{
    try {
        Robot* Rob;
        Rob = createRobotMessor();

        std::vector<float_type> configuration,Fz;

        configuration.push_back(5);

        Fz=Rob->computeCompliance(configuration);
           std::cout<<""<<std::endl;
           std::cout<<"wyniki Fz:"<<std::endl;
           std::cout<<""<<std::endl;
        for(int i=0;i<6;i++)
        {
            cout<<Fz[i]<<endl;

        }
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
