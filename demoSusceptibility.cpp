
#include "include/defs/defs.h"
#include "include/robotModel/robot.h"
#include "include/robotModel/robotMessor2.h"
#include <iostream>
#include <stdio.h>
/*
Maciej Okoniewski
Marcin Zgolinski
*/
using namespace std;

int main( int argc, const char** argv )
{
    try {



        Robot* Rob;
        Rob = createRobotMessor();

        std::vector<float_type> configuration,T;
        std::vector<Mat34> pos;
        for(int i=0;i<18;i++)
         {
        configuration.push_back(i*2);
         }

           std::cout<<""<<std::endl;
           std::cout<<"Torque:"<<std::endl;
           std::cout<<""<<std::endl;
           T=Rob->computeCompliance(configuration);
       for(int i=0;i<18;i++)
        {
            std::cout<<T[i]<<std::endl;

        }

       pos=Rob->conputeLinksPosition(configuration);
       std::cout<<"dziala "<<pos[1](0,0)<<std::endl;
       return 0;

   }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
