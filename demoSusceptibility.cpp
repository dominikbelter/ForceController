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
        std::vector<float_type> configuration,C;
        std::vector<Mat34> pos,pos2;

        Mat34 testmoveplatform;

        testmoveplatform.setIdentity();
        testmoveplatform(0, 3) = 0.3;
        testmoveplatform(1, 3) = 0;
        testmoveplatform(2, 3) = 0.1;


         configuration = Rob->movePlatform(testmoveplatform);

           std::cout<<""<<std::endl;
           std::cout<<"Compliance:"<<std::endl;
           std::cout<<""<<std::endl;
           C=Rob->computeCompliance(configuration);

       for(int i=0;i<18;i++)
        {
          std::cout<<C[i]<<std::endl;

        }

       Board *demo = createBoardDynamixel();


        ///DB proszę jeszcze utworzyć obiekt typu BoardDynamixel i wysłać obliczone podatności do serwomechanizmów
       return 0;

   }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
