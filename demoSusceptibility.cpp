#include "include/defs/defs.h"
#include "include/robotModel/robot.h"
#include "include/robotModel/robotMessor2.h"
#include "include/board/board.h"
#include "include/board/boardDynamixel.h"
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
        Mat34 move1;
        move1.setIdentity();
        move1(0, 3) = 0;
        move1(1, 3) = 0;
        move1(2, 3) = -0.05;
        Mat34 testmoveplatform;

        testmoveplatform.setIdentity();
        testmoveplatform(0, 3) = 0.0;
        testmoveplatform(1, 3) = 0;
        testmoveplatform(2, 3) = 0.0;


        Board *demo = createBoardDynamixel();

        for(int i = 0; i < 6; i++)
             {
                     demo->setSpeed(i, 0, 15);
                     demo->setSpeed(i, 1, 15);
                     demo->setSpeed(i, 2, 15);
             }

            for (int i = 0; i<6; i++)
           {
                 configuration.push_back(0);
                 configuration.push_back(24*3.14/180);
                 configuration.push_back(-114*3.14/180);
           }

            for(int i = 0; i < 6; i++)
            {
                int j = 3 * i;
                    demo->setPosition(i, 0, configuration[j] );
                    demo->setPosition(i, 1, configuration[j+1] );
                    demo->setPosition(i, 2, configuration[j+2] );
            }


         configuration = Rob->movePlatform(testmoveplatform);

         for (int i=0;i<configuration.size();i++){
             if (configuration[i]>3.14)
                 configuration[i]-=6.28;
             else if (configuration[i]<-3.14)
                 configuration[i]=+6.28;
         }

         for(int i = 0; i < 6; i++)
         {
             int j = 3 * i;
                 demo->setPosition(i, 0, configuration[j] );
                 demo->setPosition(i, 1, configuration[j+1] );
                 demo->setPosition(i, 2, configuration[j+2] );
         }

           std::cout<<""<<std::endl;
           std::cout<<"Compliance:"<<std::endl;
           std::cout<<""<<std::endl;
           C=Rob->computeCompliance(configuration);

       for(int i=0;i<18;i++)
        {
               std::cout<<C[i]<<std::endl;
        }

        demo->setTorqueLimit(C);
       //demo->setComplianceSlope( C );


       return 0;

   }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
