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
        move1(0, 3) = 0; ///DB to jest niepotrzebne, itd...
        move1(1, 3) = 0;
        move1(2, 3) = -0.05;
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

  for(int i = 0; i < 6; i++)
       {
               demo->setSpeed(i, 0, 15);
               demo->setSpeed(i, 1, 15);
               demo->setSpeed(i, 2, 15);
       }

       // tutaj macie katy 0,24,-114 dla kazdej nogi na sztywno wrzucone
      for (int i = 0; i<6; i++)
     {
           configuration.push_back(0);
           configuration.push_back(24*3.14/180);
           configuration.push_back(-114*3.14/180);
     }

      configurationmove1 = Rob->movePlatform(move1);
      for (int i=0;i<configurationmove5.size();i++){///DB pierwsze serwo powinno otrzymywac wartosci w oklicach zera (niezgodnosc kinematyki robota i sterownika)
          if (configurationmove1[i]>3.14)
              configurationmove1[i]-=6.28;
          else if (configurationmove1[i]<-3.14)
              configurationmove1[i]=+6.28;
      }
     for(int i = 0; i < 6; i++)
     {
         int j = 3 * i;
             demo->setPosition(i, 0, configurationmove1[j] );
             demo->setPosition(i, 1, configurationmove1[j+1] );
             demo->setPosition(i, 2, configurationmove1[j+2] );
     }


       demo->setComplianceSlope( C );

        ///DB proszę jeszcze utworzyć obiekt typu BoardDynamixel i wysłać obliczone podatności do serwomechanizmów
       return 0;

   }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
