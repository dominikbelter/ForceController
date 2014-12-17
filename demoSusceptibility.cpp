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
        std::vector<Mat34> pos,pos2;

        Mat34 testmoveplatform;

        testmoveplatform.setIdentity();
        testmoveplatform(0, 3) = 0;
        testmoveplatform(1, 3) = 0;
        testmoveplatform(2, 3) = 0;

         configuration.push_back(18);
         configuration = Rob->movePlatform(testmoveplatform);

        std::cout<<""<<std::endl;
        /*   std::cout<<"Torque:"<<std::endl;
           std::cout<<""<<std::endl;
           T=Rob->computeCompliance(configuration);
       for(int i=0;i<18;i++)
        {
            std::cout<<T[i]<<std::endl;

        }
*/
        std::vector<float_type> l1,l2,l3,l4,l5,l6;
       pos=Rob->conputeLinksPosition(configuration);

       for(int i=3; i<pos.size(); i+=4)
       {
         pos2.push_back(pos[i]);

       }

       for(int i=0;i<3;i++)
       {
       l1.push_back(pos2[0](i,3));
       l2.push_back(-1*pos2[1](i,3));
       l3.push_back(pos2[2](i,3));
       l4.push_back(pos2[3](i,3));
       l5.push_back(-1*pos2[4](i,3));
       l6.push_back(pos2[5](i,3));
       }

      /* l1[2]=-l1[2];
       l2[2]=-l2[2];
       l3[2]=-l3[2];
       l4[2]=-l4[2];
       l5[2]=-l5[2];
       l6[2]=-l6[2];*/
        cout<<" l1z="<<l1[2]<<" l2z="<<l2[2]<<" l3z="<<l3[2]<<" l4z="<<l4[2]<<" l5z="<<l5[2]<<" l6z="<<l6[2]<<" "<<endl;
        cout<<" "<<endl;

       return 0;

   }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
