#include "include/defs/defs.h"
#include "include/board/board.h"
#include "include/robotController/controllerMessor2.h"
#include "include/robotModel/robotMessor2.h"
#include "include/board/boardDynamixel.h"
#include "3rdParty/dynamixel/dynamixel.h"
#include "3rdParty/dynamixel/dxl_hal.h"
#include <iostream>
#include <stdio.h>
#include <thread>
using namespace std;
using namespace controller;



int main( int argc, const char** argv )
{

    try {
        vector <float_type> motorSpeed;
        for(int i = 0; i < 18; i++ ){
        motorSpeed.push_back(10);
        }
        Board* board = createBoardDynamixel();

        std::vector<float_type> position1;
        position1.push_back((0*M_PI)/180);
        position1.push_back((24*M_PI)/180);
        position1.push_back((114*M_PI)/180);


        board->setSpeed(motorSpeed);

        for(int i=0; i<6; i++)
        {
            board->setPosition(i, position1);
        }

        RobotController* controller = createControllerMessor2("controllerMessor2.xml");

        float_type speedo = ((ControllerMessor2*)controller)->speedPajak();

        //float_type speedo = 15;
        char wait;
        usleep(1000000);
       // cin >> wait;
        Mat34 motion1(Mat34::Identity());
        Mat34 motion2(Mat34::Identity());
        Mat34 motion3(Mat34::Identity());
        Mat34 motion4(Mat34::Identity());
        Mat34 motion5(Mat34::Identity());
        std::vector<Mat34> trajectoryForPlatform;

        motion2(1,3)=0.04;

        motion3(1,3)=0.04;
        motion3(0,3)=0.03;


        motion4(0,3)=0.04;
        motion4(1,3)=0.03;
        motion4(2,3)=0.05;

        motion5(2,3)=0.05;


        trajectoryForPlatform.push_back(motion1);
        trajectoryForPlatform.push_back(motion2);
        trajectoryForPlatform.push_back(motion3);
        trajectoryForPlatform.push_back(motion4);
        trajectoryForPlatform.push_back(motion5);

        while(true){
            controller->movePlatform(trajectoryForPlatform, speedo);
            //if(speedo<30)
                //speedo+=5;
        };



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
