#include "include/defs/defs.h"
#include "include/board/board.h"
#include "include/robotController/controllerMessor2.h"
#include "include/robotModel/robotMessor2.h"
#include "include/board/boardDynamixel.h"
#include "3rdParty/dynamixel/dynamixel.h"
#include "3rdParty/dynamixel/dxl_hal.h"
#include <iostream>
#include <stdio.h>
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

        float_type speedo = 5;

        Mat34 initial(Mat34::Identity());
        Mat34 legBack(Mat34::Identity());
        Mat34 legUp(Mat34::Identity());
        Mat34 legUp0(Mat34::Identity());

        Mat34 legUpR(Mat34::Identity());

        vector<Mat34> trajBack;
        vector<Mat34> trajBackToInitial;
        vector<Mat34> trajBackToInitialR;
        vector<Mat34> trajToInitial;
        vector<Mat34> trajUp;
        vector<Mat34> trajUp0;

        vector<Mat34> trajUpR;

        std::vector<std::vector<Mat34>> executeLegsMovementBack;

        initial(1,3)=-0.04;

        legBack(1,3)=0.04;
        trajBack.push_back(legBack);

        legUp(2,3) = -0.1;
        legUp(0,3) = -0.05;
        trajUp.push_back(legUp);

        legUp0(2,3) = -0.1;
        legUp0(1,3)=-0.04;
        legUp0(0,3) = -0.05;
        trajUp0.push_back(legUp0);

        legUpR(2,3) = -0.1;
        legUpR(0,3) = 0.05;
        trajUpR.push_back(legUpR);

        trajBackToInitial.push_back(legUp);
        trajBackToInitial.push_back(initial);

        trajBackToInitialR.push_back(legUpR);
        trajBackToInitialR.push_back(initial);

        trajToInitial.push_back(initial);

        executeLegsMovementBack.push_back(trajBack);
        executeLegsMovementBack.push_back(trajBack);
        executeLegsMovementBack.push_back(trajBack);
        executeLegsMovementBack.push_back(trajBack);
        executeLegsMovementBack.push_back(trajBack);

        usleep(1000000);

        ///////RUCH PIĘCIOPODPOROWY/////////

        std::vector<unsigned char> legNos;

        legNos.push_back(1);
        legNos.push_back(2);
        legNos.push_back(3);
        legNos.push_back(4);
        legNos.push_back(5);

        while(true)
        {
            controller->moveLeg(0, trajUp0, speedo);
            controller->moveLegs(legNos, executeLegsMovementBack, speedo);

            controller->moveLeg(0, trajToInitial, speedo);

            controller->moveLeg(5, trajBackToInitialR, speedo);
            controller->moveLeg(1, trajBackToInitial, speedo);
            controller->moveLeg(4, trajBackToInitialR, speedo);
            controller->moveLeg(2, trajBackToInitial, speedo);
            controller->moveLeg(3, trajBackToInitialR, speedo);

        }

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
