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


void MoveLegsInThreads(int l, vector<Mat34> traj, float_type speed, RobotController* cont)
{
    cont->moveLeg(l, traj, speed);
}

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
        char wait;
        usleep(1000000);
        Mat34 robotPos(Mat34::Identity());
        robotPos(0,3) = -0.052;
        robotPos(1,3) = 0.12;
        robotPos(2,3) = 1;
        controller->moveLegSingleRobot(5, robotPos, speedo);
        while(true){};
        Mat34 motion1(Mat34::Identity());
        Mat34 motion2(Mat34::Identity());
        Mat34 motion3(Mat34::Identity());
        Mat34 motion4(Mat34::Identity());
        std::vector<Mat34> trajectoryForPlatform;
        controller->movePlatform(motion4,speedo);
        motion4(1,3)=0.03;
        motion2(1,3)=0.03;
        controller->movePlatform(motion4,speedo);
        motion4(0,3)=0.03;
        motion3(1,3)=0.03;
        motion3(0,3)=0.03;
        controller->movePlatform(motion4,speedo);
        motion4(2,3)=0.03;
        controller->movePlatform(motion4,speedo);

        trajectoryForPlatform.push_back(motion1);
        trajectoryForPlatform.push_back(motion2);
        trajectoryForPlatform.push_back(motion3);
        trajectoryForPlatform.push_back(motion4);
        while(true){
            controller->movePlatform(trajectoryForPlatform, speedo);
        };




//        if (((ControllerMessor2*)controller)->useVisualizer()){
//            ((ControllerMessor2*)controller)->finishVisualizer();
//        }

        std::vector<unsigned char> legNos024;
        std::vector<unsigned char> legNos135;
        std::vector<unsigned char> legNos024135;
        std::vector<unsigned char> legNos135024;


        legNos024.push_back(0);
        legNos024.push_back(2);
        legNos024.push_back(4);

        legNos135.push_back(1);
        legNos135.push_back(3);
        legNos135.push_back(5);

        legNos024135.push_back(0);
        legNos024135.push_back(2);
        legNos024135.push_back(4);
        legNos024135.push_back(1);
        legNos024135.push_back(3);
        legNos024135.push_back(5);

        legNos135024.push_back(1);
        legNos135024.push_back(3);
        legNos135024.push_back(5);
        legNos135024.push_back(0);
        legNos135024.push_back(2);
        legNos135024.push_back(4);

        Mat34 initial(Mat34::Identity());
        Mat34 legBack(Mat34::Identity());
        Mat34 legUp(Mat34::Identity());
        Mat34 legDown(Mat34::Identity());

        Mat34 legBackR(Mat34::Identity());
        Mat34 legUpR(Mat34::Identity());

        vector<Mat34> trajBack;
        vector<Mat34> trajBackToInitial;
        vector<Mat34> trajBackToInitialR;
        vector<Mat34> trajToInitial;
        vector<Mat34> trajUp;
        vector<Mat34> trajDown;

        vector<Mat34> trajBackR;
        vector<Mat34> trajUpR;


        std::vector<std::vector<Mat34>> executeLegsMovementBack;
        std::vector<std::vector<Mat34>> executeLegsMovementUp;
        std::vector<std::vector<Mat34>> executeLegsMovementInitial;
        std::vector<std::vector<Mat34>> executeLegsMovementBackUp;
        std::vector<std::vector<Mat34>> executeLegsMovementBackInitial;

        std::vector<std::vector<Mat34>> executeLegsMovementBackUpR;
        std::vector<std::vector<Mat34>> executeLegsMovementBackInitialR;


        legBack(1,3)=0.05;
        trajBack.push_back(legBack);

        legUp(2,3) = -0.1;
        legUp(0,3) = -0.05;
        trajUp.push_back(legUp);

        legBackR(1,3)=-0.05;
        trajBackR.push_back(legBackR);

        legUpR(2,3) = -0.1;
        legUpR(0,3) = 0.05;
        trajUpR.push_back(legUpR);

        legDown(2,3) = 0.1;
        trajDown.push_back(legDown);

        trajBackToInitial.push_back(legUp);
        trajBackToInitial.push_back(initial);

        trajBackToInitialR.push_back(legUpR);
        trajBackToInitialR.push_back(initial);

        trajToInitial.push_back(initial);

        executeLegsMovementBack.push_back(trajBack);
        executeLegsMovementBack.push_back(trajBack);
        executeLegsMovementBack.push_back(trajBackR);

        executeLegsMovementUp.push_back(trajUp);
        executeLegsMovementUp.push_back(trajUp);
        executeLegsMovementUp.push_back(trajUp);

        executeLegsMovementInitial.push_back(trajToInitial);
        executeLegsMovementInitial.push_back(trajToInitial);
        executeLegsMovementInitial.push_back(trajToInitial);

        executeLegsMovementBackInitial.push_back(trajBackToInitial);
        executeLegsMovementBackInitial.push_back(trajBackToInitial);
        executeLegsMovementBackInitial.push_back(trajBackToInitial);

        executeLegsMovementBackUp.push_back(trajUp);
        executeLegsMovementBackUp.push_back(trajUp);
        executeLegsMovementBackUp.push_back(trajUpR);
        executeLegsMovementBackUp.push_back(trajBack);
        executeLegsMovementBackUp.push_back(trajBack);
        executeLegsMovementBackUp.push_back(trajBack);

        executeLegsMovementBackUpR.push_back(trajUp);
        executeLegsMovementBackUpR.push_back(trajUpR);
        executeLegsMovementBackUpR.push_back(trajUpR);
        executeLegsMovementBackUpR.push_back(trajBack);
        executeLegsMovementBackUpR.push_back(trajBack);
        executeLegsMovementBackUpR.push_back(trajBack);


        usleep(1000000);
//        while(true)
//        {
//            controller->moveLegSingleLin(2, legUp, speedo);
//           // controller->moveLegSingleLin(2, legBack, speedo);
//           // controller->moveLegSingleLin(2, legUp, speedo);
//            controller->moveLegSingleLin(2, initial, speedo);
//        }

        //Robot wstaje do pozycji home
        //controller->moveLegs(legNos024, executeLegsMovementBackInitial, speedo);
        //controller->moveLegs(legNos135, executeLegsMovementBackInitial, speedo);

        bool threeLegMoveent = true;
        bool fiveLegMovement = false;
        ///////RUCH TRÓJPODPOROWY/////////
        if(threeLegMoveent)
        {
            while(true)
            {
                controller->moveLegs(legNos024135, executeLegsMovementBackUp, speedo);
                controller->moveLegs(legNos024, executeLegsMovementInitial, speedo);

                controller->moveLegs(legNos135024, executeLegsMovementBackUpR, speedo);
                controller->moveLegs(legNos135, executeLegsMovementInitial, speedo);

            }

            while(true)
            {
                controller->moveLegs(legNos024, executeLegsMovementUp, speedo);
                controller->moveLegs(legNos135, executeLegsMovementBack, speedo);

                controller->moveLegs(legNos024, executeLegsMovementInitial, speedo);
                controller->moveLegs(legNos135, executeLegsMovementUp, speedo);

                controller->moveLegs(legNos024, executeLegsMovementBack, speedo);
                controller->moveLegs(legNos135, executeLegsMovementInitial, speedo);
            }
        }

        ///////RUCH PIĘCIOPODPOROWY/////////
        if(fiveLegMovement)
        {
            executeLegsMovementBack.push_back(trajBackR);
            executeLegsMovementBack.push_back(trajBackR);

            std::vector<unsigned char> legNos;

            legNos.push_back(1);
            legNos.push_back(2);
            legNos.push_back(3);
            legNos.push_back(4);
            legNos.push_back(5);

            while(true)
            {
                controller->moveLeg(0, trajUp, speedo);
                controller->moveLegs(legNos, executeLegsMovementBack, speedo);

                controller->moveLeg(0, trajToInitial, speedo);

                controller->moveLeg(5, trajBackToInitialR, speedo);
                controller->moveLeg(1, trajBackToInitial, speedo);
                controller->moveLeg(4, trajBackToInitialR, speedo);
                controller->moveLeg(2, trajBackToInitial, speedo);
                controller->moveLeg(3, trajBackToInitialR, speedo);

            }
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
