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

        bool startMotion = false;
        vector<float_type> readTorque(3);
        while(!startMotion)
        {
            board->readCurrent(5, 0, readTorque[0]);
            board->readCurrent(5, 1, readTorque[1]);
            board->readCurrent(5, 2, readTorque[2]);
            for (int i=0; i<3; i++)
            {
                cout << "NOGA 5 WENZEL " << i << "   " << readTorque[i] << endl;;
            }

            cout << endl;

            usleep(1000000);
        }

        for(int i=0; i<6; i++)
        {
            board->setPosition(i, position1);
        }

        RobotController* controller = createControllerMessor2("controllerMessor2.xml");

        float_type speedo = 5;

        usleep(1000000);

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

        Mat34 legUpR(Mat34::Identity());

        vector<Mat34> trajBack;
        vector<Mat34> trajToInitial;
        vector<Mat34> trajUp;

        vector<Mat34> trajUpR;

        std::vector<std::vector<Mat34>> executeLegsMovementInitial;
        std::vector<std::vector<Mat34>> executeLegsMovementBackUp;
        std::vector<std::vector<Mat34>> executeLegsMovementBackUpR;

        initial(1,3)=-0.04;

        legBack(1,3)=0.04;
        trajBack.push_back(legBack);

        legUp(2,3) = -0.1;
        legUp(0,3) = -0.05;
        trajUp.push_back(legUp);

        legUpR(2,3) = -0.1;
        legUpR(0,3) = 0.05;
        trajUpR.push_back(legUpR);

        trajToInitial.push_back(initial);

        executeLegsMovementInitial.push_back(trajToInitial);
        executeLegsMovementInitial.push_back(trajToInitial);
        executeLegsMovementInitial.push_back(trajToInitial);

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

        while(true)
        {
            controller->moveLegs(legNos024135, executeLegsMovementBackUp, speedo);
            controller->moveLegs(legNos024, executeLegsMovementInitial, speedo);

            controller->moveLegs(legNos135024, executeLegsMovementBackUpR, speedo);
            controller->moveLegs(legNos135, executeLegsMovementInitial, speedo);

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
