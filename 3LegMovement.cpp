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

        std::vector<float_type> position2;
        position2.push_back((10*M_PI)/180);
        position2.push_back((34*M_PI)/180);
        position2.push_back((84*M_PI)/180);


        std::vector<float_type> position3;
        position3.push_back((-10*M_PI)/180);
        position3.push_back((16*M_PI)/180);
        position3.push_back((90*M_PI)/180);



        board->setSpeed(motorSpeed);

        bool startMotion = false;
        vector<float_type> readTorque(3);
//        while(!startMotion)
//        {
//            board->readTorque(5, 0, readTorque[0]);
//            board->readTorque(5, 1, readTorque[1]);
//            board->readTorque(5, 2, readTorque[2]);
//            for (int i=0; i<3; i++)
//            {
//                cout << "NOGA 5 WENZEL " << i << "   " << readTorque[i] << endl;;
//            }

//            cout << endl;

//            usleep(1000000);
//        }

        for(int i=0; i<6; i++)
        {
            board->setPosition(i, position1);
        }

        usleep(1000000);




        /*while(true)
        {
            board->setSpeed(5,speedTest);
            board->setPosition(5, position1);
            usleep(1000000);
            board->setPosition(5, position2);
            usleep(1000000);
            board->setPosition(5, position3);
            usleep(1000000);
            if(speedTest[0]<50)
            {
                speedTest[0]+=5;
                speedTest[1]+=5;
                speedTest[2]+=5;
            }
        }*/


        RobotController* controller = createControllerMessor2("controllerMessor2.xml");

        float_type speedo = 20;
        char start;
        cin >> start;


        std::vector<unsigned char> legNos024135;
        std::vector<unsigned char> legNos135024;

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

        Mat34 initialFront(Mat34::Identity());
        Mat34 initialMid(Mat34::Identity());
        Mat34 initialBack(Mat34::Identity());

        Mat34 legBackFront(Mat34::Identity());
        Mat34 legBackMid(Mat34::Identity());
        Mat34 legBackBack(Mat34::Identity());

        Mat34 legUpFront(Mat34::Identity());
        Mat34 legUpMid(Mat34::Identity());
        Mat34 legUpBack(Mat34::Identity());

        Mat34 legUpRFront(Mat34::Identity());
        Mat34 legUpRMid(Mat34::Identity());
        Mat34 legUpRBack(Mat34::Identity());

        vector<Mat34> trajBackFront;
        vector<Mat34> trajUpFront;
        vector<Mat34> trajUpRFront;

        vector<Mat34> trajBackMid;
        vector<Mat34> trajUpMid;
        vector<Mat34> trajUpRMid;

        vector<Mat34> trajBackBack;
        vector<Mat34> trajUpBack;
        vector<Mat34> trajUpRBack;


        std::vector<std::vector<Mat34>> executeLegsMovementBackUp;
        std::vector<std::vector<Mat34>> executeLegsMovementBackUpR;

        initialFront(1,3)=-0.08;
        legBackFront(1,3)=0.00;
        legUpFront(0,3) = -0.05;
        legUpFront(1,3) = -0.04;
        legUpFront(2,3) = -0.1;

        trajBackFront.push_back(legBackFront);
        trajUpFront.push_back(legUpFront);
        trajUpFront.push_back(initialFront);

        initialMid(1,3)=-0.04;
        legBackMid(1,3)=0.04;
        legUpMid(0,3) = -0.05;
        legUpMid(1,3) = 0.00;
        legUpMid(2,3) = -0.1;

        trajBackMid.push_back(legBackMid);
        trajUpMid.push_back(legUpMid);
        trajUpMid.push_back(initialMid);

        initialBack(1,3)=0.00;
        legBackBack(1,3)=0.08;
        legUpBack(0,3) = -0.05;
        legUpBack(1,3) = 0.04;
        legUpBack(2,3) = -0.1;

        trajBackBack.push_back(legBackBack);
        trajUpBack.push_back(legUpBack);
        trajUpBack.push_back(initialBack);

        legUpRFront(0,3) = 0.05;
        legUpRFront(1,3) = -0.04;
        legUpRFront(2,3) = -0.1;

        legUpRBack(0,3) = 0.05;
        legUpRBack(1,3) = 0.04;
        legUpRBack(2,3) = -0.1;

        legUpRMid(0,3) = 0.05;
        legUpRMid(1,3) = 0.00;
        legUpRMid(2,3) = -0.1;

        trajUpRFront.push_back(legUpRFront);
        trajUpRFront.push_back(initialFront);

        trajUpRMid.push_back(legUpRMid);
        trajUpRMid.push_back(initialMid);

        trajUpRBack.push_back(legUpRBack);
        trajUpRBack.push_back(initialBack);


        executeLegsMovementBackUp.push_back(trajUpFront);
        executeLegsMovementBackUp.push_back(trajUpBack);
        executeLegsMovementBackUp.push_back(trajUpRMid);
        executeLegsMovementBackUp.push_back(trajBackMid);
        executeLegsMovementBackUp.push_back(trajBackBack);
        executeLegsMovementBackUp.push_back(trajBackFront);

        executeLegsMovementBackUpR.push_back(trajUpMid);
        executeLegsMovementBackUpR.push_back(trajUpRBack);
        executeLegsMovementBackUpR.push_back(trajUpRFront);
        executeLegsMovementBackUpR.push_back(trajBackFront);
        executeLegsMovementBackUpR.push_back(trajBackBack);
        executeLegsMovementBackUpR.push_back(trajBackMid);





        while(true)
        {
            controller->moveLegSingle(5,position1,speedo,false);
            controller->moveLegSingle(5,position2,speedo,false);
            controller->moveLegSingle(5,position3,speedo,false);
            if(speedo<50)
            {
                speedo+=5;
            }

            //controller->moveLegs(legNos024135, executeLegsMovementBackUp, speedo);
           // controller->moveLegs(legNos024, executeLegsMovementInitial, speedo);

            //controller->moveLegs(legNos135024, executeLegsMovementBackUpR, speedo);
           // controller->moveLegs(legNos135, executeLegsMovementInitial, speedo);

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
