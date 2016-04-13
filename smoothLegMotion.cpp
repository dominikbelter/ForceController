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
        RobotController* controller = createControllerMessor2("controllerMessor2.xml");

        float_type speedo = 10;


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

        vector<Mat34> trajBack;
        vector<Mat34> trajBackToInitial;
        vector<Mat34> trajToInitial;
        vector<Mat34> trajUp;
        vector<Mat34> trajDown;

        std::vector<std::vector<Mat34>> executeLegsMovementBack;
        std::vector<std::vector<Mat34>> executeLegsMovementUp;
        std::vector<std::vector<Mat34>> executeLegsMovementInitial;
        std::vector<std::vector<Mat34>> executeLegsMovementBackUp;
        std::vector<std::vector<Mat34>> executeLegsMovementBackInitial;


        legBack(1,3)=0.07;
        trajBack.push_back(legBack);

        legUp(2,3) = -0.1;
        legUp(0,3) = -0.05;
        trajUp.push_back(legUp);

        legDown(2,3) = 0.1;
        trajDown.push_back(legDown);

        trajBackToInitial.push_back(legUp);
        trajBackToInitial.push_back(initial);

        trajToInitial.push_back(initial);

        executeLegsMovementBack.push_back(trajBack);
        executeLegsMovementBack.push_back(trajBack);
        executeLegsMovementBack.push_back(trajBack);

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
        executeLegsMovementBackUp.push_back(trajUp);
        executeLegsMovementBackUp.push_back(trajBack);
        executeLegsMovementBackUp.push_back(trajBack);
        executeLegsMovementBackUp.push_back(trajBack);


        usleep(1000000);
        while(true)
        {
            controller->moveLegSingle(0, legUp, speedo);
            controller->moveLegSingle(0, legBack, speedo);
            controller->moveLegSingle(0, legUp, speedo);
            controller->moveLegSingle(0, initial, speedo);
        }

        //Robot wstaje do pozycji home
        //controller->moveLegs(legNos024, executeLegsMovementBackInitial, speedo);
        //controller->moveLegs(legNos135, executeLegsMovementBackInitial, speedo);

        bool threeLegMoveent = false;
        bool fiveLegMovement = false;
        ///////RUCH TRÓJPODPOROWY/////////
        if(threeLegMoveent)
        {
            while(true)
            {
                controller->moveLegs(legNos024135, executeLegsMovementBackUp, speedo);
                controller->moveLegs(legNos024, executeLegsMovementInitial, speedo);

                controller->moveLegs(legNos135024, executeLegsMovementBackUp, speedo);
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
            executeLegsMovementBack.push_back(trajBack);
            executeLegsMovementBack.push_back(trajBack);

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

                controller->moveLeg(5, trajBackToInitial, speedo);
                controller->moveLeg(1, trajBackToInitial, speedo);
                controller->moveLeg(4, trajBackToInitial, speedo);
                controller->moveLeg(2, trajBackToInitial, speedo);
                controller->moveLeg(3, trajBackToInitial, speedo);

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


//    int position = 0;
//    float_type readAngle[3];
//    float_type offset = 0.10;

////    float_type readkat = 0;
////    vector<float_type> readkatLeg;

////    float_type momenty[] = {0, 0, 0};
//    vector <float_type> motorSpeed;
//    for(int i = 0; i < 18; i++ ){
//        motorSpeed.push_back(5);
//    }

//    Board *robot = createBoardDynamixel();

//    robot->setSpeed(motorSpeed);

//    while(true)
//    {
//        if(robot->readContact(1))
//        {
//            cout << "JEST" << endl;
//        }
//        else
//            cout << "NIE MA" << endl;

//        usleep(1000000);
//    }


//    std::vector<float_type> position1_0;
//    std::vector<float_type> position1_1;
//    std::vector<float_type> position1_2;

//    std::vector<float_type> position2_0;
//    std::vector<float_type> position2_1;
//    std::vector<float_type> position2_2;

//    std::vector<float_type> position3_0;
//    std::vector<float_type> position3_1;
//    std::vector<float_type> position3_2;

//    //INITIAL POSITION//
//    position1_0.push_back((30*M_PI)/180);
//    position1_0.push_back((24*M_PI)/180);
//    position1_0.push_back((114*M_PI)/180);

//    position1_1.push_back((0*M_PI)/180);
//    position1_1.push_back((24*M_PI)/180);
//    position1_1.push_back((114*M_PI)/180);

//    position1_2.push_back((-30*M_PI)/180);
//    position1_2.push_back((24*M_PI)/180);
//    position1_2.push_back((114*M_PI)/180);

//    //3 LEGS UP//
//    position2_0.push_back((30*M_PI)/180);
//    position2_0.push_back((44*M_PI)/180);
//    position2_0.push_back((114*M_PI)/180);

//    position2_1.push_back((0*M_PI)/180);
//    position2_1.push_back((44*M_PI)/180);
//    position2_1.push_back((114*M_PI)/180);

//    position2_2.push_back((-30*M_PI)/180);
//    position2_2.push_back((44*M_PI)/180);
//    position2_2.push_back((114*M_PI)/180);

//    //2 LEGS BACK (BOARD FORWARD)//
//    position3_0.push_back((10*M_PI)/180);
//    position3_0.push_back((24*M_PI)/180);
//    position3_0.push_back((114*M_PI)/180);

//    position3_1.push_back((-20*M_PI)/180);
//    position3_1.push_back((24*M_PI)/180);
//    position3_1.push_back((114*M_PI)/180);

//    position3_2.push_back((-50*M_PI)/180);
//    position3_2.push_back((24*M_PI)/180);
//    position3_2.push_back((114*M_PI)/180);


//    robot->setPosition(0, position1_0);
//    robot->setPosition(1, position1_1);
//    robot->setPosition(2, position1_2);
//    robot->setPosition(3, position1_2);
//    robot->setPosition(4, position1_1);
//    robot->setPosition(5, position1_0);


//    while(true)
//        {


//    while(position == 0)
//    {
//        robot->readPosition(5, 0, readAngle[0], true);
//        robot->readPosition(5, 1, readAngle[1], true);
//        robot->readPosition(5, 2, readAngle[2], true);

////        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
////        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
////        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;

//        if((abs(readAngle[0] - position1_0[0]) < offset) && (abs(readAngle[1] - position1_0[1]) < offset) && (abs(readAngle[2] - position1_0[2]) < offset) )
//        {
//            position = 1;
//            robot->setPosition(0, position2_0);
//            robot->setPosition(4, position2_1);
//            robot->setPosition(2, position2_2);
//        }
//    }



//    while(position == 1)
//    {
//        robot->readPosition(0, 0, readAngle[0], true);
//        robot->readPosition(0, 1, readAngle[1], true);
//        robot->readPosition(0, 2, readAngle[2], true);

////        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
////        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
////        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;

//        if((abs(readAngle[0] - position2_0[0]) < offset) && (abs(readAngle[1] - position2_0[1]) < offset) && (abs(readAngle[2] - position2_0[2]) < offset) )
//        {
//            position = 2;
//            robot->setPosition(5, position3_0);
//            robot->setPosition(1, position3_1);
//            robot->setPosition(3, position3_2);
//        }
//    }

//    while(position == 2)
//    {
//        robot->readPosition(5, 0, readAngle[0], true);
//        robot->readPosition(5, 1, readAngle[1], true);
//        robot->readPosition(5, 2, readAngle[2], true);

////        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
////        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
////        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;

//        if((abs(readAngle[0] - position3_0[0]) < offset) && (abs(readAngle[1] - position3_0[1]) < offset) && (abs(readAngle[2] - position3_0[2]) < offset) )
//        {
//            position = 3;
//            robot->setPosition(0, position1_0);
//            robot->setPosition(4, position1_1);
//            robot->setPosition(2, position1_2);
//        }
//    }

//    while(position == 3)
//    {
//        robot->readPosition(0, 0, readAngle[0], true);
//        robot->readPosition(0, 1, readAngle[1], true);
//        robot->readPosition(0, 2, readAngle[2], true);

////        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
////        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
////        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;

//        if((abs(readAngle[0] - position1_0[0]) < offset) && (abs(readAngle[1] - position1_0[1]) < offset) && (abs(readAngle[2] - position1_0[2]) < offset) )
//        {
//            position = 4;
//            robot->setPosition(5, position2_0);
//            robot->setPosition(1, position2_1);
//            robot->setPosition(3, position2_2);
//        }
//    }

//    while(position == 4)
//    {
//        robot->readPosition(5, 0, readAngle[0], true);
//        robot->readPosition(5, 1, readAngle[1], true);
//        robot->readPosition(5, 2, readAngle[2], true);

////        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
////        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
////        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;

//        if((abs(readAngle[0] - position2_0[0]) < offset) && (abs(readAngle[1] - position2_0[1]) < offset) && (abs(readAngle[2] - position2_0[2]) < offset) )
//        {
//            position = 5;
//            robot->setPosition(0, position3_0);
//            robot->setPosition(4, position3_1);
//            robot->setPosition(2, position3_2);
//        }
//    }

//    while(position == 5)
//    {
//        robot->readPosition(0, 0, readAngle[0], true);
//        robot->readPosition(0, 1, readAngle[1], true);
//        robot->readPosition(0, 2, readAngle[2], true);

////        cout << "servo 0: " << readAngle[0]*180/M_PI << endl;
////        cout << "servo 1: " << readAngle[1]*180/M_PI << endl;
////        cout << "servo 2: " << readAngle[2]*180/M_PI << endl;

//        if((abs(readAngle[0] - position3_0[0]) < offset) && (abs(readAngle[1] - position3_0[1]) < offset) && (abs(readAngle[2] - position3_0[2]) < offset) )
//        {
//            position = 0;
//            robot->setPosition(5, position1_0);
//            robot->setPosition(1, position1_1);
//            robot->setPosition(3, position1_2);
//        }
//    }



//    }

    return 0;
}
