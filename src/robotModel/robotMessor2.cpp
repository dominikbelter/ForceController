#include "../include/robotModel/robotMessor2.h"
#include <iostream>
#include "../include/legModel/insectLeg.h"
#include <Eigen/Dense>

using namespace controller;
using namespace std;

RobotMessor::Ptr robotmessor;

RobotMessor::RobotMessor(void) : Robot("Type Messor", TYPE_MESSOR2)
{
}


RobotMessor::~RobotMessor(void)
{

}

///Compute configuration of the leg for the reference motion of the platform
/**
* @param motion - specified motion
* @return reference values for servomotors
*/
std::vector<float_type> RobotMessor::computeLegConfiguration(int legNo, const Mat34 bodyMotion, std::vector<float_type> startConfiguration) {
    Mat34 newLegMountPoint = bodyMotion * legMountPoints[legNo];
    Mat34 currentFootPosition;

    int side = (legNo<3) ? 0 : 1;
    currentFootPosition = legMountPoints[legNo] * legModel->forwardKinematic(startConfiguration, 3, side).matrix();

    Mat34 footInMount;
    footInMount.matrix() = newLegMountPoint.matrix().inverse() * currentFootPosition.matrix();

    std::vector<float_type> conf;
    conf = legModel->inverseKinematic(footInMount, 3, side);
    return conf;
}


std::vector<float_type> RobotMessor::fluentPlatform(Mat34 asked, int legNo)
{

    int side = (legNo<3) ? 0 : 1;
    Mat34 initial(Mat34::Identity());
    Mat34 global(Mat34::Identity());
    global(0,3) = 10;
    global(1,3) = 10;
    global(2,3) = 10;

    std::vector<float_type> conf, confObserve, confGtoX, confGtoR, confAsked;
    conf = computeLegConfiguration(legNo, initial, configurationStart);             //calculate initial configuration
    confObserve = computeLegConfiguration(legNo, global, configurationStart);
    initial = legModel->forwardKinematic(conf, 3, side);

    Mat34 currentFootPosition;
    //currentFootPosition = legMountPoints[legNo] * legModel->forwardKinematic(configurationStart, 3, side);//position Mat at the start
    currentFootPosition = legMountPoints[legNo] * initial;

    Mat34 GtoX, GtoR;
    //z obserwatora konfiguracja dla stopy
    confGtoX = computeLegConfiguration(legNo, currentFootPosition, confObserve);
    GtoX = legModel->forwardKinematic(confGtoX,3, side);

    //z obserwatora patrzymy na srodek platformy
    confGtoR =  computeLegConfiguration(legNo, initial, confObserve);
    GtoR = legModel->forwardKinematic(confGtoR, 3, side);

    //z platformy patrzymy na nowa pozycje platofrmy
    confAsked = computeLegConfiguration(legNo, asked, configurationStart);
    asked = legModel->forwardKinematic(confAsked, 3, side);

    std::vector<float_type> result;
    asked = GtoX * GtoR.inverse() * asked.inverse();
    result = computeLegConfiguration(legNo, asked, configurationStart);

    return result;
}



Mat34 RobotMessor::legCPos(std::vector<float_type> configuration, int legNo)
{
    Mat34 currentFootPosition;
    Mat34 startFootPosition;
    Mat34 returnFootPosition;
    Mat34 temp;

    int side = (legNo<3) ? 0 : 1;

    startFootPosition.matrix() = legMountPoints[legNo] * legModel->forwardKinematic(configurationStart, 3, side).matrix();
    currentFootPosition.matrix() = legModel->forwardKinematic(configuration, 3, side).matrix();

    for(int i=0;i<4;i++)
    {
        for(int j=0;j<3;j++)
        {
            currentFootPosition(i,j) = startFootPosition(i,j);
        }
        cout << endl;
    }

    temp.matrix() = startFootPosition.matrix() * currentFootPosition.matrix().inverse();

    returnFootPosition.matrix() = temp.matrix() * legMountPoints[legNo].matrix().inverse();

    return returnFootPosition;
}

Mat34 RobotMessor::robotToFootTransformation(const Mat34& traj, int legNo)
{
    Mat34 returnFootPosition;
    Mat34 currentFootPosition;
    Mat34 startFootPosition;
    Mat34 temp;

    int side = (legNo<3) ? 0 : 1;

    startFootPosition.matrix() = legMountPoints[legNo] * legModel->forwardKinematic(configurationStart, 3, side).matrix();
    currentFootPosition = legMountPoints[legNo].inverse() * traj ;

    temp.matrix() = startFootPosition.matrix() * currentFootPosition.matrix().inverse();

    returnFootPosition.matrix() = temp.matrix() * legMountPoints[legNo].matrix().inverse();

    return returnFootPosition;
}

///Compute configuration of the robot for the reference motion
std::vector<float_type> RobotMessor::movePlatform(const Mat34& motion)
{
    std::vector<float_type> conf, conf2;

    for (int i = 0; i<legsNo; i++){
        conf2 = computeLegConfiguration(i, motion, configurationStart);
        conf.reserve( conf.size() + conf2.size() );
        conf.insert(conf.end(), conf2.begin(), conf2.end());
	}
    //-------------------------------------------
	return conf;
}

///Compute configuration of the robot for the reference motion (each foot generates separate motion)
std::vector<float_type> RobotMessor::movePlatform(const std::vector<Mat34>& motion)
{
    std::vector<float_type> conf, conf2;

    if (motion.size()!=legsNo){
        std::cout << "movePlatform: Incorrect number of orders\n";
    }

    for (int i = 0; i<legsNo; i++){
        conf2 = computeLegConfiguration(i, motion[i], configurationStart);
        conf.reserve( conf.size() + conf2.size() );
        conf.insert(conf.end(), conf2.begin(), conf2.end());
    }
    return conf;
}

std::vector<float_type> RobotMessor::moveLeg(unsigned char legNo, const Mat34& trajectory)
{
    std::vector<float_type> conf;

    conf = computeLegConfiguration(legNo, trajectory, configurationStart);

    //-------------------------------------------
    return conf;

}

///Compute configuration of the robot for the reference motion (in relation to neutral pose)
std::vector<float_type> RobotMessor::movePlatformNeutral(const Mat34 motion)
{
    std::vector<float_type> conf, conf2;

    for (int i = 0; i<legsNo; i++){
        conf2 = computeLegConfiguration(i, motion, configurationStart);
        conf.reserve( conf.size() + conf2.size() );
        conf.insert(conf.end(), conf2.begin(), conf2.end());
    }
    //-------------------------------------------
    return conf;
}

 /// new method: computes forward kinematics for each leg and returns position of each link of the robot (body is [0,0,0]^T)
std::vector<Mat34> RobotMessor::conputeLinksPosition(std::vector<float_type> configuration)
{

	std::vector<Mat34> linksPos;
	std::vector<float_type> conf;

    //-----------------------------------------
    for (int h = 0; h<configuration.size(); h=h + 3)
	{
		for (int j = 0; j<3; j++)
		{
			conf.push_back(configuration[j + h]);
		}

         if (h<9)
         {
            for (int i = 0; i<3; i++)
            {
                linksPos.push_back(legMountPoints[h / 3] * legModel->forwardKinematic(conf, i,0));
            }
             }
         else
         {
             for (int i = 0; i<3; i++)
             {
                 linksPos.push_back(legMountPoints[h / 3] * legModel->forwardKinematic(conf, i,1));
             }

         }
         if (h<9){
        linksPos.push_back(legMountPoints[h / 3] *legModel->forwardKinematic(conf, -1,0));
}
         else
             linksPos.push_back(legMountPoints[h / 3] * legModel->forwardKinematic(conf, -1,1));

        for (int k = 0; k<3; k++)
		{
			conf.pop_back();
		}
	}
	//-------------------------------------------

    return linksPos;
}


///Compute force in each joint of the legs, input configuration of the robot
std::vector<float_type> RobotMessor::computeCompliance(const std::vector<float_type> configuration)
{
         TorqueForce TF1,TF2,TF3,TF4,TF5,TF6;
         //Coefficient matrix of force and torque equations
         typedef Eigen::Matrix<float_type,6,3> Mat63;
         typedef Eigen::Matrix<float_type,6,1> Mat16;
         Mat16 B,B2,x2,x,Fx,Fy,Fz;
         Mat63 A,A2;
         std::vector<Mat34> pos,pos2;
         std::vector<float_type> l1,l2,l3,l4,l5,l6,FZ,FX,FY;
         float m=2,g=9.81,F=m*g;
         std::vector<float_type> torque1,torque2,torque3,torque4,torque5,torque6,TORQUE,compliance;

        pos=conputeLinksPosition(configuration);

        for(int i=3; i<pos.size(); i+=4){
          pos2.push_back(pos[i]);
        }

        for(int i=0;i<3;i++)
        {
            l1.push_back(pos2[0](i,3));
            l2.push_back(pos2[1](i,3));
            l3.push_back(pos2[2](i,3));
            l4.push_back(pos2[3](i,3));
            l5.push_back(pos2[4](i,3));
            l6.push_back(pos2[5](i,3));
        }

        //equation of Torque y
        A(0,0)=((l1[0]*(l1[2]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2))))+(l1[2]*(l1[0]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2)))));
        A2(0,1)=((l2[0]*(l2[2]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2))))+(l2[2]*(l2[0]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2)))));
        A(0,2)=((l3[0]*(l3[2]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2))))+(l3[2]*(l3[0]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2)))));

        A2(0,0)=((l4[0]*(l4[2]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2))))+(l4[2]*(l4[0]/sqrt(pow(l4[0],2)-pow(l4[1],2)+pow(l4[2],2)))));
        A(0,1)=((l5[0]*(l5[2]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2))))+(l5[2]*(l5[0]/sqrt(pow(l5[0],2)-pow(l5[1],2)+pow(l5[2],2)))));
        A2(0,2)=((l6[0]*(l6[2]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2))))+(l6[2]*(l6[0]/sqrt(pow(l6[0],2)-pow(l6[1],2)+pow(l6[2],2)))));

        //equation of Torque x
        A(1,0)=((l1[1]*(l1[2]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2))))+(l1[2]*(l1[1]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2)))));
        A2(1,1)=((l2[1]*(l2[2]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2))))+(l2[2]*(l2[1]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2)))));
        A(1,2)=((l3[1]*(l3[2]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2))))+(l3[2]*(l3[1]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2)))));

        A2(1,0)=((l4[1]*(l4[2]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2))))+(l4[2]*(l4[1]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2)))));
        A(1,1)=((l5[1]*(l5[2]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2))))+(l5[2]*(l5[1]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2)))));
        A2(1,2)=((l6[1]*(l6[2]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2))))+(l6[2]*(l6[1]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2)))));

        //equation of Torque z
        A(2,0)=((l1[0]*(l1[1]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2))))+(l1[1]*(l1[0]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2)))));
        A2(2,1)=((l2[0]*(l2[1]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2))))+(l2[1]*(l2[0]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2)))));
        A(2,2)=((l3[0]*(l3[1]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2))))+(l3[1]*(l3[0]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2)))));

        A2(2,0)=((l4[0]*(l4[1]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2))))+(l4[1]*(l4[0]/sqrt(pow(l4[0],2)-pow(l4[1],2)+pow(l4[2],2)))));
        A(2,1)=((l5[0]*(l5[1]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2))))+(l5[1]*(l5[0]/sqrt(pow(l5[0],2)-pow(l5[1],2)+pow(l5[2],2)))));
        A2(2,2)=((l6[0]*(l6[1]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2))))+(l6[1]*(l6[0]/sqrt(pow(l6[0],2)-pow(l6[1],2)+pow(l6[2],2)))));

        //equation of Force z
        A(3,0)=l1[2]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2));
        A2(3,1)=l2[2]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2));
        A(3,2)=l3[2]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2));

        A2(3,0)=l4[2]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2));
        A(3,1)=l5[2]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2));
        A2(3,2)=l6[2]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2));

        //equation of Force x
        A(4,0)=l1[0]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2));
        A2(4,1)=l2[0]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2));
        A(4,2)=l3[0]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2));

        A2(4,0)=l4[0]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2));
        A(4,1)=l5[0]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2));
        A2(4,2)=l6[0]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2));

        //equation of Force y
        A(5,0)=l1[1]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2));
        A2(5,1)=l2[1]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2));
        A(5,2)=l3[1]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2));

        A2(5,0)=l4[1]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2));
        A(5,1)=l5[1]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2));
        A2(5,2)=l6[1]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2));

        //vectors of results
        B(0,0)=0;
        B(1,0)=0;
        B(2,0)=0;
        B(3,0)=F/2;
        B(4,0)=0;
        B(5,0)=0;

        B2(0,0)=0;
        B2(1,0)=0;
        B2(2,0)=0;
        B2(3,0)=F/2;
        B2(4,0)=0;
        B2(5,0)=0;

        //solution (x -> f1,f2,...,f6)
        x = A.colPivHouseholderQr().solve(B);
        x2 = A2.colPivHouseholderQr().solve(B2);


        //projection of forces
        Fx(0,0)=x(0,0)*l1[0]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2));
        Fx(1,0)=x2(1,0)*l2[0]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2));
        Fx(2,0)=x(2,0)*l3[0]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2));

        Fx(3,0)=x2(0,0)*l4[0]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2));
        Fx(4,0)=x(1,0)*l5[0]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2));
        Fx(5,0)=x2(2,0)*l6[0]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2));

        Fy(0,0)=x(0,0)*l1[1]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2));
        Fy(1,0)=x2(1,0)*l2[1]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2));
        Fy(2,0)=x(2,0)*l3[1]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2));

        Fy(3,0)= x2(0,0)*l4[1]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2));
        Fy(4,0)=x(1,0)*l5[1]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2));
        Fy(5,0)=x2(2,0)*l6[1]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2));

        Fz(0,0)=x(0,0)*l1[2]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2));
        Fz(1,0)=x2(1,0)*l2[2]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2));
        Fz(2,0)=x(2,0)*l3[2]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2));

        Fz(3,0)=x2(0,0)*l4[2]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2));
        Fz(4,0)=x(1,0)*l5[2]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2));
        Fz(5,0)=x2(2,0)*l6[2]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2));


        for(int i=0;i<6;i++)
         {
         FZ.push_back(Fz(i,0));
         FX.push_back(Fx(i,0));
         FY.push_back(Fy(i,0));
         }

         TF1.force.x()=0;
         TF1.force.y()=0;
         TF1.force.z()=Fz(0,0);

         TF2.force.x()=0;
         TF2.force.y()=0;
         TF2.force.z()=Fz(1,0);

         TF3.force.x()=0;
         TF3.force.y()=0;
         TF3.force.z()=Fz(2,0);

         TF4.force.x()=0;
         TF4.force.y()=0;
         TF4.force.z()=Fz(3,0);

         TF5.force.x()=0;
         TF5.force.y()=0;
         TF5.force.z()=Fz(4,0);

         TF6.force.x()=0;
         TF6.force.y()=0;
         TF6.force.z()=Fz(5,0);

         std::vector<float_type> c1,c2,c3,c4,c5,c6;

        for(int i=0; i<3;i++)
            c1.push_back(configuration[i]);


        for(int i=3; i<6;i++)
            c2.push_back(configuration[i]);

        for(int i=6; i<9;i++)
            c3.push_back(configuration[i]);

        for(int i=9; i<12;i++)
            c4.push_back(configuration[i]);

        for(int i=12; i<15;i++)
            c5.push_back(configuration[i]);

        for(int i=15; i<18;i++)
            c6.push_back(configuration[i]);

         //Torque
        torque1=legModel->computLoad(TF1.force,c1,1);
        torque2=legModel->computLoad(TF2.force,c2,1);
        torque3=legModel->computLoad(TF3.force,c3,1);
        torque4=legModel->computLoad(TF4.force,c4,0);
        torque5=legModel->computLoad(TF5.force,c5,0);
        torque6=legModel->computLoad(TF6.force,c6,0);

        TORQUE.insert(TORQUE.end(),torque1.begin(),torque1.end());
        TORQUE.insert(TORQUE.end(),torque2.begin(),torque2.end());
        TORQUE.insert(TORQUE.end(),torque3.begin(),torque3.end());
        TORQUE.insert(TORQUE.end(),torque4.begin(),torque4.end());
        TORQUE.insert(TORQUE.end(),torque5.begin(),torque5.end());
        TORQUE.insert(TORQUE.end(),torque6.begin(),torque6.end());

        for(int i=0;i<18;i++)
         {
             compliance.push_back(TORQUE[i]*0.59*3);
             compliance[i]=sqrt(compliance[i]*compliance[i]);

             if(compliance[i]>1)
                       compliance[i]=1;

         }


        return compliance;

}

Mat34 RobotMessor::getLegPosFromRobot(Mat34 robotTraj, int legNo)
{
    Mat34 localTraj = robotTraj * legMountPoints[legNo];
//    Mat34 newLegMountPoint = localTraj * legMountPoints[legNo];
//    Mat34 currentFootPosition;

//    int side = (legNo<3) ? 0 : 1;
//    currentFootPosition = legMountPoints[legNo] * legModel->forwardKinematic(configurationStart, 3, side).matrix();


//    Mat34 footInMount;
//    footInMount.matrix() = newLegMountPoint.matrix().inverse() * currentFootPosition.matrix();

    cout << "wspolrzedne nogi" << endl;
    for(int x=0;x<3;x++)
    {
        cout << localTraj(x,3) << endl;
    }
    return localTraj;
}

 controller::Robot* controller::createRobotMessor(void) {
     robotmessor.reset(new RobotMessor());
     return robotmessor.get();
 }

 controller::Robot* controller::createRobotMessor(std::string filename) {
     robotmessor.reset(new RobotMessor(filename));
     return robotmessor.get();
 }
