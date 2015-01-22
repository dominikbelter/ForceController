#include "../include/robotModel/robotMessor2.h"
#include <iostream>
#include "../include/legModel/insectLeg.h"
#include <Eigen/Dense>

using namespace controller;





RobotMessor::Ptr robotmessor;

RobotMessor::RobotMessor(void) : Robot("Type Messor", TYPE_MESSOR2)
{
	//Translation for each Leg

	Leg0 = createInsectLeg("../resources/legModel.xml");

	width_max = 0.1025; ///distance from center to middle leg
	width_min = 0.052; /// distance from x to front leg
	length = 0.12; ///distance front legs from x

	L0.setIdentity();
	L0(0, 3) = width_min;
	L0(1, 3) = length;
	L0(2, 3) = 0;

	L1.setIdentity();
	L1(0, 3) = width_max;
	L1(1, 3) = 0;
	L1(2, 3) = 0;

	L2.setIdentity();
	L2(0, 3) = width_min;
	L2(1, 3) = -length;
	L2(2, 3) = 0;

	L3.setIdentity();
	L3(0, 3) = -width_min;
	L3(1, 3) = -length;
	L3(2, 3) = 0;

	L4.setIdentity();
	L4(0, 3) = -width_max;
	L4(1, 3) = 0;
	L4(2, 3) = 0;

	L5.setIdentity();
	L5(0, 3) = -width_min;
	L5(1, 3) = length;
	L5(2, 3) = 0;

	L_all.push_back(L0);
	L_all.push_back(L1);
	L_all.push_back(L2);
	L_all.push_back(L3);
	L_all.push_back(L4);
	L_all.push_back(L5);

	OldMotion.setIdentity();
	OldMotion(0, 3) = 0;
	OldMotion(1, 3) = 0;
    OldMotion(2, 3) = 0.0;

    NeutralMotion.setIdentity();
    NeutralMotion(0, 3) = 0;
    NeutralMotion(1, 3) = 0;
    NeutralMotion(2, 3) = 0.12;


    for (int i = 0; i<6; i++)
        {
            configurationstart.push_back(0);
            configurationstart.push_back(24*3.14/180);
            configurationstart.push_back(-114*3.14/180);
        }
        configurationact=configurationstart;

}




RobotMessor::~RobotMessor(void)
{

}

///Compute configuration of the robot for the reference motion
std::vector<float_type> RobotMessor::movePlatform(const Mat34& motion)
{
    std::vector<float_type> conf, conf2;
    Mat34 x, y, s;


    //newmotion = OldMotion*motion;

	//-----------------------------------------
    using namespace std;

	for (int i = 0; i<6; i++)
	{

        x = motion * L_all[i];
        //cout<<"x\n"<<x.matrix()<<endl;

        s = OldMotion * L_all[i];

        if (i < 3)
        {
            s.matrix() *= Leg0 ->forwardKinematic(configurationact, 3, 0).matrix();
            //cout<<"fk\n"<<Leg0 ->forwardKinematic(configurationact, 3, 0).matrix()<<endl;
        }
        else
        {
            s.matrix() *= Leg0 ->forwardKinematic(configurationact, 3, 1).matrix();
            //cout<<"fk\n"<<Leg0 ->forwardKinematic(configurationact, 3, 1).matrix()<<endl;
        }

       // cout<<"s\n"<<s.matrix()<<endl;

        y.matrix() = x.matrix().inverse() * s.matrix();
       // cout<<"y\n"<<y.matrix()<<endl;



        // actleg.matrix() = actleg.matrix().inverse();

        if (i<3)
        {

        conf2 = Leg0->inverseKinematic(y, 3, 0);
        }
        else
        {
        conf2 = Leg0->inverseKinematic(y, 3, 1);
        }
		conf.push_back(conf2[0]);
		conf.push_back(conf2[1]);
		conf.push_back(conf2[2]);
        /*
        for(int e=0;e<3;e++)
        {
            cout<<conf2[i]<<endl;
        }
        //getchar();
        */
	}
	//-------------------------------------------
    configurationact = conf;
	return conf;
}
		
///Compute configuration of the robot for the reference motion (in relation to neutral pose)
 std::vector<float_type> RobotMessor::movePlatformNeutral(const Mat34 motion)
{
     {
         std::vector<float_type> conf, conf2;
         Mat34 x, y, s;

         //newmotion = OldMotion*motion;
         //-----------------------------------------
         using namespace std;

         for (int i = 0; i<6; i++)
         {

             x = motion * L_all[i];
            // cout<<"x\n"<<x.matrix()<<endl;

             s = NeutralMotion * L_all[i];

             if (i < 3)
             {
                 s.matrix() *= Leg0 ->forwardKinematic(configurationstart, 3, 0).matrix();
                 //cout<<"fk\n"<<Leg0 ->forwardKinematic(configurationstart, 3, 0).matrix()<<endl;
             }
             else
             {
                 s.matrix() *= Leg0 ->forwardKinematic(configurationstart, 3, 1).matrix();
                // cout<<"fk\n"<<Leg0 ->forwardKinematic(configurationstart, 3, 1).matrix()<<endl;
             }

             //cout<<"s\n"<<s.matrix()<<endl;

             y.matrix() = x.matrix().inverse() * s.matrix();
            // cout<<"y\n"<<y.matrix()<<endl;

             if (i<3)
             {

             conf2 = Leg0->inverseKinematic(y, 3, 0);
             }
             else
             {
             conf2 = Leg0->inverseKinematic(y, 3, 1);
             }
             conf.push_back(conf2[0]);
             conf.push_back(conf2[1]);
             conf.push_back(conf2[2]);
             /*
             for(int e=0;e<3;e++)
             {
                 cout<<conf2[i]<<endl;
             }
             */
         }
         return conf;
     }
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
            linksPos.push_back(L_all[h / 3] * Leg0->forwardKinematic(conf, i,0));
		}
         }
         else
         {
             for (int i = 0; i<3; i++)
             {
                 linksPos.push_back(L_all[h / 3] * Leg0->forwardKinematic(conf, i,1));
             }

         }
         if (h<9)

        linksPos.push_back(L_all[h / 3] * Leg0->forwardKinematic(conf, -1,0));
         else
             linksPos.push_back(L_all[h / 3] * Leg0->forwardKinematic(conf, -1,1));

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
         typedef Eigen::Matrix<float_type,6,6> Mat66;
         typedef Eigen::Matrix<float_type,6,1> Mat16;
         Mat16 B,x,Fx,Fy,Fz;
         Mat66 A;
         std::vector<Mat34> pos,pos2;
         std::vector<float_type> l1,l2,l3,l4,l5,l6,FZ,FX,FY;
         float m=2,g=9.81,F=m*g;
         std::vector<float_type> torque1,torque2,torque3,torque4,torque5,torque6,TORQUE,compliance;

        pos=conputeLinksPosition(configuration);

        for(int i=3; i<pos.size(); i+=4)
        {
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



         /*l1.push_back(-0.3);l1.push_back(0.1);l1.push_back(-0.1);
         l2.push_back(-0.35);l2.push_back(0.0);l2.push_back(-0.1);
         l3.push_back(-0.3);l3.push_back(-0.1);l3.push_back(-0.1);
         l4.push_back(0.3);l4.push_back(-0.1);l4.push_back(-0.1);
         l5.push_back(0.35);l5.push_back(0.0);l5.push_back(-0.1);
         l6.push_back(0.3);l6.push_back(0.1);l6.push_back(-0.1);*/

         //equation of Torque y
         A(0,0)=((l1[0]*(l1[2]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2))))+(l1[2]*(l1[0]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2)))));
         A(0,1)=((l2[0]*(l2[2]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2))))+(l2[2]*(l2[0]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2)))));
         A(0,2)=((l3[0]*(l3[2]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2))))+(l3[2]*(l3[0]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2)))));
         A(0,3)=((l4[0]*(l4[2]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2))))+(l4[2]*(l4[0]/sqrt(pow(l4[0],2)-pow(l4[1],2)+pow(l4[2],2)))));
         A(0,4)=((l5[0]*(l5[2]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2))))+(l5[2]*(l5[0]/sqrt(pow(l5[0],2)-pow(l5[1],2)+pow(l5[2],2)))));
         A(0,5)=((l6[0]*(l6[2]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2))))+(l6[2]*(l6[0]/sqrt(pow(l6[0],2)-pow(l6[1],2)+pow(l6[2],2)))));
         //equation of Torque x
         A(1,0)=((l1[1]*(l1[2]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2))))+(l1[2]*(l1[1]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2)))));
         A(1,1)=((l2[1]*(l2[2]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2))))+(l2[2]*(l2[1]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2)))));
         A(1,2)=((l3[1]*(l3[2]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2))))+(l3[2]*(l3[1]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2)))));
         A(1,3)=((l4[1]*(l4[2]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2))))+(l4[2]*(l4[1]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2)))));
         A(1,4)=((l5[1]*(l5[2]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2))))+(l5[2]*(l5[1]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2)))));
         A(1,5)=((l6[1]*(l6[2]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2))))+(l6[2]*(l6[1]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2)))));
         //equation of Torque z
         A(2,0)=((l1[0]*(l1[1]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2))))+(l1[1]*(l1[0]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2)))));
         A(2,1)=((l2[0]*(l2[1]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2))))+(l2[1]*(l2[0]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2)))));
         A(2,2)=((l3[0]*(l3[1]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2))))+(l3[1]*(l3[0]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2)))));
         A(2,3)=((l4[0]*(l4[1]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2))))+(l4[1]*(l4[0]/sqrt(pow(l4[0],2)-pow(l4[1],2)+pow(l4[2],2)))));
         A(2,4)=((l5[0]*(l5[1]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2))))+(l5[1]*(l5[0]/sqrt(pow(l5[0],2)-pow(l5[1],2)+pow(l5[2],2)))));
         A(2,5)=((l6[0]*(l6[1]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2))))+(l6[1]*(l6[0]/sqrt(pow(l6[0],2)-pow(l6[1],2)+pow(l6[2],2)))));
         //equation of Force z
         A(3,0)=l1[2]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2));
         A(3,1)=l2[2]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2));
         A(3,2)=l3[2]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2));
         A(3,3)=l4[2]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2));
         A(3,4)=l5[2]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2));
         A(3,5)=l6[2]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2));
         //equation of Force x
         A(4,0)=l1[0]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2));
         A(4,1)=l2[0]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2));
         A(4,2)=l3[0]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2));
         A(4,3)=l4[0]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2));
         A(4,4)=l5[0]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2));
         A(4,5)=l6[0]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2));
         //equation of Force y
         A(5,0)=l1[1]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2));
         A(5,1)=l2[1]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2));
         A(5,2)=l3[1]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2));
         A(5,3)=l4[1]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2));
         A(5,4)=l5[1]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2));
         A(5,5)=l6[1]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2));

         //vector of results
         B(0,0)=0;
         B(1,0)=0;
         B(2,0)=0;
         B(3,0)=F;
         B(4,0)=0;
         B(5,0)=0;

         //solution (x -> f1,f2,...,f6)
         x = A.colPivHouseholderQr().solve(B);

         //projection of forces
         Fx(0,0)=x(0,0)*l1[0]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2));
         Fx(1,0)=x(1,0)*l2[0]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2));
         Fx(2,0)=x(2,0)*l3[0]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2));
         Fx(3,0)=x(3,0)*l4[0]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2));
         Fx(4,0)=x(4,0)*l5[0]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2));
         Fx(5,0)=x(5,0)*l6[0]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2));

         Fy(0,0)=x(0,0)*l1[1]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2));
         Fy(1,0)=x(1,0)*l2[1]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2));
         Fy(2,0)=x(2,0)*l3[1]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2));
         Fy(3,0)= x(3,0)*l4[1]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2));
         Fy(4,0)=x(4,0)*l5[1]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2));
         Fy(5,0)=x(5,0)*l6[1]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2));

         Fz(0,0)=x(0,0)*l1[2]/sqrt(pow(l1[0],2)+pow(l1[1],2)+pow(l1[2],2));
         Fz(1,0)=x(1,0)*l2[2]/sqrt(pow(l2[0],2)+pow(l2[1],2)+pow(l2[2],2));
         Fz(2,0)=x(2,0)*l3[2]/sqrt(pow(l3[0],2)+pow(l3[1],2)+pow(l3[2],2));
         Fz(3,0)=x(3,0)*l4[2]/sqrt(pow(l4[0],2)+pow(l4[1],2)+pow(l4[2],2));
         Fz(4,0)=x(4,0)*l5[2]/sqrt(pow(l5[0],2)+pow(l5[1],2)+pow(l5[2],2));
         Fz(5,0)=x(5,0)*l6[2]/sqrt(pow(l6[0],2)+pow(l6[1],2)+pow(l6[2],2));

         for(int i=0;i<6;i++)
         {
         FZ.push_back(Fz(i,0));
         FX.push_back(Fx(i,0));
         FY.push_back(Fy(i,0));
         }

         TF1.force.x()=Fx(0,0);
         TF1.force.y()=Fy(0,0);
         TF1.force.z()=Fz(0,0);

         TF2.force.x()=Fx(1,0);
         TF2.force.y()=Fy(1,0);
         TF2.force.z()=Fz(1,0);

         TF3.force.x()=Fx(2,0);
         TF3.force.y()=Fy(2,0);
         TF3.force.z()=Fz(2,0);

         TF4.force.x()=Fx(3,0);
         TF4.force.y()=Fy(3,0);
         TF4.force.z()=Fz(3,0);

         TF5.force.x()=Fx(4,0);
         TF5.force.y()=Fy(4,0);
         TF5.force.z()=Fz(4,0);

         TF6.force.x()=Fx(5,0);
         TF6.force.y()=Fy(5,0);
         TF6.force.z()=Fz(5,0);

         //Torque
        torque1=Leg0->computLoad(TF1.force,configuration);
        torque2=Leg0->computLoad(TF2.force,configuration);
        torque3=Leg0->computLoad(TF3.force,configuration);
        torque4=Leg0->computLoad(TF4.force,configuration);
        torque5=Leg0->computLoad(TF5.force,configuration);
        torque6=Leg0->computLoad(TF6.force,configuration);
        TORQUE.insert(TORQUE.end(),torque1.begin(),torque1.end());
        TORQUE.insert(TORQUE.end(),torque2.begin(),torque2.end());
        TORQUE.insert(TORQUE.end(),torque3.begin(),torque3.end());
        TORQUE.insert(TORQUE.end(),torque4.begin(),torque4.end());
        TORQUE.insert(TORQUE.end(),torque5.begin(),torque5.end());
        TORQUE.insert(TORQUE.end(),torque6.begin(),torque6.end());

        for(int i=0;i<18;i++)
         {

             compliance.push_back(TORQUE[i]*0.25);
             compliance[i]=sqrt(compliance[i]*compliance[i]);

                     if(compliance[i]>1)
                        compliance[i]=1;


         }

        return TORQUE;

}

 controller::Robot* controller::createRobotMessor(void) {
     robotmessor.reset(new RobotMessor());
     return robotmessor.get();
 }

 controller::Robot* controller::createRobotMessor(std::string filename) {
     robotmessor.reset(new RobotMessor(filename));
     return robotmessor.get();
 }
