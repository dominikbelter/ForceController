#include "include/defs/defs.h"
#include "include/kinematic/kinematic.h"
#include "../include/kinematic/kinematicLie.h"
#include <iostream>
#include <stdio.h>

using namespace std;

int main( int argc, const char** argv )
{
	try {
		controller::Kinematic* demoKine;
		demoKine = createKinematicLie("legModel.xml");
		std::cout << "Kinematic type:" << demoKine->getName() << "\n";
		std::cout << "\nINVERSE KINEMATICS\n" << std::endl;
		std::cout << "Matrix of the position, orientation\n" << std::endl;
		Mat34 pose;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				pose(j, i) = 0;
			}
		}
		pose(0, 0) = 1;
		pose(1, 1) = 1;
		pose(2, 2) = 1;
		pose(3, 3) = 1;
		pose(0, 3) = 0.17;
		pose(1, 3) = 0;
		pose(2, 3) = -0.175;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				std::cout << pose(i, j) << " ";
			}
			std::cout << std::endl;
		}
		std::vector<float_type> tmp = demoKine->inverseKinematic(pose);
		std::cout << "\nVector of displacements and joint angles:\n" << std::endl;
		std::cout << "theta0 theta1 theta2" << std::endl;
		std::cout << tmp[0] <<" "<<tmp[1]<<" "<<tmp[2]<< std::endl;
		system("PAUSE");
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
