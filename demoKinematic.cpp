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
		std::cout << "Init Vector of displacements and joint angles:\n" << std::endl;
		std::vector<float_type> configuration;
		configuration.push_back(0);
		configuration.push_back(-M_PI*24/180);
        configuration.push_back(M_PI*114/180);
		std::cout << configuration[0] << " " << configuration[1] << " " << configuration[2] << std::endl;
		std::cout << "\nFORWARD KINEMATICS\n" << std::endl;
		std::cout << "Matrix of the position, orientation:\n" << std::endl;
		Mat34 pose;
		pose = demoKine->forwardKinematic(configuration);
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				std::cout<<pose(i, j)<<" ";
			}
			std::cout << "\n";
		}
		std::cout << "\nINVERSE KINEMATICS\n" << std::endl;
		std::vector<float_type> tmp = demoKine->inverseKinematic(pose);
		std::cout << "Vector of displacements and joint angles:\n" << std::endl;
		std::cout << tmp[0] <<" "<<tmp[1]<<" "<<tmp[2]<< std::endl;
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
