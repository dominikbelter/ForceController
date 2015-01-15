/** @file demoLeg.cpp
*
* @author Lukasz Mejlun
* @author Wojciech Nowakowski
*/

#include "include/defs/defs.h"
#include "include/legModel/insectLeg.h"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace controller;

int main( int argc, const char** argv )
{
  try
  {
		cout << "DEMO STEROWNIKA NOGI" << endl;

    Leg* legModel;
		legModel = createInsectLeg("../resources/legModel.xml");
		cout << "Leg type: " << legModel->getName() << endl << endl;

		Mat34 linkPose;
		linkPose(0, 0) = 0; linkPose(0, 1) = 0; linkPose(0, 2) = 1; linkPose(0, 3) = 0.164;
		linkPose(1, 0) = 0; linkPose(1, 1) = 1; linkPose(1, 2) = 0; linkPose(1, 3) = 0.0;
		linkPose(2, 0) = -1; linkPose(2, 1) = 0; linkPose(2, 2) = 0; linkPose(2, 3) = -0.119;

		cout << "Destination Position" << endl;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				cout << linkPose(i, j) << " ";
			}
			cout << endl;
		}

		cout << endl << "Configuration" << endl;
		vector<float_type> config;
		int linksNo = legModel->getLinksNo();
		config = legModel->inverseKinematic(linkPose, linksNo, false);
		cout << config[0] << ", " << config[1] << ", " << config[2] << endl;

		linkPose = legModel->forwardKinematic(config, linksNo, false);
		cout << endl << "Forward Kinematic" << endl;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				cout << linkPose(i, j) << " ";
			}
			cout << endl;
		}

		Vec3 sila(0,3,1);
		vector<float_type> qload;

		qload = legModel->computLoad(sila, config);
		cout << endl << "Computed loads: " << qload[0] << ", " << qload[1] << ", " << qload[2] << endl;

		cout << endl << "If Leg is left: " << endl;

		cout << endl << "Configuration" << endl;
		config = legModel->inverseKinematic(linkPose, linksNo, true);
		cout << config[0] << ", " << config[1] << ", " << config[2] << endl;

		linkPose = legModel->forwardKinematic(config, linksNo, true);
		cout << endl << "Forward Kinematic" << endl;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				cout << linkPose(i, j) << " ";
			}
			cout << endl;
		}

		config[0] = 0;
		config[1] = -24*M_PI/180;
		config[2] = 114*M_PI/180;
		linkPose = legModel->forwardKinematic(config, linksNo, false);
		cout << endl << "Test Inverse Kinematic" << endl;
		cout << "testowa konfiguracja: " << config[0] << ", " << config[1] << ", " << config[2] << endl;
		cout << "   Pozycja" << endl;
		for (int i = 0; i < 3; ++i)
		{
			cout << "   ";
			for (int j = 0; j < 4; ++j)
			{
				cout << linkPose(i, j) << " ";
			}
			cout << endl;
		}
		config = legModel->inverseKinematic(linkPose, linksNo, false);
		cout << "otrzymana konfiguracja: " << config[0] << ", " << config[1] << ", " << config[2] << endl;

    getchar();
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
    return 1;
  }
  return 0;
}
