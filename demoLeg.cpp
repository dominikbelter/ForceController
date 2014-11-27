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

int main( int argc, const char** argv )
{
  try
  {
		cout << "DEMO STEROWNIKA NOGI" << endl;

    Leg* legModel;
    legModel = createInsectLeg("../resources/legModel.xml");
		cout << "Leg type: " << legModel->getName() << endl << endl;

		Mat34 linkPose;
		linkPose(0, 0) = 1; linkPose(0, 1) = 0; linkPose(0, 2) = 0; linkPose(0, 3) = 0.1;
		linkPose(1, 0) = 0; linkPose(1, 1) = 1; linkPose(1, 2) = 0; linkPose(1, 3) = 0.1;
		linkPose(2, 0) = 0; linkPose(2, 1) = 0; linkPose(2, 2) = 1; linkPose(2, 3) = 0.1;

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
		unsigned int linksNo = 3;//legModel->linksNo;
		config.push_back(0);
		config.push_back(0);
		config.push_back(0);
		config = legModel->inverseKinematic(linkPose, linksNo);
		cout << config[0] << ", " << config[1] << ", " << config[2];

    getchar();
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
    return 1;
  }
  return 0;
}
