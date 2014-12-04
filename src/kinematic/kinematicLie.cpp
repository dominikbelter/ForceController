//Norbert Werblinski
//Adam Czeszejkowski

#include "../include/kinematic/kinematicLie.h"
#include <iostream>


using namespace controller;

/// A single instance of BoardDynamixel
KinematicLie::Ptr kinematicLie;

KinematicLie::KinematicLie(void) : Kinematic("Kinematic Lie", TYPE_LIE) {
}
KinematicLie::KinematicLie(std::string configFilename) : Kinematic(configFilename, "Kienamtic Lie", TYPE_LIE){

	std::string filename = "../../resources/" + configFilename;
	conf.LoadFile(filename.c_str());
	if (conf.FirstChildElement() == nullptr)
		std::cout << "unable to load Kinematic config file.\n";
	else {
		tinyxml2::XMLNode * pRoot = conf.FirstChildElement("parameters");
		float_type val;
		linksNo = std::stoi(conf.FirstChildElement("conf")->FirstChildElement("linksNo")->GetText());
		jointsNo = std::stoi(conf.FirstChildElement("conf")->FirstChildElement("jointsNo")->GetText());
		//ksi = new std::vector<float_type>[linksNo];
		for (int i = 0; i < linksNo; i++)
		{
			std::string tmp = "Joint" + std::to_string(i);
			pElement = pRoot->FirstChildElement(tmp.c_str());
			pListElement = pElement->FirstChildElement("value");
			std::vector<float_type> tmpV;
			while (pListElement != nullptr)
			{
				parameters = pListElement->QueryDoubleText(&val);
				pListElement = pListElement->NextSiblingElement("value");
				tmpV.push_back(val);
			}
			ksi.push_back(tmpV);
		}
		pElement = pRoot->FirstChildElement("g0");
		pListElement = pElement->FirstChildElement("value");
		while (pListElement != nullptr)
		{
			parameters = pListElement->QueryDoubleText(&val);
			pListElement = pListElement->NextSiblingElement("value");
			g0.push_back(val);
		}
		////////////////////////////////////////////////test/////////////////////////////////////////////////////////
		//for (int i = 0; i < linksNo; i++)
		//{
		//std::cout << "Joint" + std::to_string(i) + ": ";
		//for (int j = 0; j < 6; j++)
		//{
		//std::cout << ksi[i][j] << " ";
		//}
		//std::cout << std::endl;
		//}
		//std::cout << "d0: ";
		//for (int j = 0; j < 3; j++)
		//{
		//std::cout << g0[j] << " ";
		//}
		//std::cout << std::endl;	
		}
}


KinematicLie::~KinematicLie(void) {
}

/*Compute forward kinematic, default (-1) -- the last joint
*temporary solution
*!!!!WORKS ONLY FOR Messor 2!!!
*/
Mat34 KinematicLie::forwardKinematic(const std::vector<float_type>& configuration, unsigned int linkNo){
	Mat34 fkmatrix;
	fkmatrix.affine();
	Eigen::Matrix4d e1, e2, e3, g0;
	float_type O1 = configuration[0];
	float_type O2 = configuration[1];
	float_type O3 = configuration[2];
	e1 << cos(O1), -sin(O1), 0, 0, sin(O1), cos(O1), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
	e2 << cos(O2), 0, sin(O2), 0.05 * (1 - cos(O2)), 0, 1, 0, 0, -sin(O2), 0, cos(O2), 0.05 * sin(O2), 0, 0, 0, 1;
	e3 << cos(O3), 0, sin(O3), 0.17*(1 - cos(O3)), 0, 1, 0, 0, -sin(O3), 0, cos(O3), 0.17*sin(O3), 0, 0, 0, 1;
	g0 << 1, 0, 0 ,0.345, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
	fkmatrix = e1*e2*e3*g0;
	return fkmatrix;
}

/*Compute inverse kinematic, default (-1) -- the last joint
*!!!!WORKS ONLY FOR Messor 2!!!
*
*/
std::vector<float_type> KinematicLie::inverseKinematic(const Mat34& linkPose, unsigned int linkNo){
	std::vector<float_type> configVector;
	std::vector<float_type> L; // lenhgts of links
	L.push_back(ksi[1][2]);
	L.push_back(ksi[2][2]-ksi[1][2]);
	L.push_back(g0[0] - ksi[2][2]);
	float_type x = linkPose(0, 3);
	float_type y = linkPose(1, 3);
	float_type z = linkPose(2, 3);
	configVector.push_back(atan2(y, x));//theta0
	switch (linkNo)
	{
	case 1:
		{
			break;
		}
	case 2:
		{
			  configVector.push_back(-atan2(z, x - L[0]));//theta1
			break;
		}
	default:
		{
			float_type l2 = (pow(x - L[0], 2) + pow(z, 2));
			float_type l = sqrt(l2);
			float_type B = atan2(z, x - L[0]);
			float_type g = (pow(L[1], 2) + l2 - pow(L[2], 2)) / (2 * L[1] * l);
			if (g > 1) g = 1;
			else if (g < -1) g = -1;
			float_type Y = -acos(g);
			configVector.push_back(-B + Y);//theta1
			float_type p = (l2 - pow(L[1], 2) - pow(L[2], 2)) / (2 * L[1] * L[2]);
			if (p > 1) p = 1;
			else if (p < -1) p = -1;
			configVector.push_back(acos(p));//theta2
			break;
		}
	}	
	return configVector;
}

/// Return set of link's poses
std::vector<Mat34> KinematicLie::getState(const std::vector<float_type>& configuration){
	std::vector<Mat34> linksPoses;
	for (int i = 0; i < linksNo; i++)
	{
		linksPoses.push_back(forwardKinematic(configuration, i));
	}
	return linksPoses;
}

controller::Kinematic* controller::createKinematicLie(void) {
    kinematicLie.reset(new KinematicLie());
    return kinematicLie.get();
}

controller::Kinematic* controller::createKinematicLie(std::string filename) {
    kinematicLie.reset(new KinematicLie(filename));
    return kinematicLie.get();
}
