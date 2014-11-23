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
		for (int i = 0; i < linksNo; i++)
		{
		std::cout << "Joint" + std::to_string(i) + ": ";
		for (int j = 0; j < 6; j++)
		{
		std::cout << ksi[i][j] << " ";
		}
		std::cout << std::endl;
		}
		std::cout << "d0: ";
		for (int j = 0; j < 3; j++)
		{
		std::cout << g0[j] << " ";
		}
		std::cout << std::endl;	
		}
}


KinematicLie::~KinematicLie(void) {
}

/// Compute forward kinematic, default (-1) -- the last joint
Mat34 KinematicLie::forwardKinematic(const std::vector<float_type>& configuration, unsigned int linkNo){
	Mat34 tmp;
	return tmp;
}

/*Compute inverse kinematic, default (-1) -- the last joint
*!!!!WORKS ONLY FOR Messor 2!!!
*
*/
std::vector<float_type> KinematicLie::inverseKinematic(const Mat34& linkPose, unsigned int linkNo){
	std::vector<float_type> tmp;
	switch (linkNo)
	{
	case 1:
		{
			break;
		}
	case 2:
		{
			break;
		}
	case 3:
		{

		}
		default:
		{
			std::vector<float_type> L; // lenhgts of links
			L.push_back(0.05);
			L.push_back(0.12);
			L.push_back(0.175);
			float_type x = linkPose(0, 3);
				   float_type y = linkPose(1, 3);
				   float_type z = linkPose(2, 3);
				   float_type theta0 = atan2(y, x);
				   float_type l2 = (pow(x - L[0], 2) + pow(z, 2));
				   float_type l = sqrt(l2);
				   float_type B = atan2(z, x - L[0]);
				   float_type g = (pow(L[1], 2) + l2 - pow(L[2], 2)) / (2 * L[1] * l);
				   if (g > 1) g = 1;
				   else if (g < -1) g = -1;
				   float_type Y = acos(g);
				   float_type theta1 = -B - Y;
				   float_type p = (l2 - pow(L[1], 2) - pow(L[2], 2)) / (2 * L[1] * L[2]);
				   if (p > 1) p = 1;
				   else if (p < -1) p = -1;
				   float_type theta2 = acos(p);
				   tmp.push_back(theta0);
				   tmp.push_back(theta1);
				   tmp.push_back(theta2);
				   break;
		}
	}	
    return tmp;
}

/// Return set of link's poses
std::vector<Mat34> KinematicLie::getState(const std::vector<float_type>& configuration){
	std::vector<Mat34> tmp;
	return tmp;
}

controller::Kinematic* controller::createKinematicLie(void) {
    kinematicLie.reset(new KinematicLie());
    return kinematicLie.get();
}

controller::Kinematic* controller::createKinematicLie(std::string filename) {
    kinematicLie.reset(new KinematicLie(filename));
    return kinematicLie.get();
}
