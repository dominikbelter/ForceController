//Norbert Werblinski
//Adam Czeszejkowski

#include "../include/kinematic/kinematicLie.h"
#include <iostream>


using namespace controller;

/// A single instance of BoardDynamixel
KinematicLie::Ptr kinematicLie;

KinematicLie::KinematicLie(void) : Kinematic("Kinematic Lie", TYPE_LIE) {
}

KinematicLie::~KinematicLie(void) {
}

/// Compute forward kinematic, default (-1) -- the last joint
Mat34 KinematicLie::forwardKinematic(const std::vector<float_type>& configuration, unsigned int linkNo){
	Mat34 tmp;
	return tmp;
}

/// Compute inverse kinematic, default (-1) -- the last joint
/*
*!!!!WORKS ONLY FOR Messor 2!!!
*
*/
std::vector<float_type> KinematicLie::inverseKinematic(const Mat34& linkPose, unsigned int linkNo){
    std::vector<float_type> tmp;
	std::vector<float_type> L; // lenhgts of links
	L.push_back(0.05);
	L.push_back(0.12);
	L.push_back(0.175);
	float_type x = linkPose(0, 3);
	float_type y = linkPose(1, 3);
	float_type z = linkPose(2, 3);
	float_type theta0 = atan2(y, x);
	float_type l2 = (pow(x - L[0],2) + pow(z,2));
	float_type l = sqrt(l2);
	float_type B = atan2(z, x - L[0]);
	float_type g = (pow(L[1], 2) + l2 - pow(L[2], 2)) / (2 * L[1] * l);
	if (g > 1) g = 1;
	else if (g < -1) g = -1;
	float_type Y = acos(g);
	float_type theta1=-B-Y;
	float_type p = (l2 - pow(L[1], 2) - pow(L[2], 2)) / (2 * L[1] * L[2]);
	if (p > 1) p = 1;
	else if (p < -1) p = -1;
	float_type theta2=acos(p);
	tmp.push_back(theta0);
	tmp.push_back(theta1);
	tmp.push_back(theta2);
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
