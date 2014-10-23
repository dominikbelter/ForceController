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
std::vector<float_type> KinematicLie::inverseKinematic(const Mat34& linkPose, unsigned int linkNo){
    std::vector<float_type> tmp;
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
