#include "visualization\visualizerGL.h"


using namespace controller;
	

/// A single instance of VisualizerGL
VisualizerGL::Ptr visualizerGL;

void VisualizerGL::drawRobot(const Mat34& robotPose, std::vector<float_type> configuration) {
	// do sth
	
}

controller::Visualizer* controller::createVisualizerGL(const std::string _name) {
	visualizerGL.reset(new VisualizerGL(_name));
	return visualizerGL.get();
}

controller::Visualizer* controller::createVisualizerGL(std::string configFilename, const std::string _name) {
	visualizerGL.reset(new VisualizerGL(configFilename, _name));
	return visualizerGL.get();
}