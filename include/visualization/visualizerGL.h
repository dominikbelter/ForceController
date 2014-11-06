/** @file visualizerGL.h
*
* Visualizer interface
*/
/**
* @author Tomasz Walczewski
* @author Sebastian Sebulba Drogowski
* @mainpage
*/
#ifndef _VISUALIZERGL_H_
#define _VISUALIZERGL_H_


#include "../defs/defs.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <string>
#include <vector>
#include "visualizer.h"

namespace controller {
	Visualizer* createVisualizerGL(const std::string _name);
	Visualizer* createVisualizerGL(std::string configFilename, const std::string _name);
};


using namespace controller;

	/// Visualizer interface
	class VisualizerGL : public Visualizer {
	public:


		/// Pointer
		typedef std::unique_ptr<VisualizerGL> Ptr;

		// overloaded constructor
		VisualizerGL(const std::string _name) : Visualizer(_name, TYPE_GL) {};

		VisualizerGL(std::string configFilename, const std::string _name) : Visualizer(configFilename, _name, TYPE_GL) {}

		// Name of the Visualizer
		const std::string& getName() const { return name; }

		//Compute torque in each joint for given the force applied in the foot
		void drawRobot(const Mat34& robotPose, std::vector<float_type> configuration);

		// Virtual descrutor
		~VisualizerGL() {}

	};


#endif // _VISUALIZERGL_H_
