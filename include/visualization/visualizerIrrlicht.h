/** @file visualizerIrrlicht.h
*
* Visualizer interface
*/


#ifndef _VISUALIZERIRRLICHT_H_
#define _VISUALIZERIRRLICHT_H_


#include "irrlicht.h"
#include "../defs/defs.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <string>
#include <vector>
#include "visualizer.h"
#include "../include/visualization/myeventreceiver.h"


namespace controller {
    Visualizer* createVisualizerIrrlicht(const std::string _name, int width, int height, irr::f32 axisBoxSize, bool debug);
    Visualizer* createVisualizerIrrlicht(std::string configFilename, const std::string _name);
};

using namespace controller;


    /// VisualizerGL interface
    class VisualizerIrrlicht : public Visualizer {
    public:

        /// Pointer
        typedef std::unique_ptr<VisualizerIrrlicht> Ptr;

        /// overloaded constructor
        VisualizerIrrlicht(const std::string _name, int width, int height, irr::f32 axisBoxSize, bool debug);

        VisualizerIrrlicht(std::string configFilename, const std::string _name );

        /** Name of the Robot
        * @param name
        */
        const std::string& getName() const { return name; }

        /** Draws a robot
        * @param robotPose
        * @param configuration
        */
        void drawRobot(const Mat34& robotPose, std::vector<float_type> configuration);

        void setPosition(std::vector<float_type> configuration);

        void setPosition(unsigned char legNo, std::vector<float_type> configuration);

        void setDebugMode(bool enable);

        std::vector<float_type> getPosition(int legNo);
        /// Virtual descrutor
        ~VisualizerIrrlicht() {}

    private:
        bool debug;

        MyEventReceiver receiver;
        IrrlichtDevice *device;

        irr::video::IVideoDriver* video;
        irr::scene::IMeshBuffer* coxaMeshBuffer;
        irr::scene::IMeshBuffer* vitulusMeshBuffer;
        irr::scene::IMeshBuffer* femurMeshBuffer;
        irr::scene::IMeshBuffer* korpusMeshBuffer;
        irr::video::SMaterial videoMaterial;
        irr::scene::ICameraSceneNode* camera;
        irr::scene::ISceneManager * manager;

        std::vector<float_type> m_configuration;

        const irr::f32 axisBoxSize;

        /** Draws a signle leg
        * @param  legIndex
        * @param  position
        * @param  rotation
        */
        void drawLeg(int legIndex,  irr::core::vector3d<irr::f32> position, irr::core::vector3d<irr::f32> radians, std::vector<float_type> configuration);



        /** Rotates and transforms the space
        * @param radians
        * @param position
        */
        void ourTransform(irr::core::vector3d<irr::f32> radians, irr::core::vector3d<irr::f32> position);

        /** Tranforms Mat34 to Irrlicht
        * @param robotPose robotPose
        */
        void mat34ToIrrlichtTransform(const Mat34& robotPose);


        /** Rotates and transforms the space
        * @param xRad
        * @param yRad
        * @param zRad
        * @param xTrans
        * @param yTrans
        * @param zTrans
        */
        void ourTransform(irr::f32 xRad, irr::f32 yRad, irr::f32 zRad, irr::f32 xTrans, irr::f32 yTrans, irr::f32 zTrans);


        /** Draws axis in debug mode
        */
        void drawAxis();

        /** Draws robot body
        * @param radians
        * @param position
        */
        void drawBody(irr::core::vector3d<irr::f32> position, irr::core::vector3d<irr::f32> radians);

        /** Initializes Irrlicht engine. Loads meshes and materials.
        */
        int initialize(int width, int height);

        /** Sets camera target
         *  @param target
        */
        void setCameraTarget(irr::core::vector3df target);

        /** Sets camera position
         *  @param position
        */
        void setCameraPosition(irr::core::vector3df position);


    };




#endif // _VISUALIZERIRRLICHT_H_
