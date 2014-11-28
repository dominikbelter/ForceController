#include "../include/visualization/visualizerIrrlicht.h"
#include <math.h>

using namespace controller;
using namespace std;
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

/// A single instance of VisualizerGL
VisualizerIrrlicht::Ptr visualizerIrrlicht;



void VisualizerIrrlicht::mat34ToIrrlichtTransform(const Mat34& robotPose) {

    core::matrix4 mat;

    float alfa, beta, gamma;

    gamma = atan(robotPose(2,1)/robotPose(2,2));
    alfa =  atan(robotPose(1,0)/robotPose(0,0));
    beta= atan(-robotPose(2,0)/sqrt(1-robotPose(2,0)*robotPose(2,0)));

    if(isnan(gamma))
        gamma=0;
    if(isnan(alfa))
        alfa =0;
    if(isnan(beta))
        beta=0;


    mat.setRotationRadians(vector3d<f32>(alfa, beta, gamma));
    mat.setTranslation(vector3d<f32>(robotPose(1,3), robotPose(2,3), robotPose(0,3)));

    if(debug)
        cout << alfa << endl << beta<< endl << gamma << endl;


   ourTransform(alfa,beta,gamma, robotPose(0,3), robotPose(1,3), robotPose(2,3));

}


void VisualizerIrrlicht::drawRobot(const Mat34& robotPose, std::vector<float_type> configuration) {

    int i = 0;
    int y =0;
    while (true) {
            video->beginScene(true, true, video::SColor(255, 0, 10, 200));
            video->setMaterial(videoMaterial);



            drawAxis();

           mat34ToIrrlichtTransform(robotPose);

           drawBody(vector3d<f32>(0, 0, 0), vector3d<f32>(PI / 2, 0, 0));


            drawLeg(1, vector3d<f32>(0, 0, 0), vector3d<f32>(0, 0, 0), configuration);

            drawLeg(2, vector3d<f32>(-12, 0, 5), vector3d<f32>(0, 0, 0), configuration);
            drawLeg(3, vector3d<f32>(-24, 0, 0), vector3d<f32>(0, 0, 0), configuration);
            drawLeg(4, vector3d<f32>(0, 0, -8), vector3d<f32>(0, PI, 0), configuration);
            drawLeg(5, vector3d<f32>(-12, 0, -13), vector3d<f32>(0, PI, 0), configuration);
            drawLeg(6, vector3d<f32>(-24, 0, -8), vector3d<f32>(0, PI, 0), configuration);

            manager->drawAll();



            video->endScene();


    }
}

controller::Visualizer* controller::createVisualizerIrrlicht(const std::string _name, int width, int height) {
    visualizerIrrlicht.reset(new VisualizerIrrlicht(_name, width, height));
    return visualizerIrrlicht.get();
}

controller::Visualizer* controller::createVisualizerIrrlicht(std::string configFilename, const std::string _name, int width, int height) {
    visualizerIrrlicht.reset(new VisualizerIrrlicht(configFilename, _name, width, height));
    return visualizerIrrlicht.get();
}

VisualizerIrrlicht::VisualizerIrrlicht(const std::string _name, int width, int height) : Visualizer(_name, TYPE_IRRLICHT) {
    initialize(width, height);
}

VisualizerIrrlicht::VisualizerIrrlicht(std::string configFilename, const std::string _name, int width, int height) : Visualizer(configFilename, _name, TYPE_IRRLICHT) {
    initialize(width, height);
}

int VisualizerIrrlicht::initialize(int width, int height) {
    IrrlichtDevice *device =
        createDevice(video::EDT_SOFTWARE, dimension2d<u32>(width, height), 16,
        false, false, false); //, &receiver);
    if (!device)
        return 1;

    video = device->getVideoDriver();

    manager = device->getSceneManager();

    camera = manager->addCameraSceneNode();
    device->getCursorControl()->setVisible(false);
    camera->setPosition(core::vector3df(40, 40, 30));
    camera->setTarget(core::vector3df(0, 0, 0));
    camera->setFarValue(9000);

    //Loading model
    IAnimatedMesh * coxa = manager->getMesh("../../resources/coxa_axis.stl");
    IAnimatedMesh * vitulus = manager->getMesh("../../resources/vitulus_axis.stl");
    IAnimatedMesh * femur = manager->getMesh("../../resources/femur_axis.stl");
    IAnimatedMesh * korpus = manager->getMesh("../../resources/korpus.stl");
    SMaterial meshMaterial;

    meshMaterial.Lighting = true;

    coxaMeshBuffer = coxa->getMeshBuffer(meshMaterial);
    vitulusMeshBuffer = vitulus->getMeshBuffer(meshMaterial);
    femurMeshBuffer = femur->getMeshBuffer(meshMaterial);
    korpusMeshBuffer = korpus->getMeshBuffer(meshMaterial);

    manager->getMeshManipulator()->makePlanarTextureMapping(coxaMeshBuffer, 0.04f);
    manager->getMeshManipulator()->makePlanarTextureMapping(vitulusMeshBuffer, 0.04f);
    manager->getMeshManipulator()->makePlanarTextureMapping(femurMeshBuffer, 0.04f);
    manager->getMeshManipulator()->makePlanarTextureMapping(korpusMeshBuffer, 0.04f);


    videoMaterial.Lighting = true;
    videoMaterial.MaterialType = video::EMT_SOLID;
    videoMaterial.setTexture(0, video->getTexture("../../resources/wall.jpg"));

}


void VisualizerIrrlicht::drawLeg1(int legIndex,  irr::core::vector3d<irr::f32> position, irr::core::vector3d<irr::f32> radians, std::vector<float_type> configuration) {

}

void VisualizerIrrlicht::drawLeg(int legIndex,  irr::core::vector3d<irr::f32> position, irr::core::vector3d<irr::f32> radians, std::vector<float_type> configuration) {

    core::matrix4  oldTransform;

    oldTransform = video->getTransform(video::ETS_WORLD);

    ourTransform(radians, position);



    ourTransform(0, PI / 2, configuration.at(3*(legIndex - 1) + 0), 0, 0, 0);

    video->drawMeshBuffer(coxaMeshBuffer);

    ourTransform(0, -PI / 2 - configuration.at(3*(legIndex - 1) + 1), 0, 0, 0, 5);

    video->drawMeshBuffer(femurMeshBuffer);

    ourTransform(0, -PI / 2 - configuration.at(3*(legIndex - 1) + 2), 0, 11.2, 0, 5);

    video->drawMeshBuffer(vitulusMeshBuffer);

   video->setTransform(video::ETS_WORLD, oldTransform);

}

void VisualizerIrrlicht::ourTransform(irr::core::vector3d<irr::f32> radians, irr::core::vector3d<irr::f32> position) {
    core::matrix4 mat, matOld;
    core::vector3df vectorOld;

    mat.setRotationRadians(radians);
    mat.setTranslation(position);

    matOld = video->getTransform(video::ETS_WORLD);
    mat = matOld.operator *(mat);

    vectorOld = matOld.getTranslation();

    video->setTransform(video::ETS_WORLD, mat);

    if (debug)
        cout << "X: " << vectorOld.X << " Y: " << vectorOld.Y << " Z: " << vectorOld.Z << endl;

    drawAxis();
}

void VisualizerIrrlicht::ourTransform(irr::f32 xRad, irr::f32 yRad, irr::f32 zRad, irr::f32 xTrans, irr::f32 yTrans, irr::f32 zTrans) {
    core::matrix4 mat, matOld;
    core::vector3df vectorOld;

    mat.setRotationRadians(vector3d<f32>(yRad, zRad, xRad));
    mat.setTranslation(vector3d<f32>(yTrans, zTrans, xTrans));

    matOld = video->getTransform(video::ETS_WORLD);
    mat = matOld.operator *(mat);

    vectorOld = matOld.getTranslation();

    video->setTransform(video::ETS_WORLD, mat);

    if (debug)
        cout << "X: " << vectorOld.X << " Y: " << vectorOld.Y << " Z: " << vectorOld.Z << endl;

    drawAxis();
}

void VisualizerIrrlicht::drawAxis() {
    if (debug) {
        video->draw3DBox(aabbox3d<f32>(-size, -size, 0, size, size, 10), SColor(100, 255, 0, 0));
        video->draw3DBox(aabbox3d<f32>(0, -size, -size, 10, size, size), SColor(100, 0, 255, 0));
        video->draw3DBox(aabbox3d<f32>(-size, 0, -size, size, 10, size), SColor(100, 255, 255, 255));
    }
}

void VisualizerIrrlicht::drawBody(irr::core::vector3d<irr::f32> position, irr::core::vector3d<irr::f32> radians) {
    core::matrix4  oldTransform;

    oldTransform = video->getTransform(video::ETS_WORLD);

    ourTransform(radians, position);
    video->drawMeshBuffer(korpusMeshBuffer);

    video->setTransform(video::ETS_WORLD, oldTransform);
}

void VisualizerIrrlicht::setDebugMode(bool enable) {
    this->debug = enable;
}

void VisualizerIrrlicht::setCameraTarget(irr::core::vector3df target) {
    camera->setTarget(target);
}

void VisualizerIrrlicht::setCameraPosition(irr::core::vector3df position) {
    camera->setPosition(position);
}
