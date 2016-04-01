#include "../include/visualization/visualizerIrrlicht.h"
#include <math.h>


#include "driverChoice.h"


using namespace controller;
using namespace std;
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;






/// A single instance of VisualizerIrrlicht
VisualizerIrrlicht::Ptr visualizerIrrlicht;



VisualizerIrrlicht::VisualizerIrrlicht(const std::string _name, int width, int height, irr::f32 _axisBoxSize, bool _debug) : debug(_debug) , axisBoxSize(_axisBoxSize), Visualizer(_name, TYPE_IRRLICHT) {
    initialize(width, height);
}


VisualizerIrrlicht::VisualizerIrrlicht(std::string configFilename, const std::string _name) : axisBoxSize(0.01) , Visualizer(configFilename, _name, TYPE_IRRLICHT) {

    int width, height;

    tinyxml2::XMLDocument config;


    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.FirstChildElement() == nullptr)
        std::cout << "unable to load Kinematic config file.\n";
    else {
        width = std::stoi(config.FirstChildElement("width")->GetText());
        height = std::stoi(config.FirstChildElement("height")->GetText());

        int debugResult = std::stoi(config.FirstChildElement("debug")->GetText());
        if(debugResult == 1)
            debug = true;
        else
            debug = false;

    }
    initialize(width,height);
}

void VisualizerIrrlicht::mat34ToIrrlichtTransform(const Mat34& robotPose) {

    core::matrix4 mat;

    float alfa, beta, gamma;

    /// DB lepiej atan2
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

void VisualizerIrrlicht::setPosition(std::vector<float_type> configuration){
    //m_configuration = configuration;
    //6th leg
    m_configuration[0]=-configuration[9];
    m_configuration[1]=configuration[10];
    m_configuration[2]=configuration[11];
    //5th leg
    m_configuration[3]=-configuration[12];
    m_configuration[4]=configuration[13];
    m_configuration[5]=configuration[14];
    //4th leg
    m_configuration[6]=-configuration[15];
    m_configuration[7]=configuration[16];
    m_configuration[8]=configuration[17];

    //1st leg
    m_configuration[9]=-configuration[0];
    m_configuration[10]=configuration[1];
    m_configuration[11]=configuration[2];
    //2nd leg
    m_configuration[12]=-configuration[3];
    m_configuration[13]=configuration[4];
    m_configuration[14]=configuration[5];
    //3rd leg
    m_configuration[15]=-configuration[6];
    m_configuration[16]=configuration[7];
    m_configuration[17]=configuration[8];
}

void VisualizerIrrlicht::setPosition(unsigned char legNo, std::vector<float_type> configuration){

    configuration[0] = -configuration[0];

    for(int i=0; i<3; i++)
    {
        if(legNo < 3)
            m_configuration[legNo*3+9+i] = configuration[i];
        else
            m_configuration[9-(legNo-2)*3+i] = configuration[i];
    }

}

std::vector<float_type> VisualizerIrrlicht::getPosition(int legNo)
{
    std::vector<float_type> configuration;
    for(int i=0; i<3; i++)
    {
        if(legNo < 3)
            configuration.push_back(m_configuration[legNo*3+9+i]);
        else
            configuration.push_back(m_configuration[9-(legNo-2)*3+i]);
    }

    configuration[0] = -configuration[0];

    return configuration;
}

void VisualizerIrrlicht::drawRobot(const Mat34& robotPose, std::vector<float_type> configuration) {
    this->m_configuration = configuration;
    while (device->run()) {
           if(receiver.IsKeyDown(irr::KEY_KEY_Q)) {
               vector3df pos = camera->getPosition();
               pos.X = pos.X + 1;
               camera->setPosition(pos);
           } else
           if(receiver.IsKeyDown(irr::KEY_KEY_W)) {
               vector3df pos = camera->getPosition();
               pos.X = pos.X - 1;
               camera->setPosition(pos);
           } else
               if(receiver.IsKeyDown(irr::KEY_KEY_A)) {
                   vector3df pos = camera->getPosition();
                   pos.Y = pos.Y + 1;
                   camera->setPosition(pos);
               } else
               if(receiver.IsKeyDown(irr::KEY_KEY_S)) {
                   vector3df pos = camera->getPosition();
                   pos.Y = pos.Y - 1;
                   camera->setPosition(pos);
               } else
                   if(receiver.IsKeyDown(irr::KEY_KEY_Z)) {
                       vector3df pos = camera->getPosition();
                       pos.Z = pos.Z + 1;
                       camera->setPosition(pos);
                   } else
                   if(receiver.IsKeyDown(irr::KEY_KEY_X)) {
                       vector3df pos = camera->getPosition();
                       pos.Z = pos.Z - 1;
                       camera->setPosition(pos);
                   } else

           if(debug) {
           if(receiver.IsKeyDown(irr::KEY_KEY_T)) {
             this->m_configuration[0] = this->m_configuration[0] + PI/50;
           } else
               if(receiver.IsKeyDown(irr::KEY_KEY_R)) {
               this->m_configuration[0] = this->m_configuration[0] - PI/50;
             } else
               if(receiver.IsKeyDown(irr::KEY_KEY_G)) {
               this->m_configuration[1] = this->m_configuration[1] + PI/50;
             } else
               if(receiver.IsKeyDown(irr::KEY_KEY_F)) {
               this->m_configuration[1] = this->m_configuration[1] - PI/50;
             } else
               if(receiver.IsKeyDown(irr::KEY_KEY_B)) {
                 this->m_configuration[2] = this->m_configuration[2] + PI/50;
            } else
               if(receiver.IsKeyDown(irr::KEY_KEY_V)) {
               this->m_configuration[2] = this->m_configuration[2] - PI/50;
            }
           }


            video->beginScene(true, true, video::SColor(255, 0, 10, 200));
            video->setMaterial(videoMaterial);

            drawAxis();

            mat34ToIrrlichtTransform(robotPose);

           drawBody(vector3d<f32>(0, 0, 0), vector3d<f32>(PI / 2, 0, 0));


            drawLeg(1, vector3d<f32>(0, 0, 0), vector3d<f32>(0, 0, 0), this->m_configuration);
            drawLeg(2, vector3d<f32>(-12, 0, 5), vector3d<f32>(0, 0, 0), this->m_configuration);
            drawLeg(3, vector3d<f32>(-24, 0, 0), vector3d<f32>(0, 0, 0), this->m_configuration);
            drawLeg(4, vector3d<f32>(0, 0, -8), vector3d<f32>(0, PI, 0), this->m_configuration);
            drawLeg(5, vector3d<f32>(-12, 0, -13), vector3d<f32>(0, PI, 0), this->m_configuration);
            drawLeg(6, vector3d<f32>(-24, 0, -8), vector3d<f32>(0, PI, 0), this->m_configuration);

            manager->drawAll();



            video->endScene();


    }

    device->drop();
}

controller::Visualizer* controller::createVisualizerIrrlicht(const std::string _name, int width, int height, irr::f32 axisBoxSize, bool debug) {
    visualizerIrrlicht.reset(new VisualizerIrrlicht(_name, width, height, axisBoxSize, debug));
    return visualizerIrrlicht.get();
}

controller::Visualizer* controller::createVisualizerIrrlicht(std::string configFilename, const std::string _name) {
   visualizerIrrlicht.reset(new VisualizerIrrlicht(configFilename, _name));
    return visualizerIrrlicht.get();
}



int VisualizerIrrlicht::initialize(int width, int height) {



    device =
        createDevice(video::EDT_SOFTWARE, dimension2d<u32>(width, height), 16,
        false, false, false, &receiver);
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


void VisualizerIrrlicht::drawLeg(int legIndex,  irr::core::vector3d<irr::f32> position, irr::core::vector3d<irr::f32> radians, std::vector<float_type> configuration) {

    core::matrix4  oldTransform;
    oldTransform = video->getTransform(video::ETS_WORLD);
    ourTransform(radians, position);
    ourTransform(0, PI / 2, configuration.at(3*(legIndex - 1) + 0), 0, 0, 0);

    video->drawMeshBuffer(coxaMeshBuffer);

    ///DB ugly patch -- 28deg and -50 deg offset

    ourTransform(0, -PI / 2 +28*3.14/180- configuration.at(3*(legIndex - 1) + 1) , 0, 0, 0, 5);
    video->drawMeshBuffer(femurMeshBuffer);
    ourTransform(0, -PI / 2 -50*3.14/180- configuration.at(3*(legIndex - 1) + 2) - 1.1989, 0, 11.2, 0, 5);
    //ourTransform(0, - configuration.at(3*(legIndex - 1) + 2) - 1.1989, 0, 11.2, 0, 5);
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
        video->draw3DBox(aabbox3d<f32>(-axisBoxSize, -axisBoxSize, 0, axisBoxSize, axisBoxSize, 10), SColor(100, 255, 0, 0));
        video->draw3DBox(aabbox3d<f32>(0, -axisBoxSize, -axisBoxSize, 10, axisBoxSize, axisBoxSize), SColor(100, 0, 255, 0));
        video->draw3DBox(aabbox3d<f32>(-axisBoxSize, 0, -axisBoxSize, axisBoxSize, 10, axisBoxSize), SColor(100, 255, 255, 255));
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
