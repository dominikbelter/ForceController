#include "include/defs/defs.h"
#include "include/visualization/visualizer.h"
#include <iostream>
#include <stdio.h>
#include <irrlicht.h>


// Irrlicht installation
//
// sudo apt-get install libIrrlicht1.8 libIrrlicht-dev

using namespace std;
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
using namespace std;


class MyEventReceiver : public IEventReceiver
{
public:
    // This is the one method that we have to implement
    virtual bool OnEvent(const SEvent& event)
    {
        // Remember whether each key is down or up
        if (event.EventType == irr::EET_KEY_INPUT_EVENT)
            KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;

        return false;
    }

    // This is used to check whether a key is being held down
    virtual bool IsKeyDown(EKEY_CODE keyCode) const
    {
        return KeyIsDown[keyCode];
    }

    MyEventReceiver()
    {
        for (u32 i=0; i<KEY_KEY_CODES_COUNT; ++i)
            KeyIsDown[i] = false;
    }

private:
    // We use this array to store the current state of each key
    bool KeyIsDown[KEY_KEY_CODES_COUNT];
};



class Keys
    : public IEventReceiver
{
public:
    virtual bool OnEvent(const SEvent & event)
    {
        //  Remember key event (press/unpress)
        if (event.EventType == irr::EET_KEY_INPUT_EVENT)
            KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;
cout<<"wykonujesie"<<endl;
        return false;
    }
    // Checking if key is pressed
    virtual bool IsKeyDown(EKEY_CODE keyCode) const
    {
         //cout<<keyCode<<endl;
        return KeyIsDown[keyCode];

    }
    Keys()
    {
        for (u32 i = 0; i < KEY_KEY_CODES_COUNT; ++i)
            KeyIsDown[i] = false;

    }

private:
    // Using to collect accually state all keys
    bool KeyIsDown[KEY_KEY_CODES_COUNT];
};




IMeshBuffer* coxaMeshBuffer;
IMeshBuffer* vitulusMeshBuffer;
IMeshBuffer* femurMeshBuffer;
IMeshBuffer* korpusMeshBuffer;
SMaterial videoMaterial;

const bool debug = false;
const f32 size = 0.01;


void drawAxis(video::IVideoDriver* video) {
    if(debug) {
        video->draw3DBox(aabbox3d<f32>(-size,-size,0,size,size,10),SColor(100,255,0,0));
        video->draw3DBox(aabbox3d<f32>(0,-size,-size,10,size,size),SColor(100,0,255,0));
        video->draw3DBox(aabbox3d<f32>(-size,0,-size,size,10,size),SColor(100,255,255,255));
    }

}


core::matrix4 ourTransform(video::IVideoDriver* video,  f32 xRad, f32 yRad, f32 zRad, f32 xTrans, f32 yTrans, f32 zTrans) {
    core::matrix4 mat, matOld;
    core::vector3df vectorOld;

    mat.setRotationRadians(vector3d<f32>(yRad, zRad, xRad));
    mat.setTranslation(vector3d<f32>(yTrans, zTrans, xTrans));

    matOld = video->getTransform(video::ETS_WORLD);
    mat = matOld.operator *(mat);

    vectorOld = matOld.getTranslation();

    video->setTransform(video::ETS_WORLD, mat);

    if(debug)
        cout << "X: " << vectorOld.X << " Y: " << vectorOld.Y << " Z: " << vectorOld.Z << endl;

    drawAxis(video);

    return mat;
}

core::matrix4 ourTransform(video::IVideoDriver* video, vector3d<f32> radians, vector3d<f32> position) {
    core::matrix4 mat, matOld;
    core::vector3df vectorOld;

    mat.setRotationRadians(radians);
    mat.setTranslation(position);

    matOld = video->getTransform(video::ETS_WORLD);
    mat = matOld.operator *(mat);

    vectorOld = matOld.getTranslation();

    video->setTransform(video::ETS_WORLD, mat);

    if(debug)
        cout << "X: " << vectorOld.X << " Y: " << vectorOld.Y << " Z: " << vectorOld.Z << endl;

    drawAxis(video);

    return mat;
}




void drawLeg(video::IVideoDriver* video, int legIndex, vector3d<f32> position, vector3d<f32> radians,float konfig[3][6]) {

    static long i = 0;
    static float j  = 0;
    vector3d<f32> kinematic;
   i++;


       kinematic.X = konfig[0][legIndex-1];
       kinematic.Y = konfig[1][legIndex-1];
       kinematic.Z = konfig[2][legIndex-1];




       core::matrix4  oldTransform;

    oldTransform = video->getTransform(video::ETS_WORLD);

    ourTransform(video, radians,position);

    ourTransform(video, 0,PI/2 ,kinematic.X , 0,0 ,0);

    video->drawMeshBuffer(coxaMeshBuffer);

    ourTransform(video, 0,-PI/2  - kinematic.Y,0,  0,0 ,5);

    video->drawMeshBuffer(femurMeshBuffer);

    ourTransform(video, 0,-PI - kinematic.Z ,0 ,  11.2,0 , 5);

    video->drawMeshBuffer(vitulusMeshBuffer);

    video->setTransform(video::ETS_WORLD, oldTransform);

    if(i > 1 && i < 100) {
        //i=0;
        j= j + 0.02;
    }
    if(i > 100 && i < 200) {
        //i=0;
        j= j + 0.02;
    }
    if(i=200){
        i=0;
    }
}

void drawBody(video::IVideoDriver* video, vector3d<f32> position, vector3d<f32> radians) {

    core::matrix4  oldTransform;

    oldTransform = video->getTransform(video::ETS_WORLD);

    ourTransform(video, radians,position);
    video->drawMeshBuffer(korpusMeshBuffer);

    video->setTransform(video::ETS_WORLD, oldTransform);

}


int main(int argc, const char** argv)
{
	Keys active;
    MyEventReceiver receiver;
    float konfig[3][6];
    int M;
   // M = [[2,0,0],[0,0,1],[0,1,1],[0,2,0],[1,0,1],[1,0,0],[0,0,2]];
    for(int i = 0; i<3; i++)
    {
        for(int j =0; j<6; j++)
        {
            konfig[i][j]=0;
        }
    }
    IrrlichtDevice *device =
            createDevice(video::EDT_SOFTWARE, dimension2d<u32>(640, 480), 16,
            false, false, false, &receiver);
        if (!device)
            return 1;



	video::IVideoDriver * video = device->getVideoDriver();

    scene::ISceneManager * manager = device->getSceneManager();
    scene::ICameraSceneNode * camera = manager->addCameraSceneNodeFPS();
    device->getCursorControl()->setVisible(false);
    camera->setPosition(core::vector3df(20, 20, 20));
    camera->setTarget(core::vector3df(-5, 0,0 ));
    camera->setFarValue(9000);

    //Loading model
    IAnimatedMesh * coxa  = manager->getMesh("../../resources/coxa_axis.stl");
    IAnimatedMesh * vitulus =   manager->getMesh("../../resources/vitulus_axis.stl");
    IAnimatedMesh * femur =  manager->getMesh("../../resources/femur_axis.stl");
    IAnimatedMesh * korpus = manager->getMesh("../../resources/korpus.stl");
    SMaterial meshMaterial;

    meshMaterial.Lighting = true;



    coxaMeshBuffer =   coxa->getMeshBuffer(meshMaterial);
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

    //    vector3d<f32> kinematic=vector3d<f32>(0,0,0);

cout <<konfig[2][2]<<endl;
    while(true) {

        //vector3d<f32> kinematic=vector3d<f32>(0,0,0);
        //--------------Camera Control

        video->beginScene(true, true, video::SColor(255, 0, 10, 200));

        video->setMaterial(videoMaterial);

        drawAxis(video);
        drawBody(video,vector3d<f32>(0,0,0), vector3d<f32>(PI/2,0,0));
        drawLeg(video, 1, vector3d<f32>(0,0,0), vector3d<f32>(0,0,0),konfig);
        drawLeg(video, 2, vector3d<f32>(-12,0,5), vector3d<f32>(0,0,0),konfig);
        drawLeg(video, 3, vector3d<f32>(-24,0,0), vector3d<f32>(0,0,0),konfig);
        drawLeg(video, 4, vector3d<f32>(0,0,-8), vector3d<f32>(0,PI,0),konfig);
        drawLeg(video, 5, vector3d<f32>(-12,0,-13), vector3d<f32>(0,PI,0),konfig);
        drawLeg(video, 6, vector3d<f32>(-24,0,-8), vector3d<f32>(0,PI,0),konfig);

        manager->drawAll();


        video->endScene();

//
    }



	return 0;

}



