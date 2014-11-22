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

class Keys
	: public IEventReceiver
{
public:
	virtual bool OnEvent(const SEvent & event)
	{
		//  Remember key event (press/unpress)
		if (event.EventType == irr::EET_KEY_INPUT_EVENT)
			KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;

		return false;
	}
	// Checking if key is pressed
	virtual bool IsKeyDown(EKEY_CODE keyCode) const
	{
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
SMaterial videoMaterial;

const bool debug = true;
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




void drawLeg(video::IVideoDriver* video, int legIndex, vector3d<f32> position, vector3d<f32> radians) {

    static long i = 0;
    static float j  = 0;
    i++;

    core::matrix4  oldTransform;

    oldTransform = video->getTransform(video::ETS_WORLD);

    ourTransform(video, radians,position);

    ourTransform(video, 0,PI/2 ,j , 0,0 ,0);

    video->drawMeshBuffer(coxaMeshBuffer);

    ourTransform(video, 0,-PI/2  - j,0,  0,0 ,5);

    video->drawMeshBuffer(femurMeshBuffer);

    ourTransform(video, 0,-PI/2 + PI/20*j ,0 ,  11.2,0 , 5);

    video->drawMeshBuffer(vitulusMeshBuffer);

    video->setTransform(video::ETS_WORLD, oldTransform);

    if(i > 10) {
        i=0;
        j= j + 0.02;
    }
}



int main(int argc, const char** argv)
{
	Keys active;
	int a1, b1, a2, b2, a3, b3, a4, b4, a5, b5;

	IrrlichtDevice *device =
        createDevice(video::EDT_SOFTWARE, dimension2d<u32>(1400, 900), 16,
		false, false, false, &active);
	if (!device)
		return 1;



	video::IVideoDriver * video = device->getVideoDriver();

    scene::ISceneManager * manager = device->getSceneManager();
    scene::ICameraSceneNode * camera = manager->addCameraSceneNode();
    device->getCursorControl()->setVisible(false);
    camera->setPosition(core::vector3df(20, 20, 20));
    camera->setTarget(core::vector3df(-5, 0,0 ));


    //Loading model
    IAnimatedMesh * coxa  = manager->getMesh("../../resources/coxa_axis.stl");
    IAnimatedMesh * vitulus =   manager->getMesh("../../resources/vitulus_axis.stl");
    IAnimatedMesh * femur =  manager->getMesh("../../resources/femur_axis.stl");

    SMaterial meshMaterial;

    meshMaterial.Lighting = true;



    coxaMeshBuffer =   coxa->getMeshBuffer(meshMaterial);
    vitulusMeshBuffer = vitulus->getMeshBuffer(meshMaterial);
    femurMeshBuffer = femur->getMeshBuffer(meshMaterial);

    manager->getMeshManipulator()->makePlanarTextureMapping(coxaMeshBuffer, 0.04f);
    manager->getMeshManipulator()->makePlanarTextureMapping(vitulusMeshBuffer, 0.04f);
    manager->getMeshManipulator()->makePlanarTextureMapping(femurMeshBuffer, 0.04f);


    videoMaterial.Lighting = true;
    videoMaterial.MaterialType = video::EMT_SOLID;
    videoMaterial.setTexture(0, video->getTexture("../../resources/wall.jpg"));


    while(true) {


        //--------------Camera Control
        if (active.IsKeyDown(irr::KEY_KEY_W))
        {
        core::vector3df v = camera->getPosition();
        v.Z += 0.5f;
        camera->setPosition(v);
        cout<<"key_W";
        }
        else if (active.IsKeyDown(irr::KEY_KEY_S))
        {
        core::vector3df v = camera->getPosition();
        v.Z -= 0.5f;
        camera->setPosition(v);
        }
        else if (active.IsKeyDown(irr::KEY_KEY_D))
        {
        core::vector3df v = camera->getPosition();
        v.X += 0.50f;
        camera->setPosition(v);
        }
        else if (active.IsKeyDown(irr::KEY_KEY_A))
        {
        core::vector3df v = camera->getPosition();
        v.X -= 0.50f;
        camera->setPosition(v);
        }
        else if (active.IsKeyDown(irr::KEY_UP))
        {
        core::vector3df v = camera->getPosition();
        v.Y += 0.50f;
        camera->setPosition(v);
        }
        else if (active.IsKeyDown(irr::KEY_DOWN))
        {
        core::vector3df v = camera->getPosition();
        v.Y -= 0.50f;
        camera->setPosition(v);
        }

        video->beginScene(true, true, video::SColor(255, 0, 10, 200));

        video->setMaterial(videoMaterial);

        drawAxis(video);

        drawLeg(video, 1, vector3d<f32>(-5,0,0), vector3d<f32>(0,0,0));

        drawLeg(video, 1, vector3d<f32>(5,0,0), vector3d<f32>(0,PI,0));

        manager->drawAll();

		video->endScene();


    }



	return 0;

}



