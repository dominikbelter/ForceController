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


int main(int argc, const char** argv)
{
	Keys active;
	int a1, b1, a2, b2, a3, b3, a4, b4, a5, b5;

	IrrlichtDevice *device =
		createDevice(video::EDT_SOFTWARE, dimension2d<u32>(640, 480), 16,
		false, false, false, &active);
	if (!device)
		return 1;


	video::IVideoDriver * video = device->getVideoDriver();
	scene::ISceneManager * menage = device->getSceneManager();
	scene::ICameraSceneNode * kam = menage->addCameraSceneNode();
	device->getCursorControl()->setVisible(false);
	kam->setPosition(core::vector3df(-4, 0, -30));
	kam->setFarValue(90000);


	//Downloading model

	IAnimatedMesh * coxa = menage->getMesh("../../resources/coxa_axis.stl");
	IAnimatedMesh * vitulus = menage->getMesh("../../resources/vitulus_axis.stl");
	IAnimatedMesh * femur = menage->getMesh("../../resources/femur_axis.stl");

	//-----------------------------right parts
	//---------------------------- first leg
	IAnimatedMeshSceneNode * cx = menage->addAnimatedMeshSceneNode(coxa);
	menage->getMeshManipulator()->makePlanarTextureMapping(cx->getMesh(), 0.004f);
	cx->setScale(core::vector3df(1, 1, 1));
	cx->setPosition(core::vector3df(1, 0, 0));
	cx->setRotation(core::vector3df(0, 90, 270));
	cx->setMaterialFlag(video::EMF_LIGHTING, false);
	cx->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	IAnimatedMeshSceneNode * vit = menage->addAnimatedMeshSceneNode(vitulus);
	menage->getMeshManipulator()->makePlanarTextureMapping(vit->getMesh(), 0.004f);
	vit->setScale(core::vector3df(1, 1, 1));
	vit->setRotation(core::vector3df(90, 90, 180));
	vit->setPosition(core::vector3df(16.5, 5, 0));
	vit->setMaterialFlag(video::EMF_LIGHTING, false);
	vit->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	IAnimatedMeshSceneNode * fe = menage->addAnimatedMeshSceneNode(femur);
	menage->getMeshManipulator()->makePlanarTextureMapping(fe->getMesh(), 0.004f);
	fe->setScale(core::vector3df(1, 1, 1));
	fe->setRotation(core::vector3df(90, 90, 90));
	fe->setPosition(core::vector3df(5.5, 0, 0));
	fe->setMaterialFlag(video::EMF_LIGHTING, false);
	fe->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	//-----------------------------------------second leg
	a1 = 5;
	b1 = 12;

	IAnimatedMeshSceneNode * cx1 = menage->addAnimatedMeshSceneNode(coxa);
	menage->getMeshManipulator()->makePlanarTextureMapping(cx1->getMesh(), 0.004f);
	cx1->setScale(core::vector3df(1, 1, 1));
	cx1->setPosition(core::vector3df(1 + a1, 0, b1));
	cx1->setRotation(core::vector3df(0, 90, 270));
	cx1->setMaterialFlag(video::EMF_LIGHTING, false);
	cx1->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	IAnimatedMeshSceneNode * vit1 = menage->addAnimatedMeshSceneNode(vitulus);
	menage->getMeshManipulator()->makePlanarTextureMapping(vit1->getMesh(), 0.004f);
	vit1->setScale(core::vector3df(1, 1, 1));
	vit1->setRotation(core::vector3df(90, 90, 180));
	vit1->setPosition(core::vector3df(16.5 + a1, 5, b1));
	vit1->setMaterialFlag(video::EMF_LIGHTING, false);
	vit1->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	IAnimatedMeshSceneNode * fe1 = menage->addAnimatedMeshSceneNode(femur);
	menage->getMeshManipulator()->makePlanarTextureMapping(fe1->getMesh(), 0.004f);
	fe1->setScale(core::vector3df(1, 1, 1));
	fe1->setRotation(core::vector3df(90, 90, 90));
	fe1->setPosition(core::vector3df(5.5 + a1, 0, b1));
	fe1->setMaterialFlag(video::EMF_LIGHTING, false);
	fe1->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	//-----------------------third leg
	a2 = 0;
	b2 = 24;

	IAnimatedMeshSceneNode * cx2 = menage->addAnimatedMeshSceneNode(coxa);
	menage->getMeshManipulator()->makePlanarTextureMapping(cx2->getMesh(), 0.004f);
	cx2->setScale(core::vector3df(1, 1, 1));
	cx2->setPosition(core::vector3df(1 + a2, 0, b2));
	cx2->setRotation(core::vector3df(0, 90, 270));
	cx2->setMaterialFlag(video::EMF_LIGHTING, false);
	cx2->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	IAnimatedMeshSceneNode * vit2 = menage->addAnimatedMeshSceneNode(vitulus);
	menage->getMeshManipulator()->makePlanarTextureMapping(vit2->getMesh(), 0.004f);
	vit2->setScale(core::vector3df(1, 1, 1));
	vit2->setRotation(core::vector3df(90, 90, 180));
	vit2->setPosition(core::vector3df(16.5 + a2, 5, b2));
	vit2->setMaterialFlag(video::EMF_LIGHTING, false);
	vit2->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	IAnimatedMeshSceneNode * fe2 = menage->addAnimatedMeshSceneNode(femur);
	menage->getMeshManipulator()->makePlanarTextureMapping(fe2->getMesh(), 0.004f);
	fe2->setScale(core::vector3df(1, 1, 1));
	fe2->setRotation(core::vector3df(90, 90, 90));
	fe2->setPosition(core::vector3df(5.5 + a2, 0, b2));
	fe2->setMaterialFlag(video::EMF_LIGHTING, false);
	fe2->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	//-----------------------------left part
	//-------------------------------first leg


	IAnimatedMeshSceneNode * cx3 = menage->addAnimatedMeshSceneNode(coxa);
	menage->getMeshManipulator()->makePlanarTextureMapping(cx3->getMesh(), 0.004f);
	cx3->setScale(core::vector3df(1, 1, 1));
	cx3->setPosition(core::vector3df(-8.5, 0, 0));
	cx3->setRotation(core::vector3df(0, 90, 90));
	cx3->setMaterialFlag(video::EMF_LIGHTING, false);
	cx3->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	IAnimatedMeshSceneNode * vit3 = menage->addAnimatedMeshSceneNode(vitulus);
	menage->getMeshManipulator()->makePlanarTextureMapping(vit3->getMesh(), 0.004f);
	vit3->setScale(core::vector3df(1, 1, 1));
	vit3->setRotation(core::vector3df(90, 90, 180));
	vit3->setPosition(core::vector3df(-24, 5, 0));
	vit3->setMaterialFlag(video::EMF_LIGHTING, false);
	vit3->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	IAnimatedMeshSceneNode * fe3 = menage->addAnimatedMeshSceneNode(femur);
	menage->getMeshManipulator()->makePlanarTextureMapping(fe3->getMesh(), 0.004f);
	fe3->setScale(core::vector3df(1, 1, 1));
	fe3->setRotation(core::vector3df(270, 270, 90));
	fe3->setPosition(core::vector3df(-13, 0, 0));
	fe3->setMaterialFlag(video::EMF_LIGHTING, false);
	fe3->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));
	//-----------------------second leg
	a4 = -5;
	b4 = 12;

	IAnimatedMeshSceneNode * cx4 = menage->addAnimatedMeshSceneNode(coxa);
	menage->getMeshManipulator()->makePlanarTextureMapping(cx4->getMesh(), 0.004f);
	cx4->setScale(core::vector3df(1, 1, 1));
	cx4->setPosition(core::vector3df(-8.5 + a4, 0, 0 + b4));
	cx4->setRotation(core::vector3df(0, 90, 90));
	cx4->setMaterialFlag(video::EMF_LIGHTING, false);
	cx4->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	IAnimatedMeshSceneNode * vit4 = menage->addAnimatedMeshSceneNode(vitulus);
	menage->getMeshManipulator()->makePlanarTextureMapping(vit4->getMesh(), 0.004f);
	vit4->setScale(core::vector3df(1, 1, 1));
	vit4->setRotation(core::vector3df(90, 90, 180));
	vit4->setPosition(core::vector3df(-24 + a4, 5, 0 + b4));
	vit4->setMaterialFlag(video::EMF_LIGHTING, false);
	vit4->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	IAnimatedMeshSceneNode * fe4 = menage->addAnimatedMeshSceneNode(femur);
	menage->getMeshManipulator()->makePlanarTextureMapping(fe4->getMesh(), 0.004f);
	fe4->setScale(core::vector3df(1, 1, 1));
	fe4->setRotation(core::vector3df(270, 270, 90));
	fe4->setPosition(core::vector3df(-13 + a4, 0, 0 + b4));
	fe4->setMaterialFlag(video::EMF_LIGHTING, false);
	fe4->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));
	//------------------------third leg
	a5 = 0;
	b5 = 24;

	IAnimatedMeshSceneNode * cx5 = menage->addAnimatedMeshSceneNode(coxa);
	menage->getMeshManipulator()->makePlanarTextureMapping(cx5->getMesh(), 0.004f);
	cx5->setScale(core::vector3df(1, 1, 1));
	cx5->setPosition(core::vector3df(-8.5 + a5, 0, 0 + b5));
	cx5->setRotation(core::vector3df(0, 90, 90));
	cx5->setMaterialFlag(video::EMF_LIGHTING, false);
	cx5->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	IAnimatedMeshSceneNode * vit5 = menage->addAnimatedMeshSceneNode(vitulus);
	menage->getMeshManipulator()->makePlanarTextureMapping(vit5->getMesh(), 0.004f);
	vit5->setScale(core::vector3df(1, 1, 1));
	vit5->setRotation(core::vector3df(90, 90, 180));
	vit5->setPosition(core::vector3df(-24 + a5, 5, 0 + b5));
	vit5->setMaterialFlag(video::EMF_LIGHTING, false);
	vit5->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	IAnimatedMeshSceneNode * fe5 = menage->addAnimatedMeshSceneNode(femur);
	fe5->setScale(core::vector3df(1, 1, 1));
	fe5->setRotation(core::vector3df(270, 270, 90));
	fe5->setPosition(core::vector3df(-13 + a5, 0, 0 + b5));
	fe5->setMaterialFlag(video::EMF_LIGHTING, false);
	fe5->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));


	//---------------Korpus
	IAnimatedMesh * korpus = menage->getMesh("../../resources/korpus.stl");
	IAnimatedMeshSceneNode * kor = menage->addAnimatedMeshSceneNode(korpus);
	kor->setScale(core::vector3df(1, 1, 1));
	kor->setRotation(core::vector3df(90, 90, 0));
	kor->setPosition(core::vector3df(0, 0, 0));
	kor->setMaterialFlag(video::EMF_LIGHTING, false);
	kor->setMaterialTexture(0, video->getTexture("../../resources/wall.jpg"));

	while (device->run())

	{

		//--------------Camera Control
		if (active.IsKeyDown(irr::KEY_KEY_W))
		{
			core::vector3df v = kam->getPosition();
			v.Z += 0.5f;
			kam->setPosition(v);
		}
		else if (active.IsKeyDown(irr::KEY_KEY_S))
		{
			core::vector3df v = kam->getPosition();
			v.Z -= 0.5f;
			kam->setPosition(v);
		}
		else if (active.IsKeyDown(irr::KEY_KEY_D))
		{
			core::vector3df v = kam->getPosition();
			v.X += 0.50f;
			kam->setPosition(v);
		}
		else if (active.IsKeyDown(irr::KEY_KEY_A))
		{
			core::vector3df v = kam->getPosition();
			v.X -= 0.50f;
			kam->setPosition(v);
		}
		else if (active.IsKeyDown(irr::KEY_UP))
		{
			core::vector3df v = kam->getPosition();
			v.Y += 0.50f;
			kam->setPosition(v);
		}
		else if (active.IsKeyDown(irr::KEY_DOWN))
		{
			core::vector3df v = kam->getPosition();
			v.Y -= 0.50f;
			kam->setPosition(v);
		}

		//---------------------leg test
		//right leg test
		//FRONT LEG

		else if (active.IsKeyDown(irr::KEY_KEY_B)){
			if (active.IsKeyDown(irr::KEY_KEY_H))
			{
				core::vector3df v = vit->getRotation();
				v.Z -= 1.0f;
				vit->setRotation(v);
			}
			else if (active.IsKeyDown((irr::KEY_KEY_J)))
			{
				core::vector3df v = vit->getRotation();
				v.Z += 1.0f;
				vit->setRotation(v);
			}
			else if (active.IsKeyDown((irr::KEY_KEY_N)))
			{
				core::vector3df v = fe->getRotation();
				core::vector3df x = fe->getPosition();
				core::vector3df w = vit->getRotation();
				w.Z -= 1.0f;
				v.Z -= 1.0f;
				fe->setRotation(v);
				vit->setRotation(w);
				vit->setPosition(core::vector3df(6 + 11.7*sin((v.Z + 25.7)*PI / 180), -11.7*cos((v.Z + 25.7)*PI / 180), x.Z));
			}
			else if (active.IsKeyDown(irr::KEY_KEY_M))
			{
				core::vector3df v = fe->getRotation();
				core::vector3df x = fe->getPosition();
				core::vector3df w = vit->getRotation();
				v.Z += 1.0f;
				w.Z += 1.0f;
				fe->setRotation(v);
				vit->setRotation(w);
				vit->setPosition(core::vector3df(6 + 11.7*sin((v.Z + 25.7)*PI / 180), -11.7*cos((v.Z + 25.7)*PI / 180), x.Z));
			}
		}
		//CENTER LEG
		else if (active.IsKeyDown(irr::KEY_KEY_G)){
			if (active.IsKeyDown(irr::KEY_KEY_H))
			{
				core::vector3df v = vit1->getRotation();
				v.Z -= 1.0f;
				vit1->setRotation(v);
			}
			else if (active.IsKeyDown((irr::KEY_KEY_J)))
			{
				core::vector3df v = vit1->getRotation();
				v.Z += 1.0f;
				vit1->setRotation(v);
			}
			else if (active.IsKeyDown((irr::KEY_KEY_N)))
			{
				core::vector3df v = fe1->getRotation();
				core::vector3df x = fe1->getPosition();
				core::vector3df w = vit1->getRotation();
				w.Z -= 1.0f;
				v.Z -= 1.0f;
				fe1->setRotation(v);
				vit1->setRotation(w);
				vit1->setPosition(core::vector3df(11 + 11.7*sin((v.Z + 25.7)*PI / 180), -11.7*cos((v.Z + 25.7)*PI / 180), x.Z));
			}
			else if (active.IsKeyDown(irr::KEY_KEY_M))
			{
				core::vector3df v = fe1->getRotation();
				core::vector3df x = fe1->getPosition();
				core::vector3df w = vit1->getRotation();
				v.Z += 1.0f;
				w.Z += 1.0f;
				fe1->setRotation(v);
				vit1->setRotation(w);
				vit1->setPosition(core::vector3df(11 + 11.7*sin((v.Z + 25.7)*PI / 180), -11.7*cos((v.Z + 25.7)*PI / 180), x.Z));
			}
		}



		//LAST LEG
		else if (active.IsKeyDown(irr::KEY_KEY_T)){
			if (active.IsKeyDown(irr::KEY_KEY_H))
			{
				core::vector3df v = vit2->getRotation();
				v.Z -= 1.0f;
				vit2->setRotation(v);
			}
			else if (active.IsKeyDown((irr::KEY_KEY_J)))
			{
				core::vector3df v = vit2->getRotation();
				v.Z += 1.0f;
				vit2->setRotation(v);
			}
			else if (active.IsKeyDown((irr::KEY_KEY_N)))
			{
				core::vector3df v = fe2->getRotation();
				core::vector3df x = fe2->getPosition();
				core::vector3df w = vit2->getRotation();
				w.Z -= 1.0f;
				v.Z -= 1.0f;
				fe2->setRotation(v);
				vit2->setRotation(w);
				vit2->setPosition(core::vector3df(6 + 11.7*sin((v.Z + 25.7)*PI / 180), -11.7*cos((v.Z + 25.7)*PI / 180), x.Z));
			}
			else if (active.IsKeyDown(irr::KEY_KEY_M))
			{
				core::vector3df v = fe2->getRotation();
				core::vector3df x = fe2->getPosition();
				core::vector3df w = vit2->getRotation();
				v.Z += 1.0f;
				w.Z += 1.0f;
				fe2->setRotation(v);
				vit2->setRotation(w);
				vit2->setPosition(core::vector3df(6 + 11.7*sin((v.Z + 25.7)*PI / 180), -11.7*cos((v.Z + 25.7)*PI / 180), x.Z));
			}
		}



		//-------left side
		//left leg test
		//FRONT LEG
		else if (active.IsKeyDown(irr::KEY_KEY_V)){
			if (active.IsKeyDown((irr::KEY_KEY_H)))
			{
				core::vector3df v = vit3->getRotation();
				v.Z += 1.0f;
				vit3->setRotation(v);
			}
			else if (active.IsKeyDown((irr::KEY_KEY_J)))
			{
				core::vector3df v = vit3->getRotation();
				v.Z -= 1.0f;
				vit3->setRotation(v);
			}
			else if (active.IsKeyDown((irr::KEY_KEY_N)))
			{

				core::vector3df v = fe3->getRotation();
				core::vector3df x = fe3->getPosition();
				core::vector3df w = vit3->getRotation();
				v.Z += 1.0f;
				w.Z += 1.0f;
				fe3->setRotation(v);
				vit3->setRotation(w);
				vit3->setPosition(core::vector3df(-(13.5 + 11.7*sin((v.Z - 25.7)*PI / 180)), 11.7*cos((v.Z - 25.7)*PI / 180), x.Z));


			}
			else if (active.IsKeyDown((irr::KEY_KEY_M)))
			{
				core::vector3df v = fe3->getRotation();
				core::vector3df x = fe3->getPosition();
				core::vector3df w = vit3->getRotation();
				v.Z -= 1.0f;
				w.Z -= 1.0f;
				fe3->setRotation(v);
				vit3->setRotation(w);
				vit3->setPosition(core::vector3df(-(13.5 + 11.7*sin((v.Z - 25.7)*PI / 180)), 11.7*cos((v.Z - 25.7)*PI / 180), x.Z));
			}
		}
		//CENTER LEG
		else if (active.IsKeyDown(irr::KEY_KEY_F)){
			if (active.IsKeyDown((irr::KEY_KEY_H)))
			{
				core::vector3df v = vit4->getRotation();
				v.Z += 1.0f;
				vit4->setRotation(v);
			}
			else if (active.IsKeyDown((irr::KEY_KEY_J)))
			{
				core::vector3df v = vit4->getRotation();
				v.Z -= 1.0f;
				vit4->setRotation(v);
			}
			else if (active.IsKeyDown((irr::KEY_KEY_N)))
			{

				core::vector3df v = fe4->getRotation();
				core::vector3df x = fe4->getPosition();
				core::vector3df w = vit4->getRotation();
				v.Z += 1.0f;
				w.Z += 1.0f;
				fe4->setRotation(v);
				vit4->setRotation(w);
				vit4->setPosition(core::vector3df(-(18.5 + 11.7*sin((v.Z - 25.7)*PI / 180)), 11.7*cos((v.Z - 25.7)*PI / 180), x.Z));


			}
			else if (active.IsKeyDown((irr::KEY_KEY_M)))
			{
				core::vector3df v = fe4->getRotation();
				core::vector3df x = fe4->getPosition();
				core::vector3df w = vit4->getRotation();
				v.Z -= 1.0f;
				w.Z -= 1.0f;
				fe4->setRotation(v);
				vit4->setRotation(w);
				vit4->setPosition(core::vector3df(-(18.5 + 11.7*sin((v.Z - 25.7)*PI / 180)), 11.7*cos((v.Z - 25.7)*PI / 180), x.Z));
			}
		}

		//LAST LEG
		else if (active.IsKeyDown(irr::KEY_KEY_R)){
			if (active.IsKeyDown((irr::KEY_KEY_H)))
			{
				core::vector3df v = vit5->getRotation();
				v.Z += 1.0f;
				vit5->setRotation(v);
			}
			else if (active.IsKeyDown((irr::KEY_KEY_J)))
			{
				core::vector3df v = vit5->getRotation();
				v.Z -= 1.0f;
				vit5->setRotation(v);
			}
			else if (active.IsKeyDown((irr::KEY_KEY_N)))
			{

				core::vector3df v = fe5->getRotation();
				core::vector3df x = fe5->getPosition();
				core::vector3df w = vit5->getRotation();
				v.Z += 1.0f;
				w.Z += 1.0f;
				fe5->setRotation(v);
				vit5->setRotation(w);
				vit5->setPosition(core::vector3df(-(13.5 + 11.7*sin((v.Z - 25.7)*PI / 180)), 11.7*cos((v.Z - 25.7)*PI / 180), x.Z));


			}
			else if (active.IsKeyDown((irr::KEY_KEY_M)))
			{
				core::vector3df v = fe5->getRotation();
				core::vector3df x = fe5->getPosition();
				core::vector3df w = vit5->getRotation();
				v.Z -= 1.0f;
				w.Z -= 1.0f;
				fe5->setRotation(v);
				vit5->setRotation(w);
				vit5->setPosition(core::vector3df(-(13.5 + 11.7*sin((v.Z - 25.7)*PI / 180)), 11.7*cos((v.Z - 25.7)*PI / 180), x.Z));
			}
		}
		//---------------------leg test end
		else if (active.IsKeyDown(irr::KEY_ESCAPE))
		{
			device->drop();
			return 0;
		}
		video->beginScene(true, true, video::SColor(255, 0, 10, 200));
		menage->drawAll();
		video->endScene();
	}
	//device->drop();
	return 0;

}