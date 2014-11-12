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



int main(int argc, const char** argv)
{

	int a1, b1, a2, b2, a3, b3, a4, b4, a5, b5;

	IrrlichtDevice *device =
		createDevice(video::EDT_SOFTWARE, dimension2d<u32>(640, 480), 16,
		false, false, false, 0);
	if (!device)
		return 1;

	video::IVideoDriver * video = device->getVideoDriver();
	scene::ISceneManager * menage = device->getSceneManager();
	scene::ICameraSceneNode * kam = menage->addCameraSceneNode();


	//device->getCursorControl(false);
	device->getCursorControl()->setVisible(false);
	kam->setPosition(core::vector3df(-50, 50, -50));
	//kam->setPosition( core::vector3df( -20, 20, -30) );
	//Zasiêg pola widzenia kamery
	kam->setFarValue(90000);
	//Wczytywanie modelu

	IAnimatedMesh * coxa = menage->getMesh("../../resources/coxa.stl");
	IAnimatedMesh * vitulus = menage->getMesh("../../resources/vitulus.stl");
	IAnimatedMesh * femur = menage->getMesh("../../resources/femur.stl");

	//-----------------------------prawa strona
	//---------------------------- pierwsza noga
	IAnimatedMeshSceneNode * cx = menage->addAnimatedMeshSceneNode(coxa);
	cx->setScale(core::vector3df(1, 1, 1));
	cx->setPosition(core::vector3df(-9, 18, 0));
	cx->setRotation(core::vector3df(0, 0, 270));
	cx->setMaterialFlag(video::EMF_LIGHTING, false);
	cx->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMeshSceneNode * vit = menage->addAnimatedMeshSceneNode(vitulus);
	vit->setScale(core::vector3df(1, 1, 1));
	vit->setRotation(core::vector3df(90, 90, 180));
	vit->setPosition(core::vector3df(4.5, 24.5, 0));
	vit->setMaterialFlag(video::EMF_LIGHTING, false);
	vit->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMeshSceneNode * fe = menage->addAnimatedMeshSceneNode(femur);
	fe->setScale(core::vector3df(1, 1, 1));
	fe->setRotation(core::vector3df(90, 90, 90));
	fe->setPosition(core::vector3df(0, 20, 0));
	fe->setMaterialFlag(video::EMF_LIGHTING, false);
	fe->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	//-----------------------------------------druga noga
	a1 = 4;
	b1 = 12;

	IAnimatedMeshSceneNode * cx1 = menage->addAnimatedMeshSceneNode(coxa);
	cx1->setScale(core::vector3df(1, 1, 1));
	cx1->setPosition(core::vector3df(-9 + a1, 18, 0 + b1));
	cx1->setRotation(core::vector3df(0, 0, 270));
	cx1->setMaterialFlag(video::EMF_LIGHTING, false);
	cx1->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMeshSceneNode * vit1 = menage->addAnimatedMeshSceneNode(vitulus);
	vit1->setScale(core::vector3df(1, 1, 1));
	vit1->setRotation(core::vector3df(90, 90, 180));
	vit1->setPosition(core::vector3df(4.5 + a1, 24.5, 0 + b1));
	vit1->setMaterialFlag(video::EMF_LIGHTING, false);
	vit1->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMeshSceneNode * fe1 = menage->addAnimatedMeshSceneNode(femur);
	fe1->setScale(core::vector3df(1, 1, 1));
	fe1->setRotation(core::vector3df(90, 90, 90));
	fe1->setPosition(core::vector3df(0 + a1, 20, 0 + b1));
	fe1->setMaterialFlag(video::EMF_LIGHTING, false);
	fe1->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	//-----------------------trzecia noga
	a2 = 0;
	b2 = 24;

	IAnimatedMeshSceneNode * cx2 = menage->addAnimatedMeshSceneNode(coxa);
	cx2->setScale(core::vector3df(1, 1, 1));
	cx2->setPosition(core::vector3df(-9 + a2, 18, 0 + b2));
	cx2->setRotation(core::vector3df(0, 0, 270));
	cx2->setMaterialFlag(video::EMF_LIGHTING, false);
	cx2->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMeshSceneNode * vit2 = menage->addAnimatedMeshSceneNode(vitulus);
	vit2->setScale(core::vector3df(1, 1, 1));
	vit2->setRotation(core::vector3df(90, 90, 180));
	vit2->setPosition(core::vector3df(4.5 + a2, 24.5, 0 + b2));
	vit2->setMaterialFlag(video::EMF_LIGHTING, false);
	vit2->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMeshSceneNode * fe2 = menage->addAnimatedMeshSceneNode(femur);
	fe2->setScale(core::vector3df(1, 1, 1));
	fe2->setRotation(core::vector3df(90, 90, 90));
	fe2->setPosition(core::vector3df(0 + a2, 20, 0 + b2));
	fe2->setMaterialFlag(video::EMF_LIGHTING, false);
	fe2->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	//-----------------------------lewa strona
	//-------------------------------piewsza noga


	IAnimatedMeshSceneNode * cx3 = menage->addAnimatedMeshSceneNode(coxa);
	cx3->setScale(core::vector3df(1, 1, 1));
	cx3->setPosition(core::vector3df(-25, 18, 0));
	cx3->setRotation(core::vector3df(0, 0, 90));
	cx3->setMaterialFlag(video::EMF_LIGHTING, false);
	cx3->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMeshSceneNode * vit3 = menage->addAnimatedMeshSceneNode(vitulus);
	vit3->setScale(core::vector3df(1, 1, 1));
	vit3->setRotation(core::vector3df(90, 270, 180));
	vit3->setPosition(core::vector3df(-38, 23, 0));
	vit3->setMaterialFlag(video::EMF_LIGHTING, false);
	vit3->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMeshSceneNode * fe3 = menage->addAnimatedMeshSceneNode(femur);
	fe3->setScale(core::vector3df(1, 1, 1));
	fe3->setRotation(core::vector3df(270, 270, 90));
	fe3->setPosition(core::vector3df(-34, 20.5, 0));
	fe3->setMaterialFlag(video::EMF_LIGHTING, false);
	fe3->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));
	//-----------------------druga noga
	a4 = -4;
	b4 = 12;

	IAnimatedMeshSceneNode * cx4 = menage->addAnimatedMeshSceneNode(coxa);
	cx4->setScale(core::vector3df(1, 1, 1));
	cx4->setPosition(core::vector3df(-25 + a4, 18, 0 + b4));
	cx4->setRotation(core::vector3df(0, 0, 90));
	cx4->setMaterialFlag(video::EMF_LIGHTING, false);
	cx4->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMeshSceneNode * vit4 = menage->addAnimatedMeshSceneNode(vitulus);
	vit4->setScale(core::vector3df(1, 1, 1));
	vit4->setRotation(core::vector3df(90, 270, 180));
	vit4->setPosition(core::vector3df(-38 + a4, 23, 0 + b4));
	vit4->setMaterialFlag(video::EMF_LIGHTING, false);
	vit4->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMeshSceneNode * fe4 = menage->addAnimatedMeshSceneNode(femur);
	fe4->setScale(core::vector3df(1, 1, 1));
	fe4->setRotation(core::vector3df(270, 270, 90));
	fe4->setPosition(core::vector3df(-34 + a4, 20.5, 0 + b4));
	fe4->setMaterialFlag(video::EMF_LIGHTING, false);
	fe4->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));
	//------------------------trzecia noga
	a5 = 0;
	b5 = 24;

	IAnimatedMeshSceneNode * cx5 = menage->addAnimatedMeshSceneNode(coxa);
	cx5->setScale(core::vector3df(1, 1, 1));
	cx5->setPosition(core::vector3df(-25 + a5, 18, 0 + b5));
	cx5->setRotation(core::vector3df(0, 0, 90));
	cx5->setMaterialFlag(video::EMF_LIGHTING, false);
	cx5->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMeshSceneNode * vit5 = menage->addAnimatedMeshSceneNode(vitulus);
	vit5->setScale(core::vector3df(1, 1, 1));
	vit5->setRotation(core::vector3df(90, 270, 180));
	vit5->setPosition(core::vector3df(-38 + a5, 23, 0 + b5));
	vit5->setMaterialFlag(video::EMF_LIGHTING, false);
	vit5->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMeshSceneNode * fe5 = menage->addAnimatedMeshSceneNode(femur);
	fe5->setScale(core::vector3df(1, 1, 1));
	fe5->setRotation(core::vector3df(270, 270, 90));
	fe5->setPosition(core::vector3df(-34 + a5, 20.5, 0 + b5));
	fe5->setMaterialFlag(video::EMF_LIGHTING, false);
	fe5->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));


	//---------------korpus
	IAnimatedMesh * korpus = menage->getMesh("../../resources/korpus2.stl");
	IAnimatedMeshSceneNode * kor = menage->addAnimatedMeshSceneNode(korpus);
	kor->setScale(core::vector3df(1, 1, 1));
	kor->setRotation(core::vector3df(90, 90, 0));
	kor->setPosition(core::vector3df(-12, 19.5, 0));
	kor->setMaterialFlag(video::EMF_LIGHTING, false);
	kor->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	while (device->run())

	{
		video->beginScene(true, true, video::SColor(255, 0, 10, 200));
		menage->drawAll();
		video->endScene();
	}
	//device->drop();
	return 0;
}
