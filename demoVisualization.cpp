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

	IrrlichtDevice *device =
		createDevice(video::EDT_SOFTWARE, dimension2d<u32>(640, 480), 16,
		false, false, false, 0);
	if (!device)
	return 1;

	video::IVideoDriver * video = device->getVideoDriver();
	scene::ISceneManager * menage = device->getSceneManager();
	scene::ICameraSceneNode * kam = menage->addCameraSceneNodeFPS();
	device->getCursorControl()->setVisible(true);
	kam->setPosition(core::vector3df(0, 0, -100));
	//Zasiêg pola widzenia kamery
	//kam->setFarValue( 90000 );
	//Wczytywanie modelu
	IAnimatedMesh * coxa = menage->getMesh("/home/tomlock/Desktop/ForceController/src/visualization/coxa.stl");
	IAnimatedMeshSceneNode * cx = menage->addAnimatedMeshSceneNode(coxa);
	IAnimatedMeshSceneNode * cx2 = menage->addAnimatedMeshSceneNode(coxa);
	cx->setScale(core::vector3df(-30, -30, -30));
	cx->setPosition(core::vector3df(0, 0, 0));
	cx->setMaterialFlag(video::EMF_LIGHTING, false);
	cx->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	cx2->setScale(core::vector3df(-30, -30, -30));
	cx2->setPosition(core::vector3df(0, 300, 0));
	cx2->setMaterialFlag(video::EMF_LIGHTING, false);
	cx2->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMesh * vitulus = menage->getMesh("/home/tomlock/Desktop/ForceController/src/visualization/vitulus.stl");
	IAnimatedMeshSceneNode * vit = menage->addAnimatedMeshSceneNode(vitulus);
	vit->setScale(core::vector3df(-30, -30, -30));
	vit->setRotation(core::vector3df(90, 90, 0));
	vit->setPosition(core::vector3df(200, 0, 0));
	vit->setMaterialFlag(video::EMF_LIGHTING, false);
	vit->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));

	IAnimatedMesh * femur = menage->getMesh("/home/tomlock/Desktop/ForceController/src/visualization/femur.stl");
	IAnimatedMeshSceneNode * fe = menage->addAnimatedMeshSceneNode(femur);
	fe->setScale(core::vector3df(-30, -30, -30));
	fe->setRotation(core::vector3df(90, 90, 0));
	fe->setPosition(core::vector3df(200, 300, 0));
	fe->setMaterialFlag(video::EMF_LIGHTING, false);
	fe->setMaterialTexture(0, video->getTexture("media/img/pudlo.png"));
	// wczytywanie korpusu
	IAnimatedMesh * korpus = menage->getMesh("/home/tomlock/Desktop/ForceController/src/visualization/korpus2.stl");
	IAnimatedMeshSceneNode * kor = menage->addAnimatedMeshSceneNode(korpus);
	kor->setScale(core::vector3df(-30, -30, -30));
	kor->setRotation(core::vector3df(90, 90, 0));
	kor->setPosition(core::vector3df(200, 600, 0));
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
