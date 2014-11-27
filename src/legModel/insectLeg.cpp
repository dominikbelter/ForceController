/** @file insectLeg.cpp 
*
* @author Lukasz Mejlun
* @author Wojciech Nowakowski
*/

#include "../include/legModel/insectLeg.h"
#include "../include/kinematic/kinematicLie.h"
#include <iostream>

using namespace controller;

/// A single instance of insect leg
InsectLeg::Ptr insectLeg;
//KinematicLie::Ptr kinematicLie;

InsectLeg::InsectLeg(void) : Leg("Insect Leg", TYPE_INSECT) 
{

}

InsectLeg::~InsectLeg(void) 
{

}

/** Compute torque in each joint for given the force applied in the foot
* @param [in] force Wskaznik na wektor sil dzialajacych w osiach x, y i z
* @return std::vector<float_type> wektor obciazen w poszczegolnych wezlach
*/
std::vector<float_type> InsectLeg::computLoad(Vec3& force)
{
  std::vector<float_type> result;

  return result;
}

/** Compute forward kinematic, default (-1) -- the last joint
* @param [in] configuration zmienne konfiguracyjne nogi
* @param [in] linkNo liczba wezlow kinematycznych
* @return Mat34 macierz jednorodna nogi
*/
Mat34 InsectLeg::forwardKinematic(std::vector<float_type> configuration, int linkNo)
{
	Kinematic* demoKine;
	demoKine = createKinematicLie("../resources/legModel.xml");
	return demoKine->forwardKinematic(configuration, linkNo);
	//return kinematicLie->forwardKinematic(configuration, linkNo);
}

/** Compute inverse kinematic, default (-1) -- the last joint
* @param [in] linkPose macierz jednorodna nogi
* @param [in] linkNo liczba wezlow kinematycznych
* @return std::vector<float_type> zmienne konfiguracyjne nogi
*/
std::vector<float_type> InsectLeg::inverseKinematic(Mat34 linkPose, int linkNo)
{
	Kinematic* demoKine;
	demoKine = createKinematicLie("../resources/legModel.xml");
	return demoKine->inverseKinematic(linkPose, linkNo);
	//return kinematicLie->inverseKinematic(linkPose, linkNo);
}

/** Bezargumentowy konstruktor obiektu typu Leg*
 * @return controller::Leg* wskaznik na obiekt typu Leg
 */
controller::Leg* controller::createInsectLeg(void) 
{
  insectLeg.reset(new InsectLeg());
  return insectLeg.get();
}

/** Konstruktor obiektu typu Leg* przyjmujacy za argument polozenie pliku konfiguracyjnego typu xml
 * @param [in] filename wzgledna sciezka dostepu do pliku
 * @return controller::Leg* wskaznik na obiekt typu Leg
 */
controller::Leg* controller::createInsectLeg(std::string filename) 
{
  insectLeg.reset(new InsectLeg(filename));
  return insectLeg.get();
}
