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
KinematicLie::Ptr kinematicLie;

InsectLeg::InsectLeg(void) : Leg("Insect Leg", TYPE_INSECT) 
{

}

InsectLeg::~InsectLeg(void) 
{

}

/** Compute torque in each joint for given the force applied in the foot
* @param [in] force Wskaznik na wektor sil dzialajacych w osiach x, y i z
*/
std::vector<float_type> InsectLeg::computLoad(Vec3& force)
{
  std::vector<float_type> result;

  return result;
}

/** Compute forward kinematic, default (-1) -- the last joint
* @param [in] configuration konfiguracja nogi
* @param [in] linkNo liczba wezlow kinematycznych
*/
Mat34 InsectLeg::forwardKinematic(std::vector<float_type> configuration, unsigned int linkNo)
{
  return kinematicLie->forwardKinematic(configuration, linkNo);
}

/** Compute inverse kinematic, default (-1) -- the last joint
* @param [in] linkPose macierz jednorodna nogi
* @param [in] linkNo liczba wezlow kinematycznych
*/
std::vector<float_type> InsectLeg::inverseKinematic(Mat34 linkPose, unsigned int linkNo)
{
  return kinematicLie->inverseKinematic(linkPose, linkNo);
}

controller::Leg* controller::createInsectLeg(void) 
{
  insectLeg.reset(new InsectLeg());
  return insectLeg.get();
}

controller::Leg* controller::createInsectLeg(std::string filename) 
{
  insectLeg.reset(new InsectLeg(filename));
  return insectLeg.get();
}
