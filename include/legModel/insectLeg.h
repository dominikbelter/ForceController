/** @file insectLeg.h
*
* Leg interface for insects
*
* @author Lukasz Mejlun
* @author Wojciech Nowakowski
*/

#ifndef _INSECTLEG_H_INCLUDED
#define _INSECTLEG_H_INCLUDED

#include "leg.h"
#include <iostream>
#include <chrono>
#include <memory>
#include <mutex>

namespace controller 
{
	/// create a single leg model (insect type)
	Leg* createInsectLeg(void);
	/// create a single leg model (insect type) and load configuration from file
	Leg* createInsectLeg(std::string filename);
};

using namespace controller;

/// Insect Leg implementation
class InsectLeg : public Leg 
{
	public:
		/// Pointer
		typedef std::unique_ptr<InsectLeg> Ptr;
	
		/** Bezargumentowy konstruktor obiektu typu Leg*
		 * @return controller::Leg* wskaznik na obiekt typu Leg
		 */
		InsectLeg(void);
			
		/** Konstruktor obiektu typu Leg* przyjmujacy za argument polozenie pliku konfiguracyjnego typu xml
		 * @param [in] filename wzgledna sciezka dostepu do pliku
		 * @return controller::Leg* wskaznik na obiekt typu Leg
		 */
		InsectLeg(std::string configFilename) : Leg(configFilename, "Insect Leg", TYPE_INSECT)
		{
		  tinyxml2::XMLDocument config;
		  std::string filename = "../../resources/" + configFilename;
		  config.LoadFile(filename.c_str());
		  if (config.ErrorID())
		  {
		      std::cout << "unable to load Kinematic config file.\n";
		  }
		  else
		  {
              tinyxml2::XMLElement * parameters = config.FirstChildElement( "conf" );
		      int param;
		      parameters->QueryIntAttribute("linksNo", &param); linksNo = param;
		      parameters->QueryIntAttribute("jointsNo", &param); jointsNo = param;

		      float_type paramf;
		      parameters = config.FirstChildElement("Link0");
		      parameters = parameters->FirstChildElement( "parameters" );
		      parameters->QueryDoubleAttribute("length", &paramf); lengths[0] = paramf;
		      parameters = config.FirstChildElement("Link1");
		      parameters = parameters->FirstChildElement( "parameters" );
		      parameters->QueryDoubleAttribute("length", &paramf); lengths[1] = paramf;
		      parameters = config.FirstChildElement("Link2");
		      parameters = parameters->FirstChildElement( "parameters" );
		      parameters->QueryDoubleAttribute("length", &paramf); lengths[2] = paramf;

		      std::cout << "links no: " << linksNo << " joints no: " << jointsNo << "\n";
		      std::cout << "Lenght1: " << lengths[0] << std::endl;
		      std::cout << "Lenght2: " << lengths[1] << std::endl;
		      std::cout << "Lenght3: " << lengths[2] << std::endl;
		  }
		}
	
		/// Destructor
		~InsectLeg(void);
	
		/// Name of the leg model
		const std::string& getName() const { return name; }

		/** Compute torque in each joint for given the force applied in the foot
		* @param [in] force Wskaznik na wektor sil dzialajacych w osiach x, y i z
		* @return std::vector<float_type> wektor obciazen w poszczegolnych wezlach
		*/
		std::vector<float_type> computLoad(Vec3& force);

		/** Compute forward kinematic, default (-1) -- the last joint
		* @param [in] configuration zmienne konfiguracyjne nogi
		* @param [in] linkNo liczba wezlow kinematycznych
		* @return Mat34 macierz jednorodna nogi
		*/
		Mat34 forwardKinematic(std::vector<float_type> configuration, unsigned int linkNo = -1);

		/** Compute inverse kinematic, default (-1) -- the last joint
		* @param [in] linkPose macierz jednorodna nogi
		* @param [in] linkNo liczba wezlow kinematycznych
		* @return std::vector<float_type> zmienne konfiguracyjne nogi
		*/
		std::vector<float_type> inverseKinematic(Mat34 linkPose, unsigned int linkNo = -1);

	private:
		/// number of joints
		unsigned int jointsNo;

                /// number of links
                unsigned int linksNo;

                /// lengths of legs
                float_type lengths[3];

};


#endif	// _INSECTLEG_H_INCLUDED
