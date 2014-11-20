/** @file kinematic.h
 *
 * Kinematic interface
 *
 */
 
 /**
* @author Adam Czeszejkowski
* @author Norbert Werbliñski
* @mainpage
*/

#ifndef _KINEMATIC_H_
#define _KINEMATIC_H_

#include "../defs/defs.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <string>
#include <vector>

namespace controller {
    /// Kinematic interface
    class Kinematic {
        public:

            /// Kinematic type
            enum Type {
                    /// Lie groups
                    TYPE_LIE,
                    /// Denavit-Hartenberg
                    TYPE_DENAVIT_HARTNBERG,
            };

            /// overloaded constructor
            Kinematic(const std::string _name, Type _type) : name(_name), type(_type) {};
			/** Kinematic constructor
			* 
			* Structure of the target XML file:
			* - Element "conf"
			* - - Element "linksNo"
			* - - Element "jointsNo"
			* - Element "parameters"
			* - - Element "JointX"        where X is the number of the joint(0 to jointsNo-1)
			* - - Element "g0"
			*/

            Kinematic(std::string configFilename, const std::string _name, Type _type) : name(_name), type(_type){
				std::string filename = "../../resources/" + configFilename;
				if (conf.FirstChildElement()==nullptr)
					std::cout << "unable to load Kinematic config file.\n";
				else {
					conf.LoadFile(filename.c_str());
					tinyxml2::XMLNode * pRoot = conf.FirstChildElement("parameters");
					float_type val;
					switch (_type)
					{
					case controller::Kinematic::TYPE_LIE:
						linksNo = std::stoi(conf.FirstChildElement("conf")->FirstChildElement("linksNo")->GetText());
						jointsNo = std::stoi(conf.FirstChildElement("conf")->FirstChildElement("jointsNo")->GetText());
						ksi = new std::vector<float_type>[linksNo];
						for (int i = 0; i < linksNo; i++)
						{
							std::string tmp = "Joint" + std::to_string(i);
							pElement = pRoot->FirstChildElement(tmp.c_str());
							pListElement = pElement->FirstChildElement("value");
							while (pListElement != nullptr)
							{
								parameters=pListElement->QueryDoubleText(&val);
								pListElement = pListElement->NextSiblingElement("value");
								ksi[i].push_back(val);
							}
						}
						pElement = pRoot->FirstChildElement("g0");
						pListElement = pElement->FirstChildElement("value");
						while (pListElement != nullptr)
						{
							parameters = pListElement->QueryDoubleText(&val);
							pListElement = pListElement->NextSiblingElement("value");
							g0.push_back(val);
						}
						for (int i = 0; i < linksNo; i++)
						{
							std::cout<<"Joint" + std::to_string(i)+": ";
							for (int j = 0; j < 6; j++)
							{
								std::cout << ksi[i][j] << " ";
							}
							std::cout << std::endl;
						}
						std::cout << "d0: ";
						for (int j = 0; j < 3; j++)
						{
							std::cout << g0[j] << " ";
						}
						std::cout << std::endl;
						break;
					case controller::Kinematic::TYPE_DENAVIT_HARTNBERG:
						break;
					}
				}				
            }
            
             /** Name of the kienematic model
             *
             * 
             *@return name Nazwa pliku
             *
             */
            virtual const std::string& getName() const { return name; }
            
            /** Compute forward kinematic, default (-1) -- the last joint
             *
             * 
             *@param [in] configuration Wektor wspolrzednych wewnetrznych
             *@param [in] linkNo Numer ogniwa
             *@return Macierz rotacji i translacji
             *
             */
            
            virtual Mat34 forwardKinematic(const std::vector<float_type>& configuration, unsigned int linkNo=-1) = 0;

             /** Compute inverse kinematic, default (-1) -- the last joint
              *
              * 
              *@param [in] linkPose Macierz translacji i rotacji
              *@param [in] linkNo Numer ogniwa
              *@return Wektor wspolrzednych konfiguracyjnych
              *
              */
            virtual std::vector<float_type> inverseKinematic(const Mat34& linkPose, unsigned int linkNo=-1) = 0;

             /** Return set of link's poses
             *
             * 
             *@param [in] configuration Wektor wspolrzednych wewnetrznych
             *@return Macierz rotacji i translacji
             *
             */
            virtual std::vector<Mat34> getState(const std::vector<float_type>& configuration) = 0;

            /// Virtual descrutor
            virtual ~Kinematic() {
				if (conf.FirstChildElement() != nullptr) delete[] ksi;
			}

        protected:
            /// Board type
            Type type;

            /// Board name
            const std::string name;

            /// number of joints
            unsigned int jointsNo;

            /// Number of links
            unsigned int linksNo;

			//ksi tables
			std::vector<float_type>* ksi;
			//g0 table
			std::vector<float_type> g0;

			tinyxml2::XMLError parameters;
			tinyxml2::XMLElement * pElement;
			tinyxml2::XMLElement * pListElement;
			tinyxml2::XMLDocument conf;
    };
};

#endif // _BOARD_H_
