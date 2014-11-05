/** @file demoLeg.cpp
*
* @author Lukasz Mejlun
* @author Wojciech Nowakowski
*/

#include "include/defs/defs.h"
#include "include/legModel/insectLeg.h"
#include <iostream>
#include <stdio.h>

using namespace std;

int main( int argc, const char** argv )
{
  try
  {
    cout << "DEMO STEROWNIKA NOGI" << "\n";

    Leg* legModel;
    legModel = createInsectLeg("../resources/legModel.xml");
    std::cout << "Leg type: " << legModel->getName() << "\n";
    getchar();
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
    return 1;
  }
  return 0;
}
