/*	Autorzy:
		£ukasz Mej³un
		Wojciech Nowakowski
*/

#include "include/defs/defs.h"
#include "include/legModel/leg.h"
#include <iostream>
#include <stdio.h>

using namespace std;

int main( int argc, const char** argv )
{
	try 
	{
		//Grabber* grabber;
		//grabber = createFileGrabber();
		cout << "DEMO STEROWNIKA NOGI";
		getchar();
	}
  catch (const std::exception& ex) 
	{
		std::cerr << ex.what() << std::endl;
    return 1;
  }
  return 0;
}
