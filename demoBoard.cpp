#include "include/defs/defs.h"
#include "include/board/board.h"
#include "include/board/boardDynamixel.h"
#include <iostream>
#include <stdio.h>
// test Ubuntu Limek
// Limek - Micha³ Limañski 123

// Limek
//pelka

using namespace std;

int main( int argc, const char** argv )
{
    try {
        Board* board = createBoardDynamixel();
         //Grabber* grabber;
         //grabber = createFileGrabber();
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
