#ifndef MYEVENTRECEIVER_H
#define MYEVENTRECEIVER_H

#include "irrlicht.h"


using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
using namespace std;

class MyEventReceiver : public IEventReceiver
{
public:

    MyEventReceiver();

    /** This method is triggered when the event occurs.
    * @param const SEvent& event
    */
    virtual bool OnEvent(const SEvent& event);

    /** This is used to check whether a key is being held down
    * @param EKEY_CODE keyCode
    */
    virtual bool IsKeyDown(EKEY_CODE keyCode) const;




private:
    bool KeyIsDown[KEY_KEY_CODES_COUNT];
};

#endif // MYEVENTRECEIVER_H
