#include "common.h"

int main()
{
    show_debug_windows = true;
    startup();
    
    std::thread trackBlue(trackColour, 2);
    std::thread getImage(getNewImage);
    std::thread transmit(transmitData);

    trackBlue.join();
    getImage.join();
    transmit.join();
    
    shutdown();
}
