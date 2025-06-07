#include "common.h"

int main()
{
    show_debug_windows = true;
    startup();
    
    std::thread trackOrange(trackColour, 0);
    std::thread getImage(getNewImage);
    std::thread transmit(transmitData);

    trackOrange.join();
    getImage.join();
    transmit.join();
    
    shutdown();
}
