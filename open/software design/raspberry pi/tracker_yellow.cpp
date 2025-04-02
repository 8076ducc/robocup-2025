#include "common.h"

int main()
{
    show_debug_windows = true;
    startup();
    
    std::thread trackYellow(trackColour, 1);
    std::thread getImage(getNewImage);
    std::thread transmit(transmitData);

    trackYellow.join();
    getImage.join();
    transmit.join();    
    
    shutdown();
}
