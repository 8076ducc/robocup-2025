#include "common.h"

int main()
{
    show_debug_windows = false;
    startup();

    while (true)
    {
        getNewImage();
        std::thread trackOrange(trackColour, 0);
        std::thread trackYellow(trackColour, 1);
        std::thread trackBlue(trackColour, 2);
        trackOrange.join();
        trackYellow.join();
        trackBlue.join();
        std::thread transmit(transmitData);
    }

    shutdown();
}
