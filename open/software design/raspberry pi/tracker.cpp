#include "common.h"

int main()
{
    show_debug_windows = false;
    startup();

    std::thread trackOrange(trackColour, 0);
    std::thread trackYellow(trackColour, 1);
    std::thread trackBlue(trackColour, 2);
    std::thread getImage(getNewImage);
    std::thread transmit(transmitData);

    trackOrange.join();
    trackYellow.join();
    trackBlue.join();
    getImage.join();
    transmit.join();

    cam.stopVideo();
    cv::destroyAllWindows();
}
