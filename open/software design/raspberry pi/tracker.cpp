#include "common.h"

int main()
{
    show_debug_windows = false;
    startup();

    while (true)
    {
        // digitalWrite(23, HIGH);

        std::thread trackOrange(trackColour, 0);
        std::thread trackYellow(trackColour, 1);
        std::thread trackBlue(trackColour, 2);
        std::thread getImage(getNewImage);
        std::thread transmit(transmitData);
        std::thread receive(receiveData);

        trackOrange.join();
        trackYellow.join();
        trackBlue.join();
        getImage.join();
        transmit.join();
        receive.join();
    }

    cam.stopVideo();
    cv::destroyAllWindows();
}
