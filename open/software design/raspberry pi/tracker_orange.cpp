#include "common.h"

int main()
{
    show_debug_windows = true;
    startup();

    while (true)
    {
        // digitalWrite(23, HIGH);

        std::thread trackOrange(trackColour, 0);
        std::thread getImage(getNewImage);
        std::thread transmit(transmitData);
        // std::thread receive(receiveData);

        trackOrange.join();
        getImage.join();
        transmit.join();
        // receive.join();
    }

    cam.stopVideo();
    cv::destroyAllWindows();
}
