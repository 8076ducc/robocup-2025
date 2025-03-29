#include "common.h"

int main()
{
    show_debug_windows = true;
    std::ofstream values;

    startup();

    while (true)
    {
        // digitalWrite(23, HIGH);

        std::thread trackBlue(trackColour, 2);
        std::thread getImage(getNewImage);
        std::thread transmit(transmitData);
        std::thread receive(receiveData);

        trackBlue.join();
        getImage.join();
        transmit.join();
        receive.join();
    }

    cam.stopVideo();
    cv::destroyAllWindows();
}
