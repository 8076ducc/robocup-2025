#include "common.h"

int main()
{
    show_debug_windows = true;
    std::ofstream values;

    startup();

    while (true)
    {
        // digitalWrite(23, HIGH);

        std::thread trackYellow(trackColour, 1);
        std::thread getImage(getNewImage);
        std::thread transmit(transmitData);
        std::thread receive(receiveData);

        trackYellow.join();
        getImage.join();
        transmit.join();
        receive.join();

        if (show_debug_windows)
        {
            cv::waitKey(1);
        }
    }

    // write colour values to file with line break
    values.open("values/yellow.txt", std::ios::app);
    values << colour.threshold[0] << " " << colour.threshold[1] << " "
           << colour.threshold[2] << " " << colour.threshold[3] << " "
           << colour.threshold[4] << " " << colour.threshold[5] << "\n";
    values.close();

    cam.stopVideo();
    cv::destroyAllWindows();
}
