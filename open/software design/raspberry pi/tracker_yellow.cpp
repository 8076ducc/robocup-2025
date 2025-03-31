#include "common.h"

int main()
{
    show_debug_windows = false;
    startup();

    double fps = 0;
    while (true)
    {
	auto tStartSteady = std::chrono::steady_clock::now();
        getNewImage();
        std::thread trackYellow(trackColour, 1);
        trackYellow.join();
	transmitData();
	auto tEndSteady = std::chrono::steady_clock::now();
	std::chrono::nanoseconds diff = tEndSteady - tStartSteady;
	fps = 0.9 * fps + 0.1 * (1000000000 / diff.count());
	std::cout << fps << std::endl;
    }

    shutdown();
}
