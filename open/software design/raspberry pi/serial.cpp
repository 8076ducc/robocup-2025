#include <stdint.h> // uint8_t, etc.
#include <stddef.h> // size_t
#include <wiringPi.h>
#include <wiringSerial.h>

#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <unistd.h>
#define byte uint8_t

int main() 
{
	wiringPiSetup();
    wiringPiSetupGpio();

    int serialDeviceId = serialOpen("/dev/ttyAMA0", 9600);

    if (serialDeviceId < 0)
    {
        std::cout << "unable to open serial device" << std::endl;
        return 0;
    }

    if (wiringPiSetup() == -1)
    {
        std::cout << "unable to start wiringPi" << std::endl;
        return 0;
    }
        
    while (true) {
    serialPutchar(serialDeviceId, 'a'); // a is 97 in decimal
    
	//std::cout << serialDataAvail(serialDeviceId) << std::endl;
		std::cout << serialGetchar(serialDeviceId) << std::endl;}
}
