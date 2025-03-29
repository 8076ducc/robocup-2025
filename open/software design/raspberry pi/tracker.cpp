#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <unistd.h>
#include <wiringPi.h>
#include <iostream>
#define byte uint8_t

#include "serial.h"
#include "threads.h"

int video_x = 864;
int video_y = 864;
lccv::PiCamera cam;	

bool tune_orange = true;
bool tune_yellow = false;
bool tune_blue = false;

void getNewImage()
{
    double fps = 0;

    while (true)
    {
        auto tStartSteady = std::chrono::steady_clock::now();

        imageStatus = cam.getVideoFrame(unsizedImage, 1000);
	//cv::circle(unsizedImage, cv::Point(500, 500), 2, CV_RGB(255, 0, 0), -1);
	//cv::circle(unsizedImage, cv::Point(514, 513), 1, CV_RGB(0, 255, 0), -1);
	//cv::circle(unsizedImage, cv::Point(514, 513), 35, CV_RGB(0, 255, 0), -1);
	//cv::imshow("image", unsizedImage);

        new_orange_frame = true;
        new_yellow_frame = true;
        new_blue_frame = true;
        
        auto tEndSteady = std::chrono::steady_clock::now();
        std::chrono::nanoseconds diff = tEndSteady - tStartSteady;
        fps = 0.9 * fps + 0.1 * (1000000000 / diff.count());
        // std::cout << fps << std::endl;
    }
}

void transmitData()
{
    while (true)
    {
        serialWrite(tx_data.bytes, sizeof(tx_data.bytes));
    }
}

void receiveData()
{	
    while (true)
    {
        serialRead();
        
        //if (tune_orange) {
		//	rx_data.track_orange = true;
		//} else {
		//	rx_data.track_orange = false;
		//}
		
		//if (tune_yellow) {
		//	rx_data.track_yellow = true;
		//} else {
		//	rx_data.track_yellow = false;
		//}
		
		//if (tune_blue) {
		//	rx_data.track_blue = true;
		//} else {
		//	rx_data.track_blue = false;
		//}
    }
}

int main()
{
    setUpSerial();
    cam.options->video_width = video_x;
    cam.options->video_height = video_y;
    cam.options->framerate = 120;
    cam.options->verbose = true;
    // cam.options->af_index = AutoFocus_Modes::AF_MANUAL;
    cam.options->lens_position = 20.0f;

    cam.startVideo();
    imageStatus = cam.getVideoFrame(unsizedImage, 1000);
    new_orange_frame = false;
    new_yellow_frame = false;
    new_blue_frame = false;
    
    
    pinMode(23, OUTPUT);

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
        
        if (show_debug_windows) {
			cv::waitKey(1);
		}
		
		
		
    }

    cam.stopVideo();
    cv::destroyAllWindows();
}
