#ifndef LIGHT_RING_H
#define LIGHT_RING_H

#include "main.h"

class LightRing
{
public:
    void setup();
    void calibrate();
    void printThresholds();
    void read();

    int line_start_ldr;
    int line_end_ldr;

    int ldr_readings[32];

    int ldr_min_readings[32];
    int ldr_max_readings[32];

#ifdef BOT_A
    // TUNE THIS
    const int ldr_thresholds[32] = {
        1628, // 0
        1412, // 1
        1338, // 2
        1509, // 3
        1438, // 4
        1493, // 5
        1513, // 6
        1636, // 7
        1569, // 8
        1535, // 9
        1619, // 10
        1580, // 11
        1569, // 12
        1574, // 13
        1523, // 14
        1483, // 15
        1504, // 16
        1358, // 17
        1178, // 18
        1425, // 19
        1529, // 20
        1181, // 21
        1401, // 22
        1395, // 23
        1550, // 24
        1536, // 25
        1644, // 26
        1616, // 27
        1625, // 28
        1601, // 29
        1555, // 30
        1540, // 31
    };

#else
    const int ldr_thresholds[32] = {
        2087, // 0
        2181, // 1
        2173, // 2
        2182, // 3
        2252, // 4
        2236, // 5
        2241, // 6
        2268, // 7
        2172, // 8
        2212, // 9
        2250, // 10
        2284, // 11
        2328, // 12
        2255, // 13
        2308, // 14
        2314, // 15
        2127, // 16
        2058, // 17
        2072, // 18
        2033, // 19
        2082, // 20
        2043, // 21
        2064, // 22
        2110, // 23
        2143, // 24
        2100, // 25
        2103, // 26
        2093, // 27
        2050, // 28
        2075, // 29
        2075, // 30
        2090, // 31
    };
    // END TUNE

#endif

    const int mux_signals[16][4] = {
        {0, 0, 0, 0}, // 0
        {1, 0, 0, 0}, // 1
        {0, 1, 0, 0}, // 2
        {1, 1, 0, 0}, // 3
        {0, 0, 1, 0}, // 4
        {1, 0, 1, 0}, // 5
        {0, 1, 1, 0}, // 6
        {1, 1, 1, 0}, // 7
        {0, 0, 0, 1}, // 8
        {1, 0, 0, 1}, // 9
        {0, 1, 0, 1}, // 10
        {1, 1, 0, 1}, // 11
        {0, 0, 1, 1}, // 12
        {1, 0, 1, 1}, // 13
        {0, 1, 1, 1}, // 14
        {1, 1, 1, 1}  // 15
    };
};

const double ldr_angle = 11.25;

#endif