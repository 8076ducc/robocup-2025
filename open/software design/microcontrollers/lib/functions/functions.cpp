#include "functions.h"

double correctBearing(double bearing)
{
    if (bearing > 360)
    {
        bearing = bearing - 360;
    }
    else if (bearing < 0)
    {
        bearing = bearing + 360;
    }

    return bearing;
}

int sgn(double val)
{
    if (val > 0)
    {
        return 1;
    }
    else if (val < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

double bound(double value, double low, double high) {
    return min(max(value, low), high);
}

double xyToBearing(int x, int y)
{
    double principal_angle = degrees(atan2(y, x));
    return correctBearing(fmod(450 - principal_angle, 360));
}