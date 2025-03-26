#include "main.h"

const int minPulseWidth = 500; // 1ms pulse for 0 degrees
const int maxPulseWidth = 2500; // 2ms pulse for 180 degrees

bool kicked = false;

void Kicker::setup()
{
    pinMode(servoPin, OUTPUT);
}

void Kicker::kick()
{
    if (!kicked)
    {
        kicked = true;
        time_kicked = millis();
    }
}

void Kicker::reset()
{
    if (kicked) {
        // digitalWrite(13, LOW);
        if (millis() - time_kicked < 2000){
            digitalWrite(servoPin, HIGH);         // Start the pulse
            delayMicroseconds(minPulseWidth);     // Wait for 1ms (pulse width for 0 degrees)
            digitalWrite(servoPin, LOW);
        } else {
            kicked=false;
        }
    } else{
        // digitalWrite(13, HIGH);
        digitalWrite(servoPin, HIGH);         // Start the pulse
        delayMicroseconds((maxPulseWidth+minPulseWidth)/2);     // Wait for 1ms (pulse width for 0 degrees)
        digitalWrite(servoPin, LOW);
    }
}