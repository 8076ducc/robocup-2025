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

unsigned long previousMillis = 0; // Store the last time the servo position was updated
unsigned long interval = 0;       // Interval for the current servo action
int servoPosition = 0;            // Store current servo position
int servo_count = 0;                    // servo_counter to keep track of the actions


void Kicker::reset()
{
    if (kicked) {
        unsigned long currentMillis = millis();  // Get the current time
        switch (servo_count) {
            case 0:
            servoPosition = 175;
            interval = 2000;  // Time for the first position
            break;
            case 1:
            servoPosition = 110;
            interval = 700;   // Time for the second position
            break;
            case 2:
            servoPosition = 180;
            interval = 700;   // Time for the third position
            break;
            case 3:
            servoPosition = 0;
            interval = 1000;  // Time for the fourth position
            break;
            default:
            servo_count = 0;  // Reset servo_count if it's out of bounds
            break;
        }
        
        // Check if the interval has passed
        if (currentMillis - previousMillis >= interval) {
            digitalWrite(servoPin, HIGH);         // Start the pulse
            delayMicroseconds(map(servoPosition, 0, 180, 500, 2500));     // Wait for 1ms (pulse width for 0 degrees)
            digitalWrite(servoPin, LOW);
            // servo.write(servoPosition);  // Update the servo position
            previousMillis = currentMillis;  // Save the last time the position was updated
            servo_count++;  // Move to the next case
            if (servo_count > 3) {  // If all cases have been executed, reset the count
                servo_count = 0;
                kicked = false;  // Reset the kicked state
            }
        }
    } else {
        // digitalWrite(13, HIGH);
        // digitalWrite(servoPin, HIGH);         // Start the pulse
        // delayMicroseconds((maxPulseWidth+minPulseWidth)/2);     // Wait for 1ms (pulse width for 0 degrees)
        // digitalWrite(servoPin, LOW);
    }
    delay(20);
}