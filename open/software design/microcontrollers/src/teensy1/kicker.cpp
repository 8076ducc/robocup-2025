#include "main.h"

#include <PWMServo.h>

const int minPulseWidth = 500; // 1ms pulse for 0 degrees
const int maxPulseWidth = 2500; // 2ms pulse for 180 degrees

PWMServo servo;

void Kicker::setup()
{
    // servo.attach(23);
    pinMode(servoPin, OUTPUT);
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(1600);     // Wait for 1ms (pulse width for 0 degrees)
    digitalWrite(servoPin, LOW);
    // servo.write(99);
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
    Serial.println(kicked);
    if (kicked) {
        digitalWriteFast(servoPin, HIGH);
        delayMicroseconds(2100);     // Wait for 1ms (pulse width for 0 degrees)
        digitalWriteFast(servoPin, LOW);
        delay(1200);
        digitalWriteFast(servoPin, HIGH);
        delayMicroseconds(500);     // Wait for 1ms (pulse width for 0 degrees)
        digitalWriteFast(servoPin, LOW);
        delay(3000);
        digitalWriteFast(servoPin, HIGH);
        delayMicroseconds(2000);     // Wait for 1ms (pulse width for 0 degrees)
        digitalWriteFast(servoPin, LOW);
        delay(3000);
        kicked=false;
        delay(2000);
        kicked=false;
        // unsigned long currentMillis = millis();  // Get the current time
        // switch (servo_count) {
        //     case 0:
        //     servoPosition = 180;
        //     interval = 1200;  // Time for the first position
        //     break;
        //     case 1:
        //     servoPosition = 0;
        //     interval = 1000;   // Time for the second position
        //     break;
        //     case 2:
        //     servoPosition = 99;
        //     interval = 10000;   // Time for the third position
        //     kicked = false;  // Reset the kicked state
        //     break;
        //     default:
        //     servo_count = 0;  // Reset servo_count if it's out of bounds
        //     break;
        // }
        
        // // Check if the interval has passed
        // if (currentMillis - previousMillis >= interval) {
        //     // digitalWrite(servoPin, HIGH);         // Start the pulse
        //     // delayMicroseconds(map(servoPosition, 0, 180, 500, 2500));     // Wait for 1ms (pulse width for 0 degrees)
        //     // digitalWrite(servoPin, LOW);
        //     servo.write(servoPosition);  // Update the servo position
        //     previousMillis = currentMillis;  // Save the last time the position was updated
        //     servo_count++;  // Move to the next case
        //     if (servo_count > 3) { 
        //         kicked = false;  // Reset the kicked state
        //     }
        // }
    } else {
        // digitalWrite(13, HIGH);
        // digitalWrite(servoPin, HIGH);         // Start the pulse
        // delayMicroseconds((maxPulseWidth+minPulseWidth)/2);     // Wait for 1ms (pulse width for 0 degrees)
        // digitalWrite(servoPin, LOW);
    }
    // delay(20);
}