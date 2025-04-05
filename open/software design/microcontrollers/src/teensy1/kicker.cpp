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
    digitalWriteFast(23, HIGH);         // Start the pulse
    delayMicroseconds(2500);     // Wait for 1ms (pulse width for 0 degrees)
    digitalWriteFast(23, LOW);

    delay(300);

    robot.base.motorOut(1, 0);
    robot.base.motorOut(2, 0);
    robot.base.motorOut(3, 0);
    robot.base.motorOut(4, 0);

    delay(1000);
    
    digitalWriteFast(23, HIGH);
    delayMicroseconds(500);     // Wait for 1ms (pulse width for 0 degrees)
    digitalWriteFast(23, LOW);
    delay(1500);
    
    digitalWriteFast(23, HIGH);
    delayMicroseconds(2300);     // Wait for 1ms (pulse width for 0 degrees)
    digitalWriteFast(23, LOW);

    delay(1000);
}