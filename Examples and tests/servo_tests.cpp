#include <Arduino.h>
#include <RF24.h>
#include "Servo.h"

#define DEBUG
#define STM32F1
// #define ARDUINO_NANO

// #define TESTBYDEGREES
// #define MAXRANGEDEGREES 180   // SG90 MAX is 2400 by datasheet
// #define MINRANGEDEGREES 0    // SG90 MIN is  500 by datasheet
#define TESTBYMICROSECONDS // higher resolution, good for testing servo resolution
#define MAXRANGEMICROSECONDS 2400   // SG90 MAX is 2400 by datasheet
#define MINRANGEMICROSECONDS 500    // SG90 MIN is  500 by datasheet


/*-----( Declare Constants and Pin Numbers )-----*/
#ifdef STM32F1
#define servo_yaw_pin A1
#define servo_pitch_pin A0
#elif #defined ARDUINO_NANO // arduino pins
#define servo_yaw_pin 2
#define servo_pitch_pin 3
#endif

Servo servo_yaw;
Servo servo_pitch;

uint8_t data;

void setup()
{
    // put your setup code here, to run once:
    servo_yaw.attach(servo_yaw_pin);     //links the yaw servo pin to the library
    servo_pitch.attach(servo_pitch_pin); //links the pitch servo pin to the library

    data = 90;
    servo_yaw.write(90);
    servo_pitch.write(90);

#ifdef TESTBYMICROSECONDS
    data = map(90, 0, 180, 500, 2400);
#endif

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    // put your main code here, to run repeatedly:
    // static uint8_t data=90;
    data++;
#ifdef TESTBYDEGREES
    if (data > MAXRANGEDEGREES)
        data = MINRANGEDEGREES;
    servo_yaw.write(data);
    servo_pitch.write(data);
#elif defined(TESTBYMICROSECONDS) // for more accurate testing
    if (data > MAXRANGEMICROSECONDS)
        data = MINRANGEMICROSECONDS;
    servo_yaw.writeMicroseconds(data);
    servo_pitch.writeMicroseconds(data);
#endif
    delay(20);
}