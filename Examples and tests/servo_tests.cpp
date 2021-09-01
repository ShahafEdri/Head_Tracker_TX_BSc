#include <Arduino.h>
#include <RF24.h>
#include "Servo.h"


#define STM32F1

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
    
uint8_t data=90;

void setup() {
// put your setup code here, to run once:
    servo_yaw.attach(servo_yaw_pin);//links the yaw servo pin to the library
    servo_pitch.attach(servo_pitch_pin);//links the pitch servo pin to the library

    servo_yaw.write(90);
    servo_pitch.write(90);

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
    // static uint8_t data=90;
    data++;
    if (data>140)
        data=20;
    servo_yaw.write(data);
    servo_pitch.write(data);
    delay(20);
}