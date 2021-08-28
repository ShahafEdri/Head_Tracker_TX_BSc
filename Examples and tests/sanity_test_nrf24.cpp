#include "Arduino.h"

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

RF24 radio(7,8);

void setup()
{
 while (!Serial);
 Serial.begin(9600);
 printf_begin();
 radio.begin();
 radio.setDataRate(RF24_2MBPS);
 radio.setPALevel(RF24_PA_MIN);
}

void loop()
{
 radio.printDetails();
 delay(10000);
}
