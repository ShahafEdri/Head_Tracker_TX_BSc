#include "Arduino.h"

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

#define STM32F1

/*-----( Declare Constants and Pin Numbers )-----*/
#ifdef STM32F1
    #define  CE_PIN  PB1   // The pins to be used for CE and SN
    #define  CSN_PIN PB0
#elif #defined ARDUINO_NANO // arduino pins
    #define  CE_PIN  7   // The pins to be used for CE and SN
    #define  CSN_PIN 8
#endif

RF24 radio(CE_PIN, CSN_PIN);

void setup() 
{
    while(radio.isChipConnected() == false)
        Serial.println("chip is NOT connected");

    Serial.println("chip not connected");
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
