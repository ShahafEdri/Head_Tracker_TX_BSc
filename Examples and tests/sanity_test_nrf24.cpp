#include "Arduino.h"

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

#define DEBUG
#define STM32F1
// #define ARDUINO_NANO

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

#define chip_connected_validation
// #define print_detailes

void loop()
{
    #ifdef chip_connected_validation
        if(radio.isChipConnected() == true)
            Serial.println("nRF24 module found");
        else
            Serial.println("nRF24 is NOT connected");
    #endif

    #ifdef print_detailes
        radio.printDetails();
        delay(10000);
    #endif
}
