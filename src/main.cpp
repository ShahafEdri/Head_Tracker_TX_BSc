#include "Arduino.h"

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>
#include <SPI.h>

#define STM32F1
// #define ARDUINO_NANO

/*-----( Declare Constants and Pin Numbers )-----*/
#ifdef STM32F1
#define  CE_PIN  PB1   // The pins to be used for CE and SN
#define  CSN_PIN PB0
#endif

#ifdef ARDUINO_NANO // arduino pins
#define  CE_PIN  7   // The pins to be used for CE and SN
#define  CSN_PIN 8
#endif

RF24 radio(CE_PIN, CSN_PIN);
bool var;

void setup()
{
    while (!Serial);
    Serial.begin(9600);
    printf_begin();
    radio.begin();
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MIN);
    radio.printDetails();
    Serial.print("\n\n\n\n\n");
}

void loop()
{
    // radio.isChipConnected();
    if ((var = radio.isChipConnected() == true))
        Serial.print("nRFl01+ FOUND!\n\n");
        // radio.printDetails();
    else
        Serial.print("NO CHIP!\n\n");
    // Serial.print("\n\n\n\n\n");
    

    delay(500);
}
