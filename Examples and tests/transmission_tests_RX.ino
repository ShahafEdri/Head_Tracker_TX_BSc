#include "Arduino.h"
#include "SPI.h"
#include "RF24.h"
#include "Servo.h"

#define STM32F1
// #define ARDUINO_NANO

/*-----( Declare Constants and Pin Numbers )-----*/
#ifdef STM32F1 // stm32 pins
// The pins to be used for CE and SN
    #define  CE_PIN  PB1
    #define  CSN_PIN PB0
// The pins to be used servo pins
    #define servo_yaw_pin A1
    #define servo_pitch_pin A0
#elif #defined ARDUINO_NANO // arduino pins
// The pins to be used for CE and SN
    #define  CE_PIN  7   
    #define  CSN_PIN 8
// The pins to be used servo pins
    #define servo_yaw_pin 2
    #define servo_pitch_pin 3
#endif

Servo servo_yaw;
Servo servo_pitch;

/*-----( Declare objects )-----*/
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus (usually) pins 7 & 8 (Can be changed) */
RF24 radio(CE_PIN, CSN_PIN);

/*-----( Declare Variables for RF24 pipes )-----*/
byte addresses[][6] = {"1Node", "2Node"}; // These will be the names of the "Pipes"

/*-----( Declare Variables for gimbal servos)-----*/


/**
  Data Structure for data
  Create a data structure for transmitting and receiving data
  This allows many variables to be easily sent and received in a single transmission
  See http://www.cplusplus.com/doc/tutorial/structures/
*/
struct dataStruct
{
  unsigned long _micros;  // to save response times
  int yaw;          // The Joystick position values
  int pitch;
  bool switchOn;          // The Joystick push-down switch
} myData;                 // This can be accessed in the form:  myData.Xposition  etc.

/*
  Data Structure for acknowledge FIX_ME need to modify RX structure
  Create a data structure for transmitting and receiving Dynamic Acknowledge
  This allows many variables to be easily sent back to receiver in a single transmission
*/
struct ackStruct
{
  unsigned long _micros;  // to save response times
  int VBattery_HeadTracker;          // The Joystick position values
  int VBattery_Plane;
  bool Signal_Loss;          // The Joystick push-down switch
  int check;			  // first check of the code FIX_ME change me to right variable
} myAck;

//The setup function is called once at startup of the sketch
void setup()
{
// Add your initialization code here

	Serial.begin(115200);  // MUST reset the Serial Monitor to 115200 (lower right of window )
	  // NOTE: The "F" in the print statements means "unchangable data; save in Flash Memory to conserve SRAM"
	  Serial.println(F("YourDuino.com Example: Send joystick data by nRF24L01 radio to another Arduino"));

	  /*-----( RF24 Initialize and preferences )-----*/
	  radio.begin();          // Initialize the nRF24L01 Radio
	  radio.setChannel(108);  // Above most WiFi frequencies
	  radio.setDataRate(RF24_250KBPS); // Fast enough.. Better range

	  // Set the Power Amplifier Level low to prevent power supply related issues since this is a
	  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
	  // PALevelcan be one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
	  radio.setPALevel(RF24_PA_MIN);

	 radio.enableAckPayload();
	 radio.enableDynamicAck();
	 radio.setAutoAck(true);

	 radio.openReadingPipe(1, addresses[0]);
	 radio.startListening();

	 servo_yaw.attach(servo_yaw_pin);//links the yaw servo pin to the library
	 servo_pitch.attach(servo_pitch_pin);//links the pitch servo pin to the library

	 servo_yaw.write(90);
	 servo_pitch.write(90);
}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here

  if (radio.available()) // if there is data ready to be retrieved from the receive pipe
	{
	  radio.writeAckPayload(1, &myAck, sizeof(myAck));	//send the data back as acknowledge
	  radio.read( &myData, sizeof(myData) );         	// Get the data
	// END radio available

		delayMicroseconds(30);
		if(Serial)
		{
			/*-----( print the received packet data )-----*/
			Serial.print(F("Packet Received - Sent response "));  // Print the received packet data
			Serial.print(myData._micros);
			Serial.print(F("uS\n Yaw= "));//x state
			Serial.print(myData.yaw);
			Serial.print(F("\t Pitch= "));//y state
			Serial.print(myData.pitch);
			Serial.println();
			if ( myData.switchOn == 1)//switch state
			{
			  Serial.println(F("\t Switch ON"));
			}
			else
			{
			  Serial.println(F("\t Switch OFF"));
			}
		}
	myAck.check = 123;
	}// END radio available

  servo_yaw.write(myData.yaw);
  servo_pitch.write(myData.pitch);

}


