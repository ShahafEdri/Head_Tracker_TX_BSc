// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

/*-----( Import needed libraries for radio NRF24 )-----*/
#include "RF24.h" // Download and Install (See above)
// #include <SPI.h>   // Comes with Arduino IDE

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

#define DEBUG
#define STM32F1
// #define ARDUINO_NANO
// #define WAIT_FOR_INPUT_CHAR_TO_START

#ifdef DEBUG
uint32_t counter = 0;
float_t counter_float = 0;
#endif

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTF(x, y) Serial.print(x, y)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTF(x, y)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLNF(x, y)
#endif

/*-----( Declare Constants and Pin Numbers )-----*/
#ifdef STM32F1 // stm32 pins
// The pins to be used for CE and SN
#define CE_PIN PB1
#define CSN_PIN PB0
/*-----( Declare Constants and Pin Numbers for mpu6050 IMU )-----*/
#define INTERRUPT_PIN PA2           // use pin 2 on Arduino Uno & most boards
#define LED_PIN LED_BUILTIN         // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define calibration_button_pin PC14 // the pin of the calibration button
#elif defined(ARDUINO_NANO)         // arduino pins
// The pins to be used for CE and SN
#define CE_PIN 7
#define CSN_PIN 8
/*-----( Declare Constants and Pin Numbers for mpu6050 IMU )-----*/
#define INTERRUPT_PIN 2          // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13               // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define calibration_button_pin 5 // the pin of the calibration button
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//NRF24 radio declaration data structure and pipe declarations
/*-----( Declare objects )-----*/
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus (usually) pins 7 & 8 (Can be changed) */
RF24 radio(CE_PIN, CSN_PIN);

/*-----( Declare Variables )-----*/
byte addresses[][6] = {"1Node", "2Node"}; // These will be the names of the "Pipes"

/*-----( received and sent time variables )-----*/
unsigned long timeNow; // Used to grab the current time, calculate delays
unsigned long started_waiting_at;
bool RESET_YAW; // switch state
bool timeout;   // Timeout? True or False

// Allows testing of radios and code without Joystick hardware. Set 'true' when joystick connected
//  boolean hasHardware = false;
bool hasHardware = true;

bool calibration_button = 0; //current state of the calibration switch

int Currentdeg = 0;
int situation = 1;
//  int Xc = 0;
//  int Xnc = 0;

int Xc_yaw = 0;
int Xnc_yaw = 0;

int Xc_pitch = 0;
int Xnc_pitch = 0;

/**
  Data Structure
  Create a data structure for transmitting and receiving data
  This allows many variables to be easily sent and received in a single transmission
  See http://www.cplusplus.com/doc/tutorial/structures/
*/
struct dataStruct
{
    unsigned long _micros; // to save response times
    int yaw;               // The Joystick position values
    int pitch;
    bool switchOn; // The Joystick push-down switch
} myData;          // This can be accessed in the form:  myData.Xposition  etc.

/*
  Data Structure for acknowledge FIX_ME need to modify TX structure
  Create a data structure for transmitting and receiving Dynamic Acknowledge
  This allows many variables to be easily sent back to receiver in a single transmission
*/
struct ackStruct
{
    unsigned long _micros; // to save response times
    int Xposition;         // The Joystick position values
    int Yposition;
    bool switchOn; // The Joystick push-down switch
    int check;     // first check of the code FIX_ME change me to right variable
} myAck;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

// ================================================================
// ===              functions to remove code load				===
// ================================================================

// initilize the mpu6050
void mpu6050_startup()
{
    pinMode(INTERRUPT_PIN, INPUT); // set interrupt pin, to know wether the mpu6050 is ready

    // initialize device
    Serial.println(F("reseting MPU6050..."));
    mpu.reset();
    delay(50);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // TODO add some sort of wait for the mpu6050 connection success

#ifdef WAIT_FOR_INPUT_CHAR_TO_START
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read())
        ; // empty buffer
    while (!Serial.available())
        ; // wait for data
    while (Serial.available() && Serial.read())
        ; // empty buffer again
#endif

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(55);
    mpu.setYGyroOffset(55);
    mpu.setZGyroOffset(-42);
    mpu.setXAccelOffset(-554); // 1688 factory default for my test chip
    mpu.setYAccelOffset(85);   // 1688 factory default for my test chip
    mpu.setZAccelOffset(3072); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // read DLPF mode rate of sample (Digital Low-Pass Filter)
        Serial.print(F("DLPF mode READ -> "));
        Serial.println(mpu.getDLPFMode());
        Serial.print(F("DLPF mode SET to -> "));
        uint8_t DLPFMode = 1;
        mpu.setDLPFMode(DLPFMode);
        Serial.println(mpu.getDLPFMode());
        // read gyro size rate of sample
        uint8_t gyroDivisionRate;
        gyroDivisionRate = mpu.getRate();
        Serial.print(F("gyro sample rate READ -> "));
        Serial.println(gyroDivisionRate);
        gyroDivisionRate = 7;
        mpu.setRate(gyroDivisionRate);
        Serial.print(F("new gyro sample rate SET -> "));
        Serial.println(mpu.getRate());

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    // configure LED for output
}

// a function that takes the mpu degree and
// change it to the limitations of the servo motors
// calbrate - put in limits of (+-90) - change from -90:90 to 0:180
void MPUDeg_2_ServoDeg(int Xc, int Xnc)
{
    if (Xc >= 0)
    { //checks which calibrations need to be performed
        if (Currentdeg >= -180 && Currentdeg < Xnc)
            situation = 1;
        else if (Currentdeg >= Xnc && Currentdeg < 0)
            situation = 2;
        else if (Currentdeg >= 0 && Currentdeg < Xc)
            situation = 3;
        else if (Currentdeg >= Xc && Currentdeg < 180)
            situation = 4;
    }
    else if (Xc < 0)
    {
        if (Currentdeg >= Xnc && Currentdeg < 180)
            situation = 1;
        else if (Currentdeg >= 0 && Currentdeg < Xnc)
            situation = 2;
        else if (Currentdeg >= Xc && Currentdeg < 0)
            situation = 3;
        else if (Currentdeg >= -180 && Currentdeg < Xc)
            situation = 4;
    }
    switch (situation)
    { // puts the calibration in the right situation
    case 1:
        //if positive calibration degree - current degree is between -180 and the opposite circular calibration degree (-180:Deg:Xnc)
        //if negative calibration degree - current degree is between the opposite circular calibration degree and 180 (Xnc:Deg:180)
        Currentdeg = abs(Xnc) + (180 - abs(Currentdeg));
        break;
    case 2:
        //if positive calibration degree - current degree is between the opposite circular calibration degree and 0 (Xnc:Deg:0)
        //if negative calibration degree - current degree is between 0 and the opposite circular calibration degree (0:Deg:Xnc)
        Currentdeg = abs(Xc) + abs(Currentdeg);
        break;
    case 3:
        //if positive calibration degree - current degree is between 0 and the calibration degree (0:Deg:Xc)
        //if negative calibration degree - current degree is between the calibration degree and 0 (Xc:Deg:0)
        Currentdeg = abs(Xc) - abs(Currentdeg);
        break;
    case 4:
        //if positive calibration degree - current degree is between the calibration degree and 180 (Xc:Deg:180)
        //if negative calibration degree - current degree is between -180 and the calibration degree (-180:Deg:Xc)
        Currentdeg = abs(Currentdeg) - abs(Xc);
        break;
    }

    if (Xc >= 0) // checks if it needs to go right or left (by the sutiation
        if (situation == 2 || situation == 3)
            Currentdeg = -Currentdeg;
        else if (Xc < 0)
            if (situation == 1 || situation == 4)
                Currentdeg = -Currentdeg;

    if (Currentdeg > 90) //puts the limits to the servo
        Currentdeg = 90;
    if (Currentdeg < -90)
        Currentdeg = -90;

    Currentdeg = map(Currentdeg, -90, 90, 180, 0); // map the servo degrees to its readable variables
}

void mpu6050_get_data()
{
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount >= 1024)
    {
        // reset so we can continue cleanly
        Serial.printf("fifocount=%d\n", fifoCount);
        mpu.resetFIFO();
        delayMicroseconds(200);
        fifoCount = mpu.getFIFOCount();
        Serial.printf("after reset, fifocount=%d\n", fifoCount);
        Serial.println(F("FIFO overflow!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x03)
    { //FIXME check if this is really necessary
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

#ifdef DEBUG
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);
        Serial.print("Xc_yaw = ");
        Serial.println(Xc_yaw);
#endif
#endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void nrf24_startup()
{
    radio.begin();                   // Initialize the nRF24L01 Radio
    radio.setChannel(108);           // Above most WiFi frequencies
    radio.setDataRate(RF24_250KBPS); // Fast enough.. Better range

    // Set the Power Amplifier Level low to prevent power supply related issues since this is a
    // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
    // PALevelcan be one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
    radio.setPALevel(RF24_PA_MAX);

    radio.enableAckPayload();
    radio.enableDynamicAck();
    radio.setAutoAck(true);
    radio.setRetries(15, 15);

    // Open a writing and reading pipe on each radio, with opposite addresses
    radio.openWritingPipe(addresses[0]);
    //////////////RF24 initialization End/////////////////
}

void send_data_rf24()
{
    if (hasHardware)
    { // Set in variables at top
        /*********************( Read the Joystick positions )*************************/
        Currentdeg = ypr[0] * 180 / M_PI;
        MPUDeg_2_ServoDeg(Xc_yaw, Xnc_yaw);
        myData.yaw = Currentdeg;

        Currentdeg = ypr[1] * 180 / M_PI;
        MPUDeg_2_ServoDeg(Xc_pitch, Xnc_pitch);
        myData.pitch = Currentdeg;

        //		myData.switchOn  = !digitalRead(RESET_YAW);  // Invert the pulldown switch
    }
    else
    {
        myData.yaw = 89; // Send some known fake data
        myData.pitch = 91;
    }

    myData._micros = micros(); // Send back for timing

    Serial.print(F("Now sending  -  "));

    if (radio.write(&myData, sizeof(myData)))
    { // Send data, checking for error ("!" means NOT)
        Serial.println(F("Transmit success "));
        if (radio.isAckPayloadAvailable())
        {
            radio.read(&myAck, sizeof(myAck));
            Serial.println(F("Ack receive success "));
        }
    }
    else
    {
        Serial.println(F("Transmit failed "));
    }

    //timeNow = micros();

    // Show the data that was transmitted and acknowledgment payload
    //	Serial.print(F("Sent "));
    //	Serial.print(timeNow);
    //	Serial.print(F(", Got response "));
    //	Serial.print(myData._micros);
    //	Serial.print(F(", Round-trip delay "));
    //	Serial.print(timeNow - myData._micros);
    //	Serial.println(F(" mS "));
    //	Serial.print(F("Ack check - "));
    //	Serial.print(myAck.check);
    //	Serial.println(F(" done "));
    //	Serial.println();

    // Send again after delay. When working OK, change to something like 100
    delayMicroseconds(500);
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial)
        ; // wait for Leonardo enumeration, others continue immediately
    // NOTE: The "F" in the print statements means "unchangable data; save in Flash Memory to conserve SRAM"
    Serial.println(F("setup begin."));
    Serial.println(F("Send mpu6050 data by nRF24L01 radio to another Arduino"));

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    pinMode(calibration_button_pin, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);

    mpu6050_startup(); //does alot of configurations for the mpu6050 to be set correctly

    nrf24_startup(); //does alot of configurations for the NRF24 to be initialized correctly

    mpu6050_get_data();

    Currentdeg = ypr[0] * 180 / M_PI;
    MPUDeg_2_ServoDeg(Xc_yaw, Xnc_yaw);
    ypr[0] = Currentdeg;

    Xc_pitch = Currentdeg;
    if (Xc_yaw >= 0)
        Xnc_yaw = Xc_yaw - 180;
    else if (Xc_yaw < 0)
        Xnc_yaw = Xc_yaw + 180;

    // test conections
    while (radio.isChipConnected() == false)
    {
        Serial.println("nRF24 is NOT connected");
        delay(2000);
    }
    while (mpu.testConnection() == false)
    {
        Serial.println("mpu6050 is NOT connected");
        delay(2000);
        mpu.reset();
        delay(50);
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
    // if programming failed, don't try to do anything TO_DO consider taking this line out
    if (!dmpReady)
    {
#if defined(DEBUG)
        Serial.printf("FAIL -> DMP is not ready");
        delay(3000);
#endif
        return;
    }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
// other program behavior stuff here
#ifdef DEBUG
        Serial.printf("main loop -> fifocount=%d\n", fifoCount);
        Serial.printf("main loop -> mpu interrupt=%d\n", mpuInterrupt);
#endif
        Serial.println(".");
        fifoCount = mpu.getFIFOCount();
    }

    mpu6050_get_data();

    send_data_rf24();
    delayMicroseconds(500);

    //    while (Serial.available() && Serial.read()); // empty buffer
    if (digitalRead(calibration_button_pin) == 0) // check if button was pressed to calibrate
    {
        //taking messure of current degree on the yaw axis
        Xc_yaw = ypr[0] * 180 / M_PI;
        if (Xc_yaw >= 0)
            Xnc_yaw = Xc_yaw - 180;
        else if (Xc_yaw < 0)
            Xnc_yaw = Xc_yaw + 180;

        //taking messure of current degree on the pitch axis
        Xc_pitch = ypr[1] * 180 / M_PI;
        if (Xc_pitch >= 0)
            Xnc_pitch = Xc_pitch - 180;
        else if (Xc_pitch < 0)
            Xnc_pitch = Xc_pitch + 180;
    }
// while (Serial.available() && Serial.read()); // empty buffer again
#ifdef DEBUG
    counter = micros() - counter;
    counter_float = (float)counter / 1000000; //in seconds
    Serial.printf("operating in %d Hz", (uint32_t)(1 / counter_float));
    // Serial.printf("operating in %d",(counter));
    Serial.println();
    counter = micros();
#endif
}
