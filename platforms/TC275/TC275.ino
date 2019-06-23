#ifdef ARDUINO_ARCH_AVR
  #define StartOfUninitialised_LMURam_Variables
  #define EndOfUninitialised_LMURam_Variables
  #define StartOfInitialised_LMURam_Variables
  #define EndOfInitialised_LMURam_Variables
  #define StartOfUninitialised_CPU1_Variables
  #define EndOfUninitialised_CPU1_Variables
  #define StartOfInitialised_CPU1_Variables
  #define EndOfInitialised_CPU1_Variables
#endif

#include "CNC.h"
#include "drive.h"
#include "steering.h"
#include "debug.h"
#include "updateTFT.h"
#include "speech.h"
#include "encoder.h"

/* LMU uninitialised data */
StartOfUninitialised_LMURam_Variables
/* Put your LMU RAM fast access variables that have no initial values here e.g. uint32 LMU_var; */
int currentSensorValueFive;
int currentSensorValueSix;
float maxCurrentValueFive;
float maxCurrentValueSix;

EndOfUninitialised_LMURam_Variables

/* LMU uninitialised data */
StartOfInitialised_LMURam_Variables

#include "TC275.h"

int stateSeven = LOW;
unsigned long previousMillisSeven = 0;

int intervalOperateCNC = 10000; // CNC timer.
int intervalMoveSlightlyForwards = 2000; // Move slightly forwards after CNC action timer.

int asyncCount =0;

long rightWheel=0;
long leftWheel=0;

int potentiometerState=LOW;    // Use barcodes or not. Drive potentiometer will be overridden when high.
//int pixyBarData=0;

int currentSensorValueRAxis =0;

long runningCurrentValueRAxis =0;
float RAxisCurrentSensorWeighting =0;

long moveStepsForwards =0;
long totalMoveStepsForwards =79601;             // 558.8 mm (22").
long jetsonReading =0;                           // analogRead(A7).
int makeTurnValue =0;
int finalSteeringValue =0;
int finalDriveValue =0;

EndOfInitialised_LMURam_Variables


/*** Core 0 ***/
void setup()
{  
  delay(5000);
  pinMode(2,OUTPUT); //STEP Steer Motor
  pinMode(3,OUTPUT); //DIRECTION HIGH is clockwise
  pinMode(4,OUTPUT); //STEP Steer Motor
  pinMode(5,OUTPUT); //DIRECTION HIGH is clockwise
  pinMode(6,OUTPUT); //STEP Drive Motor
  pinMode(7,OUTPUT); //DIRECTION HIGH is clockwise
  pinMode(10,OUTPUT); //Seems to be broken on my machine! Or maybe not compatible with motor step drive? Moved to pin 13 instead.
  pinMode(11,OUTPUT); //DIRECTION HIGH is clockwise
  pinMode(13,OUTPUT); //STEP Drive Motor   
  
  pinMode(35, OUTPUT);   // Step x axis RHS
  digitalWrite(35,LOW);
  pinMode(12, OUTPUT);   // Step x axis LHS
  digitalWrite(12,LOW);
  pinMode(53, OUTPUT);   // Direction x axis, LOW is forwards
  digitalWrite(53,LOW);
  pinMode(33, OUTPUT);   // Step y axis
  digitalWrite(33,LOW);
  pinMode(51, OUTPUT);   // Direction y axis, LOW is forwards
  digitalWrite(51,LOW);
  pinMode(31, OUTPUT);   // Step z axis
  digitalWrite(31,LOW);
  pinMode(27, OUTPUT);   // Direction z axis, LOW is forwards
  digitalWrite(27,LOW);
  pinMode(29, OUTPUT);   // Step r axis
  digitalWrite(29,LOW);
   
  pinMode(37, OUTPUT);   // LED
  digitalWrite(37,LOW);
  pinMode(39,OUTPUT);    // LED
  digitalWrite(39,LOW);
  
  pinMode(25,INPUT_PULLUP);    // Turn on/off drive motors
  pinMode(23,INPUT_PULLUP);    // Control state switch eg manual / autonomous

  pinMode(49,INPUT_PULLUP);    // Limit switch RHS forwards X
  pinMode(47,INPUT_PULLUP);    // Limit switch LHS forwards X 
  pinMode(45,INPUT_PULLUP);    // Limit switch down Z
  pinMode(43,INPUT_PULLUP);    // Limit switch left Y
  pinMode(41,INPUT_PULLUP);    // Limit switch upwards Z (no limit switch on down)
  
  delay(5000);
  
  #ifdef WEEDINATOR_SINGLE_CORE
    setup1();
    setup2();
  #endif
}

void loop()
{  
  unsigned long currentMicrosOne = micros();
  unsigned long currentMicrosTwo = micros(); 
  unsigned long currentMicrosThree = micros();
  unsigned long currentMicrosFour = micros();
  
  navState = digitalRead(25);                      // switch for Pixy / GPS.
  controlState = digitalRead(23);                  // switch for autonomous mode.
  if(controlState==LOW)                            // manual mode.
  {
    preventSteering=false;
  }
  else
  {
    preventSteering=true;
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  //weedingBegins();                                        // Main link to CNC stuff.
/////////////////////////////////////////////////////////////
  turningWhenStationary();                                   // Accounts for turning when stationary.
/////////////////////////////////////////////////////////////  
  speedTimerAsyncLeftA = 665;                                // Motors default is neutral.
  if ((difference<-300)||(difference>300))
  {
    actuallySteering = HIGH;
  }
  else
  {
    actuallySteering = LOW;  
  }
  if ((finalDriveValue<715)&&(finalDriveValue>615))  //Stationary
  {
    stationary=HIGH;
  }
  else
  {
    stationary=LOW;
    ampflowMotor();
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////
  difference = finalSteeringValue - previousFinalSteeringValue; // This gives actual movement of the steering.
//////////////////////////////////////////////////////////////////////////////////////////////////////  
  differential();
  
  if(wheelsPosition>0)
  {
   rightHandLock=HIGH;
   leftHandLock=LOW; 
  }
  if(wheelsPosition<0)
  {
   rightHandLock=LOW;
   leftHandLock=HIGH; 
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////
  difference = finalSteeringValue - previousFinalSteeringValue;
//////////////////////////////////////////////////////////////////////////////
  #ifdef WEEDINATOR_SINGLE_CORE
    loop1();
    loop2();
  #endif

} // loop
//////////////////////////////////////////////////////////////////////////////////////////////////////
/*** Core 1 ***/
#ifndef ARDUINO_ARCH_AVR
  #include "tone.h"
#endif

/* CPU1 Uninitialised Data */
StartOfUninitialised_CPU1_Variables
/* Put your CPU1 fast access variables that have no initial values here e.g. uint32 CPU1_var; */

EndOfUninitialised_CPU1_Variables

/* CPU1 Initialised Data */
StartOfInitialised_CPU1_Variables
/* Put your CPU1 fast access variables that have an initial value here e.g. uint32 CPU1_var_init = 1; */
long steeringValue =0;
int k=0;
int m=0;
int makeTurn = LOW;

int ic=0;
int icMax =0;
unsigned long currentMillis = 0;
unsigned long previousMillisFive = 0;
unsigned long previousMillisSix = 0;
int intervalFive = 0;
const int intervalSix = 1000;
int millisCalcFive =0;
int millisCalcSix =0;
int ACFrequency = 50; // Hertz



EndOfInitialised_CPU1_Variables
  
void setup1() 
{
  intervalFive = 1000 / ACFrequency; // Milliseconds.
  IncrEnc_Init();  // Encoder
  encoderCount = 10000;
}
void loop1() 
{
//////////////////////////////////////////////////////////////////////////////
  encoderReadings();
////////////////////////////////////////////////////////////////////////////// 
  ic = 0;
  runningCurrentValueRAxis = 0;
  while (ic < intervalFive)                            // AC 50 hertz is equivalent to 20 ms per AC cycle
  {
    ic++;
    delayMicroseconds(1000);                           // Capture data over one complete AC cycle.
    currentSensorValueFive = analogRead(A5);
    currentSensorValueSix = analogRead(A6);
    currentSensorValueRAxis = analogRead(A4);          // Dc current on R axis power supply

    runningCurrentValueRAxis = currentSensorValueRAxis + runningCurrentValueRAxis;
    if(currentSensorValueFive > maxCurrentValueFive)
    {
      maxCurrentValueFive = currentSensorValueFive;
    }
    if(currentSensorValueSix > maxCurrentValueSix)
    {
      maxCurrentValueSix = currentSensorValueSix;
    }
  }
//////////////////////////////////////////////////////////////////////////////
    finalCurrentSensorValueRAxis = runningCurrentValueRAxis / intervalFive;  // DC current on R axis power supply has been sampled (intervalFive) times.    
    runningmaxCurrentValueFive = (maxCurrentValueFive - 523)/80;
    runningmaxCurrentValueSix = (maxCurrentValueSix - 523)/80;
    maxCurrentValueFive = 0;
    maxCurrentValueSix = 0;
//////////////////////////////////////////////////////////////////////////////
  long driveValue=0;
  k=0;
  //if(controlState==LOW)                // Autonomous mode disabled !!!!!!!!!!!!
  //{
    while(k<100)
    { 
      jetsonReading = jetsonReading + analogRead(A7);
      driveValue = driveValue + analogRead(A1);
      k++;
    }
      finalDriveValue = driveValue/k;
      jetsonReading = jetsonReading/k;
  //}
////////////////////////////////////////////////////////////////////////////////////////
  //ArduinoPwmFreq(4,390); // set PWM freq on pin 4 to 1 kHz
  //analogWrite(4,finalDriveValue/4);
///////////////////////////////////////////////////////////////////////////////////////  
  //controlState = digitalRead(23); 
  if(navState==HIGH)
  { 
    int jetsonReadingX = jetsonReading;
    //map(jetsonReadingX,0,1024,-100,100);      // map 0 -> -100, 1024 -> 100.
    makeTurnValue = jetsonReadingX;
  }
  else
  {
    makeTurnValue = bearingDegrees - headingDegrees;  // GPS
  }

  if((makeTurnValue)>0)
  {
    makeTurn = HIGH;
  }
  else
  {
    makeTurn = LOW;
  }
  if (preventSteering==false)
  {
    if(controlState==HIGH)
    {
      steeringValue = makeTurnValue*5 + 10000;
      finalSteeringValue = steeringValue;
    }
    else
    {
      steeringValue=0;
      m=0;
      while(m<200)
      { 
        steeringValue = steeringValue + analogRead(A0);
        m++;
      }
      finalSteeringValue = 30*steeringValue/m;
    }
  }   // if (preventSteering==false)
} // loop end

/*** Core 2 ***/
/* CPU2 Uninitialised Data */
#include <Wire.h>
#include <SPI.h>

#include "speech.h"
#include "navdata.h"
#include "units.h"

/* LMU uninitialised data */
StartOfUninitialised_LMURam_Variables

String textDistanceData;
String textBearingData;

EndOfUninitialised_LMURam_Variables

/* LMU uninitialised data */
StartOfInitialised_LMURam_Variables

int STOP = LOW;
int emicBearing=0;
unsigned long previousMillis1Core3 = 0;
unsigned long previousMillis2Core3 = 0;
unsigned long previousMillis3Core3 = 0;
unsigned long previousMillis4Core3 = 0;
const long interval1Core3 = 10000;
const long interval2Core3 = 30000;
const long interval3Core3 = 20000;
unsigned long millisCalc1 = 0;
unsigned long millisCalc2 = 0;
unsigned long millisCalc3 = 0;
unsigned long millisCalc4 = 0;

String text5 = " .Make a clockwise turn. ";
int ledBlueState = LOW;

EndOfInitialised_LMURam_Variables


void setup2() 
{
  emicPort.begin( EMIC_BAUD );

  DEBUG_PORT.begin( DEBUG_BAUD );
  DEBUG_PORT.println("TC275");

  initNavData();
  setupTFT();

  delay(2000);
  while (emicPort.available()) 
  {
    emicIntro();
  }
}

void loop2() 
{
  //tone(2,jetsonReading + 500);   // pin,pitch,duration
  //if (ledBlueState == LOW) 
  //{
  //    ledBlueState = HIGH;
  //} else 
  //{
  //    ledBlueState = LOW;
  //}
  //digitalWrite(37, ledBlueState);


  //delay(200);
  unsigned long currentMillisCore3 = millis();
  millisCalc1 = currentMillisCore3 - previousMillis1Core3; // Emic speech
  millisCalc2 = currentMillisCore3 - previousMillis2Core3; // TFT screen
  millisCalc3 = currentMillisCore3 - previousMillis3Core3; // Debug print
  millisCalc4 = currentMillisCore3 - previousMillis4Core3; // I2C
/////////////////////////////////////////////////////////////////////////////////////////////////////////  
  if (millisCalc2 >= 5000)                            // timer .....  5,000
  {  
     updateTFT();                                  // TFT screen cuases code to pause.
     previousMillis2Core3=currentMillisCore3;         
  }
/////////////////////////////////////////////////////////////////////////////////////////////////////////  
  if (millisCalc1 >= emicTimerInterval)                           // timer  .... 30,000
  {  
    //DEBUG_PORT.println("Emic Speech SHOULD be activated ");
    while (emicPort.available()) 
    {
      //DEBUG_PORT.println("Emic serial IS available");
      emicSpeech1();
    }
    previousMillis1Core3=currentMillisCore3;                          // this is the only previousMillis reset!
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////// 
  if (millisCalc3 >= 250)                            // timer 1000 = 1 second.
  {  
    previousMillis3Core3=currentMillisCore3; 
    debug();
  } // millisCalc3
/////////////////////////////////////////////////////////////////////////////////////////

  uint8_t  message[ navData_t::MSG_SIZE ];
  uint8_t *ptr   = &message[0];
  uint8_t  count = 0;

  if (useI2C) {
    Wire.requestFrom( MEGA_I2C_ADDR, navData_t::MSG_SIZE );

    while (Wire.available()) 
    {
      uint8_t c = Wire.read();
      if (count < sizeof(message)) {
        count++;
        *ptr++ = c;
      }
    } //////// End of I2C read

  } else if (useConsole) {
    if (DEBUG_PORT.available() and (DEBUG_PORT.read() == 'r')) {
      uint8_t example[] = 
        { 0xB0, 0x30, 0xBE, 0x1F, // lat 532558000
          0xF0, 0x21, 0x6E, 0xFD, // lon -43114000
          0x7B, 0x00,             // waypoint 123
          0x80, 0x96, 0x98, 0x00, // dist 10000000mm
          0x39, 0x30,             // bearing 123.45
          0x85, 0x1A,             // heading 67.89
          0xEB, 0xFC, 0xFF, 0xFF, // panError -789
        };
      count = sizeof(example);
      memcpy( message, example, count );
    } else {
      DEBUG_PORT.print( '?' );
    }
  }

  if (count > 0) {
    navData.readFrom( message, count );

    latitudeUblox  = navData.location().lat();
    longitudeUblox = navData.location().lon();

    distanceMM     = navData.distance() * MM_PER_M;
    headingDegrees = navData.heading();
    bearingDegrees = navData.bearing();

    emicBearing    = bearingDegrees;
    
    pixyPanData = navData.panError();
    pixyBarData = navData.barcodeValue();
    pixyBarcode = navData.barcode();
  }
} // loop2
///////////////////////////////////////////////////////////////////////////////////////////
