

/* LMU uninitialised data */
StartOfUninitialised_LMURam_Variables
/* Put your LMU RAM fast access variables that have no initial values here e.g. uint32 LMU_var; */
int currentSensorValueFive;
int currentSensorValueSix;
float maxCurrentValueFive;
float maxCurrentValueSix;
float resultTwo;
float resultThree;
float resultFour;
EndOfUninitialised_LMURam_Variables

/* LMU uninitialised data */
StartOfInitialised_LMURam_Variables
#include "TC275.h"
float runningmaxCurrentValueFive = 1;
float runningmaxCurrentValueSix = 1;
float resultOne = 1.0000;
const int ledPin =  13;
int ledStateOne = LOW;   
int ledStateTwo = LOW;   
int ledStateThree = LOW;
int ledStateFour = LOW;

unsigned long previousMicrosOne = 0;
unsigned long previousMicrosTwo = 0;
unsigned long previousMicrosThree = 0;
unsigned long previousMicrosFour = 0;

float intervalOne = 1000; // Right hand wheel turn speed. Bigger is faster.
float intervalTwo = 1000; // Left hand wheel turn speed.
float intervalThree = 1000; // Right hand wheel drive speed. Is this left wheel?
float intervalFour = 1000; // Left hand wheel drive speed.

int difference=0;
int previousFinalSteeringValue=15300;

long rightWheel=0;
long leftWheel=0;
long wheelsPosition=0;
long ferrets=0;

int finalDriveValue=0;
long finalSteeringValue =0;
int forwards = LOW;
int backwards =LOW;
int rightHandLock =LOW;
int leftHandLock=LOW;
int clockW=LOW;
int antiClockW=LOW;
int stationary=LOW;
int actuallySteering =LOW;
EndOfInitialised_LMURam_Variables


/*** Core 0 ***/
void setup()
{  
  delay(5000);
  pinMode(ledPin, OUTPUT);
  pinMode(5,OUTPUT); //STEP Steer Motor
  pinMode(6,OUTPUT); //DIRECTION HIGH is clockwise
  pinMode(7,OUTPUT); //STEP Steer Motor
  pinMode(8,OUTPUT); //DIRECTION HIGH is clockwise
  pinMode(9,OUTPUT); //STEP Drive Motor
  pinMode(10,OUTPUT); //DIRECTION HIGH is clockwise
  pinMode(11,OUTPUT); //STEP Drive Motor
  pinMode(12,OUTPUT); //DIRECTION HIGH is clockwise

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

speedDifferential();
if ((difference<-300)||(difference>300))
{
  actuallySteering = HIGH;
}
else
{
  actuallySteering = LOW;  
}
if ((finalDriveValue<550)&&(finalDriveValue>450))  //Stationary
{
  stationary=HIGH;
}
else
{
  stationary=LOW;
}
if (finalDriveValue>=550)  //Backwards.
{
  backwards = HIGH; forwards = LOW;
  intervalThree = (400000/finalDriveValue)-100; // 140 is max speed.
  intervalFour =  (400000/finalDriveValue)-100; // 140 is max speed.
  speedDifferential();
  torqueDifferential();
  digitalWrite(10,HIGH);
  digitalWrite(12,HIGH);
  if ((currentMicrosThree - previousMicrosThree >= intervalThree)&&(intervalThree>100))
  {
    changeStateThree();
    digitalWrite(9,ledStateThree); // Drive motor step
  }
  if ((currentMicrosFour - previousMicrosFour >= intervalFour)&&(intervalFour>100))
  {
    changeStateFour();
    digitalWrite(11,ledStateFour); // Drive motor step
  }
}
if (finalDriveValue<450)//Forwards.
{
  backwards = LOW; forwards = HIGH;
  intervalThree = (finalDriveValue*1)+100; // 140 is max speed. Right hand wheel.
  intervalFour = (finalDriveValue*1)+100; // 140 is max speed.
  speedDifferential();
  torqueDifferential();
  //There are two main states that the steering is in 1) static at any angle and 2) moving to new angle. The main calculation is
  // speedDifferential(); but there is also actualTurnSpeedDifferential(); which is nested within speedDifferential(); and 
  // overides it if the steering is in the process of changing angle. The latter is important as the steering bearing is offset
  // away from the ideal position right above the wheel and means that the speed of the wheels needs to take account of the speed
  // of turn or else the wheels will effectively grind against each other.
  digitalWrite(10,LOW); // Direction
  digitalWrite(12,LOW); // Direction
  if (currentMicrosThree - previousMicrosThree >= intervalThree)
  {
    changeStateThree();
    digitalWrite(9,ledStateThree); // Drive motor step
  }
  if (currentMicrosFour - previousMicrosFour >= intervalFour)
  {
    changeStateFour();
    digitalWrite(11,ledStateFour); // Drive motor step
  }
}
  
//////////////////////////////////////////////////////////////////////////////////////////////////////
  difference = finalSteeringValue - previousFinalSteeringValue; // This gives actual movement of the steering.
//////////////////////////////////////////////////////////////////////////////////////////////////////  
  if((difference>300)&&(wheelsPosition>=0)) // Clockwise from the centre on RH lock. 300 is deadzone.
  {
    intervalOne = 2500; // Speed of RH inner wheel turn (fast) higher the value, slower the speed. Min 1250.
    intervalTwo = 2000; // Speed of LH outer wheel turn (slow) higher the value, slower the speed. Min 1250.
    clockWise();
    if (currentMicrosTwo - previousMicrosTwo >= intervalTwo)
    {
      changeStateTwo();
      digitalWrite(7,ledStateTwo); //STEP
      wheelsPosition++;
      previousFinalSteeringValue++; 
    }
    if (currentMicrosOne - previousMicrosOne >= intervalOne)
    {
      changeStateOne();
      digitalWrite(5,ledStateOne); //STEP
    }
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////  
  if((difference>300)&&(wheelsPosition<0)) // Clockwise from the full LH lock position 
  {
    intervalOne = 2500; // Speed of RH outer wheel turn (slow) higher the value, slower the speed. Min 1250.
    intervalTwo = 2000; // Speed of LH inner wheel turn (fast) higher the value, slower the speed. Min 1250.
    clockWise();
    if (currentMicrosOne - previousMicrosOne >= intervalOne)
    {
      changeStateOne();
      digitalWrite(7,ledStateOne); //STEP
    }
    if (currentMicrosTwo - previousMicrosTwo >= intervalTwo)
    {
      changeStateTwo();
      digitalWrite(5,ledStateTwo); //STEP
      wheelsPosition++;
      previousFinalSteeringValue++;
    }
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////
  if((difference<-300)&&(wheelsPosition>=0)) // Anti-clockwise from the full RH lock position 
  {
    intervalOne = 2500; // Speed of RH inner wheel turn (fast) higher the value, slower the speed. Min 1250.
    intervalTwo = 2000; // Speed of LH outer wheel turn (slow) higher the value, slower the speed. Min 1250.
    antiClockWise();
    if (currentMicrosTwo - previousMicrosTwo >= intervalTwo)
    {
      changeStateTwo();
      digitalWrite(7,ledStateTwo); //STEP
      wheelsPosition--;
      previousFinalSteeringValue--;
    }
    if (currentMicrosOne - previousMicrosOne >= intervalOne)
    {
      changeStateOne();
      digitalWrite(5,ledStateOne); //STEP
    }
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////
  if((difference<-300)&&(wheelsPosition<0)) // Anti-clockwise from the centre 
  {
    intervalOne = 2500; // Speed of RH outer wheel turn (slow) higher the value, slower the speed. Min 1250.
    intervalTwo = 2000; // Speed of LH inner wheel turn (fast) higher the value, slower the speed. Min 1250.
    antiClockWise();
    if (currentMicrosOne - previousMicrosOne >= intervalOne)
    {
      changeStateOne();
      digitalWrite(7,ledStateOne); //STEP
    }
    if (currentMicrosTwo - previousMicrosTwo >= intervalTwo)
    {
      changeStateTwo();
      digitalWrite(5,ledStateTwo); //STEP
      wheelsPosition--;
      previousFinalSteeringValue--;
    }
  }
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

void torqueDifferential()
{
  // Make RH wheel turn slightly faster if current in LH wheel too high. Three and five is LH and Four and six is RH.
  // Weighting is 6:4.
  intervalFour = ((0.60*intervalFour) + (0.40*(intervalFour * (runningmaxCurrentValueSix / runningmaxCurrentValueFive))));
}
void speedDifferential()
{
  ferrets = map(wheelsPosition,-1,-15000,1,15000);  //Changes all negative values of wheelsPosition to positive.
  if(wheelsPosition>200)
  {
     intervalFour =  intervalFour+(wheelsPosition*intervalFour)/20000; // Makes right hand inside wheel drive slower.
  }
  if(wheelsPosition<-200)
  {
     intervalThree =  intervalThree+(ferrets*intervalThree)/20000; // Makes left hand inside wheel drive slower.
  }
  actualTurnSpeedDifferential();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void actualTurnSpeedDifferential() 
// When the wheels are 'actually' steering, inside wheel can grind badly as it turns against drive. 'Actually' means
// that the steering motors are actually moving but in reality steering will also happen when motors are stationary.
// The steering has four main states 1. Clockwise & RH lock, 2. Anti-clockwise & RH lock, 3. Clockwise & LH lock and 4. Anti-clockwise & RH lock.
// Additionally, there is 'backwards' and 'forwards', giving possible 8 different states.
{
  int ATSD = 150;  //This is adjustment for steering geometry.
  float radiusRatio = 2.4;
  unsigned long currentMicrosThree = micros();
  unsigned long currentMicrosFour = micros();
  // Interval one is RH steering turn speed. Wheel radius = 280. Steering radius = 200. Radius ratio = 280/200 = 1.4.
  
  if((clockW==HIGH)&&(rightHandLock==HIGH)&&(stationary==HIGH)&&(actuallySteering==HIGH)) // Actually steering when stationary
  {
     digitalWrite(10,HIGH); // Backwards
     digitalWrite(12,LOW);  // Forwards
     intervalThree = intervalOne*radiusRatio;  // RH wheel speed
     intervalFour = intervalTwo*radiusRatio;   // LH wheel speed
     if ((currentMicrosThree - previousMicrosThree >= intervalThree))
     {
     changeStateThree();
     digitalWrite(9,ledStateThree); // LH Drive motor step
     }
     if ((currentMicrosFour - previousMicrosFour >= intervalFour))
     {
     changeStateFour();
     digitalWrite(11,ledStateFour); // RH Drive motor step
     }
  }
  if((antiClockW==HIGH)&&(rightHandLock==HIGH)&&(stationary==HIGH)&&(actuallySteering==HIGH)) // Actually steering when stationary
  {
     digitalWrite(10,LOW); // Forwards
     digitalWrite(12,HIGH);  // Backwards
     intervalThree = intervalOne*radiusRatio;  // RH wheel speed
     intervalFour = intervalTwo*radiusRatio;   // LH wheel speed
     if ((currentMicrosThree - previousMicrosThree >= intervalThree))
     {
     changeStateThree();
     digitalWrite(9,ledStateThree); // LH Drive motor step
     }
     if ((currentMicrosFour - previousMicrosFour >= intervalFour))
     {
     changeStateFour();
     digitalWrite(11,ledStateFour); // RH Drive motor step
     }
  }
  if((clockW==HIGH)&&(leftHandLock==HIGH)&&(stationary==HIGH)&&(actuallySteering==HIGH)) // Actually steering when stationary
  {
     digitalWrite(10,HIGH); // Backwards
     digitalWrite(12,LOW);  // Forwards
     intervalThree = intervalOne*radiusRatio;  // RH wheel speed
     intervalFour = intervalTwo*radiusRatio;   // LH wheel speed
     if ((currentMicrosThree - previousMicrosThree >= intervalThree))
     {
     changeStateThree();
     digitalWrite(9,ledStateThree); // LH Drive motor step
     }
     if ((currentMicrosFour - previousMicrosFour >= intervalFour))
     {
     changeStateFour();
     digitalWrite(11,ledStateFour); // RH Drive motor step
     }
  }
  if((antiClockW==HIGH)&&(leftHandLock==HIGH)&&(stationary==HIGH)&&(actuallySteering==HIGH)) // Actually steering when stationary
  {
     digitalWrite(10,LOW); // Forwards
     digitalWrite(12,HIGH);  // Backwards
     intervalThree = intervalOne*radiusRatio;  // RH wheel speed
     intervalFour = intervalTwo*radiusRatio;   // LH wheel speed
     if ((currentMicrosThree - previousMicrosThree >= intervalThree))
     {
     changeStateThree();
     digitalWrite(9,ledStateThree); // LH Drive motor step
     }
     if ((currentMicrosFour - previousMicrosFour >= intervalFour))
     {
     changeStateFour();
     digitalWrite(11,ledStateFour); // RH Drive motor step
     }
  }
  radiusRatio = 3;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FORWARDS:
  if((clockW==HIGH)&&(rightHandLock==HIGH)&&(forwards==HIGH)&&(actuallySteering==HIGH))
  {
     //intervalThree =  (1/(1/intervalThree) - (1/(intervalOne*radiusRatio))); // Makes right hand inside wheel drive a bit slower.
     //intervalFour =   (1/(1/intervalFour)  + (1/(intervalTwo*radiusRatio))); // Makes left hand outside wheel drive a bit faster.
     resultOne =  1/intervalThree;
     resultTwo = intervalOne*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne + resultThree;
     intervalThree = 1/resultFour;

     resultOne =  1/intervalFour;
     resultTwo = intervalTwo*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne - resultThree;
     intervalFour = 1/resultFour;
  }
  if((antiClockW==HIGH)&&(rightHandLock==HIGH)&&(forwards==HIGH)&&(actuallySteering==HIGH))
  {
     //intervalThree =  (1/(1/intervalThree) + (1/(intervalOne*radiusRatio))); // Makes right hand inside wheel drive a bit faster.
     //intervalFour =   (1/(1/intervalFour)  - (1/(intervalTwo*radiusRatio))); // Makes left hand outside wheel drive a bit slower.
     resultOne =  1/intervalThree;
     resultTwo = intervalOne*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne - resultThree;
     intervalThree = 1/resultFour;

     resultOne =  1/intervalFour;
     resultTwo = intervalTwo*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne + resultThree;
     intervalFour = 1/resultFour;
  }  
  if((clockW==HIGH)&&(leftHandLock==HIGH)&&(forwards==HIGH)&&(actuallySteering==HIGH))
  {
     //intervalThree =  (1/(1/intervalThree) + (1/(intervalOne*radiusRatio))); // Makes left hand inside wheel drive a bit faster.
     //intervalFour =   (1/(1/intervalFour)  - (1/(intervalTwo*radiusRatio))); // Makes right hand outside wheel drive a bit slower.
     resultOne =  1/intervalThree;
     resultTwo = intervalOne*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne + resultThree;
     intervalThree = 1/resultFour;

     resultOne =  1/intervalFour;
     resultTwo = intervalTwo*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne - resultThree;
     intervalFour = 1/resultFour;
  } 
  if((antiClockW==HIGH)&&(leftHandLock==HIGH)&&(forwards==HIGH)&&(actuallySteering==HIGH))
  {
     //intervalThree =  (1/(1/intervalThree) - (1/(intervalOne*radiusRatio))); // Makes right hand inside wheel drive a bit slower.
     //intervalFour =   (1/(1/intervalFour)  + (1/(intervalTwo*radiusRatio))); // Makes left hand outside wheel drive a bit faster.
     resultOne =  1/intervalThree;
     resultTwo = intervalOne*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne - resultThree;
     intervalThree = 1/resultFour;

     resultOne =  1/intervalFour;
     resultTwo = intervalTwo*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne + resultThree;
     intervalFour = 1/resultFour;
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BACKWARDS:
  if((clockW==HIGH)&&(rightHandLock==HIGH)&&(backwards==HIGH)&&(actuallySteering==HIGH))
  {
     //intervalThree =  (1/(1/intervalThree) - (1/(intervalOne*radiusRatio))); // Makes right hand inside wheel drive a bit slower.
     //intervalFour =   (1/(1/intervalFour)  + (1/(intervalTwo*radiusRatio))); // Makes left hand outside wheel drive a bit faster.
     resultOne =  1/intervalThree;
     resultTwo = intervalOne*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne - resultThree;
     intervalThree = 1/resultFour;

     resultOne =  1/intervalFour;
     resultTwo = intervalTwo*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne + resultThree;
     intervalFour = 1/resultFour;
  }
  if((antiClockW==HIGH)&&(rightHandLock==HIGH)&&(backwards==HIGH)&&(actuallySteering==HIGH))
  {
     //intervalThree =  (1/(1/intervalThree) + (1/(intervalOne*radiusRatio))); // Makes right hand inside wheel drive a bit faster.
     //intervalFour =   (1/(1/intervalFour)  - (1/(intervalTwo*radiusRatio))); // Makes left hand outside wheel drive a bit slower.
     resultOne =  1/intervalThree;
     resultTwo = intervalOne*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne + resultThree;
     intervalThree = 1/resultFour;

     resultOne =  1/intervalFour;
     resultTwo = intervalTwo*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne - resultThree;
     intervalFour = 1/resultFour;
  }  
  if((clockW==HIGH)&&(leftHandLock==HIGH)&&(backwards==HIGH)&&(actuallySteering==HIGH))
  {
     //intervalThree =  (1/(1/intervalThree) + (1/(intervalOne*radiusRatio))); // Makes left hand inside wheel drive a bit faster.
     //intervalFour =   (1/(1/intervalFour)  - (1/(intervalTwo*radiusRatio))); // Makes right hand outside wheel drive a bit slower.
     resultOne =  1/intervalThree;
     resultTwo = intervalOne*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne - resultThree;
     intervalThree = 1/resultFour;

     resultOne =  1/intervalFour;
     resultTwo = intervalTwo*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne + resultThree;
     intervalFour = 1/resultFour;
  } 
  if((antiClockW==HIGH)&&(leftHandLock==HIGH)&&(backwards==HIGH)&&(actuallySteering==HIGH))
  {
     //intervalThree =  (1/(1/intervalThree) - (1/(intervalOne*radiusRatio))); // Makes right hand inside wheel drive a bit slower.
     //intervalFour =   (1/(1/intervalFour)  + (1/(intervalTwo*radiusRatio))); // Makes left hand outside wheel drive a bit faster.
     resultOne =  1/intervalThree;
     resultTwo = intervalOne*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne + resultThree;
     intervalThree = 1/resultFour;

     resultOne =  1/intervalFour;
     resultTwo = intervalTwo*radiusRatio;
     resultThree = 1/resultTwo;
     resultFour = resultOne - resultThree;
     intervalFour = 1/resultFour;
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
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
int makeTurnValue =0;
long distanceMM =0;
int controlState = LOW;
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
  pinMode(23,INPUT);
  intervalFive = 1000 / ACFrequency; // Milliseconds.
}
void loop1() 
{
////////////////////////////////////////////////////////////////////////////// 
  ic = 0;
  while (ic < intervalFive)   // AC 50 hertz is equivalent to 20 ms per AC cycle
  {
    ic++;
    delayMicroseconds(1000);                 // Capture data over one complete AC cycle.
    currentSensorValueFive = analogRead(A3);
    currentSensorValueSix = analogRead(A2);
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
    runningmaxCurrentValueFive = (maxCurrentValueFive - 523)/80;
    runningmaxCurrentValueSix = (maxCurrentValueSix - 523)/80;
    maxCurrentValueFive = 0;
    maxCurrentValueSix = 0;
    //DEBUG_PORT.print("  LHS amps max:  ");DEBUG_PORT.print(runningmaxCurrentValueFive,2);
    //DEBUG_PORT.print("  RHS amps max:  ");DEBUG_PORT.println(runningmaxCurrentValueSix,2);
//////////////////////////////////////////////////////////////////////////////
  long driveValue=0;
  k=0;
  while(k<100)
  { 
    driveValue = driveValue + analogRead(A1);
    k++;
  }
  finalDriveValue = driveValue/k;
////////////////////////////////////////////////////////////////////////////////////////
  //ArduinoPwmFreq(4,390); // set PWM freq on pin 4 to 1 kHz
  //analogWrite(4,finalDriveValue/4);
///////////////////////////////////////////////////////////////////////////////////////  
  controlState = digitalRead(23); // Autonomous mode on slide switch to 5V.
  if(controlState==HIGH)
  {
    if((distanceMM<500)||(distanceMM>100000))
    {
      finalDriveValue = 512; // waiting for new waypoint
    }
    if((distanceMM>500)&&(makeTurnValue<200)&&(makeTurnValue>-200))
    {
      steeringValue = makeTurnValue*10 + 512; // Adjust steering to compass sensitivity
      finalSteeringValue = 30*steeringValue;
    }
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
  // makeTurn HIGH is clockwise
  // makeTurnValue is degrees to turn

} // loop end
void changeStateOne()
{
  unsigned long currentMicrosOne = micros();
  previousMicrosOne = currentMicrosOne;
  if (ledStateOne == LOW) 
    {
    ledStateOne = HIGH;
    } 
  else 
    {
    ledStateOne = LOW;
    }
  digitalWrite(ledPin, ledStateOne);
}
void changeStateTwo()
{
  unsigned long currentMicrosTwo = micros();  
  previousMicrosTwo = currentMicrosTwo;
  if (ledStateTwo == LOW) 
    {
    ledStateTwo = HIGH;
    } 
  else 
    {
    ledStateTwo = LOW;
    }
  digitalWrite(ledPin, ledStateTwo);
}
void changeStateThree()
{
  unsigned long currentMicrosThree = micros();  
  previousMicrosThree = currentMicrosThree;
  if (ledStateThree == LOW) 
    {
    ledStateThree = HIGH;
    } 
  else 
    {
    ledStateThree = LOW;
    }
  digitalWrite(ledPin, ledStateThree);
}
void changeStateFour()
{
  unsigned long currentMicrosFour = micros();
  previousMicrosFour = currentMicrosFour;
  if (ledStateFour == LOW)
    {
    ledStateFour = HIGH;
    } 
  else
    {
    ledStateFour = LOW;
    }
  digitalWrite(ledPin, ledStateFour);
}
void clockWise()
{
  clockW=HIGH;antiClockW=LOW;
  digitalWrite(6,LOW); //DIRECTION LOW is clockwise
  digitalWrite(8,LOW); //DIRECTION LOW is clockwise
  unsigned long currentMicrosOne = micros();
  unsigned long currentMicrosTwo = micros();
}
void antiClockWise()
{
  clockW=LOW;antiClockW=HIGH;
  digitalWrite(6,HIGH); //Anti-clockwise
  digitalWrite(8,HIGH); //Anti-clockwise
  unsigned long currentMicrosOne = micros();
  unsigned long currentMicrosTwo = micros();
}
/*** Core 2 ***/
/* CPU2 Uninitialised Data */
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#define TFT_CS     40
#define TFT_RST    24  // you can also connect this to the Arduino reset
#define TFT_DC     22
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

#include "navdata.h"
#include "units.h"

/* LMU uninitialised data */
StartOfUninitialised_LMURam_Variables
long latitudeUblox;
long longitudeUblox;
float bearingDegrees;

String text4;
String text1;
String text2;
String text3;
String text6;
String text7;
String textDistanceData;
String textBearingData;

EndOfUninitialised_LMURam_Variables

/* LMU uninitialised data */
StartOfInitialised_LMURam_Variables

int STOP = LOW;
String cmd = "S"; 
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
float headingDegrees = 0;
int makeTurn = LOW;
String text5 = " .Make a clockwise turn. ";
int ledBlueState = LOW;
EndOfInitialised_LMURam_Variables


void setup2() 
{
  emicPort.begin( EMIC_BAUD );
  
  DEBUG_PORT.begin( DEBUG_BAUD );
  DEBUG_PORT.println("TC275");
  
  initNavData();

  if (useTFT) {
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    tft.setRotation(0); 
    rectangle2 ();  
    tft.setTextColor(ILI9341_BLUE);
    tft.setTextSize(4);
    tft.setCursor(0, 20);
    tft.println("WEEDINATOR");
  }

  tone(2,1000,1000);   // pin,pitch,duration
  delay(1000);
  noTone(2);

  pinMode(37, OUTPUT);   // BLUE LED
  pinMode(39, OUTPUT);   // ORANGE LED 
  digitalWrite(37,LOW);
  digitalWrite(39,LOW); 

  while (emicPort.available()) 
  {
    emicIntro();
  }
}

void loop2() 
{
  digitalWrite(39,LOW); // Orange LED
  if (ledBlueState == LOW) 
  {
      ledBlueState = HIGH;
  } else 
  {
      ledBlueState = LOW;
  }
  digitalWrite(37, ledBlueState);
  makeTurnValue = bearingDegrees - headingDegrees;
  if((makeTurnValue)>0)                                        // Does not work for quadrants one and four.
  {
    makeTurn = HIGH;
  }
  else
  {
    makeTurn = LOW;
  }
  
  if((bearingDegrees>=0)&&(bearingDegrees<90)&&(headingDegrees>=270)&&(headingDegrees<360))  // Quadrants one and four makeTurn is reversed.
  {
    if((makeTurnValue)>0)
    {
      makeTurn = LOW;
    }
    else
    {
      makeTurn = HIGH;
      makeTurnValue = 360 + makeTurnValue;
    }
  }


  
  delay(200);
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
  if (millisCalc1 >= 30000)                           // timer  .... 30,000
  {  
    DEBUG_PORT.println("Emic Speech SHOULD be activated ");
    while (emicPort.available()) 
    {
      //DEBUG_PORT.println("Emic serial IS available");
      emicSpeech1();
    }
    previousMillis1Core3=currentMillisCore3;                          // this is the only previousMillis reset!
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////// 
  
  if (millisCalc3 >= 2000)                            // timer
  {  
    previousMillis3Core3=currentMillisCore3; 

     //DEBUG_PORT.print("Initial intervalThree= ");DEBUG_PORT.println(intervalThree);
     //int radiusRatio =3;
     //intervalThree =  (1/(1/intervalThree) - (1/(intervalOne*radiusRatio))); // Makes right hand inside wheel drive a bit slower.
     //resultOne =  1/intervalThree;
     DEBUG_PORT.print("resultOne: ");DEBUG_PORT.println(resultOne,8);
     //resultTwo = intervalOne*radiusRatio;
     DEBUG_PORT.print("resultTwo: ");DEBUG_PORT.println(resultTwo);
     //resultThree = 1/resultTwo;
     DEBUG_PORT.print("resultThree: ");DEBUG_PORT.println(resultThree,8);
     //resultFour = resultTwo - resultThree;
     DEBUG_PORT.print("resultFour: ");DEBUG_PORT.println(resultFour,8);
     //intervalThree = 1/resultFour;
     DEBUG_PORT.print("IntervalThree= ");DEBUG_PORT.println(intervalThree,8);
    
  DEBUG_PORT.println();
  DEBUG_PORT.print("LHS amps max:  ");DEBUG_PORT.print(runningmaxCurrentValueFive,2);
  DEBUG_PORT.print("  RHS amps max:  ");DEBUG_PORT.println(runningmaxCurrentValueSix,2);
  //DEBUG_PORT.print("  samples per cycle:  ");DEBUG_PORT.println(icMax);
  DEBUG_PORT.print("Integer latitude  UBLOX:  ");DEBUG_PORT.println(latitudeUblox);
  DEBUG_PORT.print("Integer longitude UBLOX:  ");DEBUG_PORT.println(longitudeUblox); 
  DEBUG_PORT.print("Bearing    = ");DEBUG_PORT.println(bearingDegrees);
  DEBUG_PORT.print("Heading    = ");DEBUG_PORT.println(headingDegrees);
  DEBUG_PORT.print("Way point  = ");DEBUG_PORT.println(navData.waypointID());
  DEBUG_PORT.print("distanceMM = ");DEBUG_PORT.println(distanceMM);
  DEBUG_PORT.println();
  DEBUG_PORT.print("controlState   Value= ");DEBUG_PORT.println(controlState);
  DEBUG_PORT.print("Final Steering Value= ");DEBUG_PORT.println(finalSteeringValue);
  DEBUG_PORT.print("Make Turn      Value= ");DEBUG_PORT.println(makeTurnValue);
  //DEBUG_PORT.print("Previous steering Value= ");DEBUG_PORT.println(previousFinalSteeringValue);
  //DEBUG_PORT.print("Difference= ");DEBUG_PORT.println(difference);
  //DEBUG_PORT.print("wheelsPosition= ");DEBUG_PORT.println(wheelsPosition);
  DEBUG_PORT.print("Final Drive Value= ");DEBUG_PORT.println(finalDriveValue);
  DEBUG_PORT.print("Steering Value= ");DEBUG_PORT.println(steeringValue);
  //DEBUG_PORT.print("analogueRead A1= ");DEBUG_PORT.println(driveValue);
  DEBUG_PORT.print("IntervalOne= ");DEBUG_PORT.println(intervalOne);
  DEBUG_PORT.print("IntervalTwo= ");DEBUG_PORT.println(intervalTwo);
  DEBUG_PORT.print("IntervalThree= ");DEBUG_PORT.println(intervalThree);
  DEBUG_PORT.print("IntervalFour= ");DEBUG_PORT.println(intervalFour);
  //DEBUG_PORT.print("velocityControlLeft= ");DEBUG_PORT.println(velocityControlLeft);
  //DEBUG_PORT.print("velocityControlRight= ");DEBUG_PORT.println(velocityControlRight); 
  //DEBUG_PORT.print("ATSDState= ");DEBUG_PORT.println(ATSDState);
  DEBUG_PORT.print("Stationary state= ");DEBUG_PORT.println(stationary);
  
  
  if(forwards==HIGH)
    {
    DEBUG_PORT.println("FORWARDS");   
    }
  if(backwards==HIGH)
    {
    DEBUG_PORT.println("BACKWARDS");   
    }
  if(rightHandLock==HIGH)
    {
    DEBUG_PORT.println("RIGHT HAND LOCK");   
    }
  if(leftHandLock==HIGH)
    {
    DEBUG_PORT.println("LEFT HAND LOCK");   
    }
  if(clockW==HIGH)
    {
    DEBUG_PORT.println("GOING CLOCKWISE");   
    }
  if(antiClockW==HIGH)
    {
    DEBUG_PORT.println("GOING ANTI-CLOCKWISE");   
    }
  DEBUG_PORT.println("");
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
        };
      count = sizeof(example);
      memcpy( message, example, count );
    } else {
      DEBUG_PORT.print( '?' );
    }
  }

  navData.readFrom( message, count );

  latitudeUblox  = navData.location().lat();
  longitudeUblox = navData.location().lon();

  distanceMM     = navData.distance() * MM_PER_M;
  headingDegrees = navData.heading();
  bearingDegrees = navData.bearing();

  emicBearing    = bearingDegrees;

} // loop2


void updateTFT() 
{
  if (not useTFT) {
    DEBUG_PORT.println( "TFT updated" );
    return;
  }

  tft.setRotation(0); 
  rectangle2 ();  
  tft.setTextColor(ILI9341_BLUE);
  tft.setTextSize(4);
  tft.setCursor(0, 20);
  tft.println("WEEDINATOR");
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(2);
  tft.setCursor(0, 72);
  tft.println("Compass:  "); 
  tft.setCursor(110, 72);
  tft.println(headingDegrees); 
  tft.setCursor(185, 67);
  tft.println("o"); 
  tft.setCursor(0, 102);
  tft.println("Bearing:");
  tft.setCursor(110, 102);
  tft.println(bearingDegrees,2);
  tft.setCursor(185, 97);
  tft.println("o");
  tft.setCursor(0, 132);
  tft.println("Distance:        mm ");
  tft.setCursor(110, 132);
  tft.println(distanceMM);
  tft.setCursor(0, 160);
  tft.println("Waypoint:            ");
  tft.setCursor(110, 160);
  tft.println(navData.waypointID());
  rectangle1 ();
  tft.setCursor(0, 190);
  tft.println("Steering Val:     ");
  tft.setCursor(170, 190);
  tft.println(finalSteeringValue);
  
  tft.setCursor(0, 218);
  //tft.println("Difference:     ");
  tft.setCursor(170, 218);
  //tft.println(difference);

  tft.setCursor(0, 246);
  //tft.println("Wheels Pos:     ");
  tft.setCursor(170, 246);
  //tft.println(wheelsPosition);
  
  //tft.setTextSize(1);
  //tft.setCursor(0, 274);
  //tft.println("FONA:   LAT             LON");
  //tft.setCursor(70, 274);
  //tft.println(latFona);
  //tft.setCursor(170, 274);
  //tft.println(lonFona);
  tft.setTextSize(4);
  tft.setCursor(15, 240);
  //tft.println("UBLOX:  LAT             LON");
  //tft.setCursor(70, 284);
  tft.println(latitudeUblox);
  tft.setCursor(15, 280);
  tft.println(longitudeUblox);
  
  tft.setTextSize(2);    
  tft.setCursor(0, 302);
  //tft.println("Direction:     ");
  tft.setCursor(170, 302);
  if(forwards==HIGH)
    {
    //tft.println("FORWARDS"); 
    }
  if(backwards==HIGH)
    {
    //tft.println("BACKWARDS");
    }
  //tft.println(finalSteeringValue);

} // updateTFT


void rectangle1 ()
{
          tft.fillRect(0, 186, 250, 135, ILI9341_RED);
          // x,y,(from top left corner)w,h
}
void rectangle2 ()
{
          tft.fillRect(0, 65, 250, 118, ILI9341_RED);
          // x,y,(from top left corner)w,h
}
void emptyEMICport()
{
  //Flush input buffer
  while (emicPort.available()) 
  {
    int inByte = emicPort.read();
    //DEBUG_PORT.write(inByte);
  }
}
void emicDetect()
{
  // Check for response from Emic 2
  emicPort.print('\n');
  // Emic 2 returns ':' if ready for command
  while (emicPort.read() != ':'); 
  // Set volume to maximum
  emicPort.print("V18\n");
  delay(10);
  emicPort.flush(); 
  emicPort.print('N');
  emicPort.print(5); 
  emicPort.print('\n');
  emicPort.print('\n');
  while (emicPort.read() != ':');
  // 'S' command = say 
  cmd = "S"; 
  text1 = "an object has been detected.";
  emicPort.print(cmd + text1 + "\n");

}
void emicIntro()
{
  // Check for response from Emic 2
  emicPort.print('\n');
  // Emic 2 returns ':' if ready for command
  while (emicPort.read() != ':'); 
  // Set volume to maximum
  emicPort.print("V18\n");
  delay(10);
  emicPort.flush(); 
  emicPort.print('N');
  emicPort.print(6); 
  emicPort.print('\n');
  emicPort.print('\n');
  while (emicPort.read() != ':');
  // 'S' command = say 
  cmd = "S"; 
  text1 = "hello world. welcome to the weedinator. Lets go smash the fuck out of some weeds.";
  emicPort.print(cmd + text1 + "\n");
  ; 
}
void emicSpeech1()
{
  // Check for response from Emic 2
  emicPort.print('\n');
  // Emic 2 returns ':' if ready for command
  while (emicPort.read() != ':'); 
  delay(10);
  emicPort.flush(); 
  emicPort.print('\n');
  emicPort.print('\n');
  while (emicPort.read() != ':');
  // 'S' command = say 
  cmd = "S"; 
  text2 = ".. and this is the distance for the weedinator to travel.";
  text3 = "and the bearing is.";
//  textDistanceData = distanceMM;
  //DEBUG_PORT.println("Emic Speech activated ");
  //DEBUG_PORT.print("Distance to travel: ");DEBUG_PORT.println(distanceMM);
  if(makeTurn == HIGH)
  {
    text5 = " Make a clock wise turn. of ..";
  }
  else
  {
    text5 = " Make an anti clock wise turn. of ..";
  }
  text7 = "Start ";
  if (navData.waypointID() != 0) {
    text4 = "We are now going to way point " + navData.waypointID() +
            text2 + distanceMM + " .. milli metres .. " +
            text3 + emicBearing/100 + " .. degrees .. " +
            text5 + makeTurnValue + " .. degrees";
  } else {
    text4 = "No way point";
  }
  text4 += " .. End of message .. ";
  text6 = "[:phone arpa speak on][:rate 190][:n0][ GAA<400,12>DD<200,15>B<200,10>LLEH<200,19>EH<500,22>S<200,18>AH<100,18>MEH<200,16>K<100,13>AH<200,12>][:n0]";
  //DEBUG_PORT.println(text4);
  emicPort.print(cmd + text4 + "\n");
  ; 
}
