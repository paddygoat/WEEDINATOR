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
if ((finalDriveValue<600)&&(finalDriveValue>400))  //Stationary
{
  stationary=HIGH;
}
else
{
  stationary=LOW;
}
if (finalDriveValue>=600)  //Backwards.
{
  backwards = HIGH; forwards = LOW;
  intervalThree = (600000/finalDriveValue)+100; // 140 is max speed.
  intervalFour =  (600000/finalDriveValue)+100; // 140 is max speed.
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
if (finalDriveValue<400)//Forwards.
{
  backwards = LOW; forwards = HIGH;
  intervalThree = (finalDriveValue*2)+300; // 140 is max speed. Right hand wheel.
  intervalFour = (finalDriveValue*2)+300; // 140 is max speed.
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
}
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
#include "tone.h"
/* CPU1 Uninitialised Data */
StartOfUninitialised_CPU1_Variables
/* Put your CPU1 fast access variables that have no initial values here e.g. uint32 CPU1_var; */
EndOfUninitialised_CPU1_Variables

/* CPU1 Initialised Data */
StartOfInitialised_CPU1_Variables
/* Put your CPU1 fast access variables that have an initial value here e.g. uint32 CPU1_var_init = 1; */
long driveValue=0;
long steeringValue =0;
int k=0;
int m=0;
int makeTurnValue =0;
long distanceMetres =0;
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
  intervalFive = 1000 / ACFrequency; 
}
void loop1() 
{
////////////////////////////////////////////////////////////////////////////// 
  ic = 0;
  while (ic < intervalFive)   // AC 50 hertz is equivalent to 20 ms per AC cycle
  {
    ic++;
    delay(1);                 // Capture data over one complete AC cycle.
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
    //SerialASC.print("  LHS amps max:  ");SerialASC.print(runningmaxCurrentValueFive,2);
    //SerialASC.print("  RHS amps max:  ");SerialASC.println(runningmaxCurrentValueSix,2);
//////////////////////////////////////////////////////////////////////////////
  driveValue=0;
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
    if((distanceMetres<500)||(distanceMetres>100000))
    {
      finalDriveValue = 512; // Machine is waiting for FONA module to get new data.
    }
    if((distanceMetres>500)&&(makeTurnValue<200)&&(makeTurnValue>-200))
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
// TwoWire Wire; // Instantiate TwoWire class
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#define TFT_CS     40
#define TFT_RST    24  // you can also connect this to the Arduino reset
#define TFT_DC     22
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

/* LMU uninitialised data */
StartOfUninitialised_LMURam_Variables
int i;
String initiator;
String dataString;
String nothing;
char c;
String y;
String w;
String a;
String b;
String d;
long latFona;
long lonFona;
long latitude;
long longitude;
long latitudeUblox;
long longitudeUblox;
float bearingDegrees;
String laatUbloxString;
String longUbloxString;
String latFonaString;
String lonFonaString;
String lat;
String lon;
String laatUblox;
String longUblox;
long prevDistMetres;

String yawString;
double yaw;
float yawFloat;
float heading;
char q;
char ww[200];
String text4;
String text1;
String text2;
String text3;
String text6;
String text7;
String textFonaData;
String textDistanceData;
String textBearingData;
char fonaCharacter[100];

EndOfUninitialised_LMURam_Variables

/* LMU uninitialised data */
StartOfInitialised_LMURam_Variables
String bearing = "100000";
String bearingString = "100000";
String distance = "100000";
String distanceString = "100000";
char url[120];
long x = 12345678;
long result=0;
String triggerWord = "Important";
int STOP = LOW;
char fonaData[40];
int aa=0;
char distanceCharacter[40];
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
int phpPageInt = 0;
int previousPhpPageInt = 0;
String phpPageString ="0";
String inititiator = "";
String compassString = "1000";
String compass = "1000";
float compassFloat = 0;
int makeTurn = LOW;
String text5 = " .Make a clockwise turn. ";
int ledBlueState = LOW;
EndOfInitialised_LMURam_Variables


void setup2() 
{
  Serial1.begin(9600);   // Emic
  Wire.begin();
  //Wire.begin(); // join i2c bus (address optional for master)
  SerialASC.begin(115200);
  //Serial1.begin(115200);
  SerialASC.println("Ready to test");
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(0); 
  rectangle2 ();  
  tft.setTextColor(ILI9341_BLUE);
  tft.setTextSize(4);
  tft.setCursor(0, 20);
  tft.println("WEEDINATOR");
  tone(2,1000,1000);   // pin,pitch,duration
  delay(1000);
  noTone(2);
  pinMode(37, OUTPUT);   // BLUE LED
  pinMode(39, OUTPUT);   // ORANGE LED 
  digitalWrite(37,LOW);
  digitalWrite(39,LOW); 
  while (Serial1.available()) 
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
  makeTurnValue = heading - compassFloat;
  if((makeTurnValue)>0)                                        // Does not work for quadrants one and four.
  {
    makeTurn = HIGH;
  }
  else
  {
    makeTurn = LOW;
  }
  
  if((heading>=0)&&(heading<90)&&(compassFloat>=270)&&(compassFloat<360))  // Quadrants one and four makeTurn is reversed.
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
  millisCalc3 = currentMillisCore3 - previousMillis3Core3; // Serial print
  millisCalc4 = currentMillisCore3 - previousMillis4Core3; // I2C
/////////////////////////////////////////////////////////////////////////////////////////////////////////  
  if (millisCalc2 >= 5000)                            // timer .....  5,000
  {  
     printTftText();                                  // TFT screen cuases code to pause.
     previousMillis2Core3=currentMillisCore3;         
  }
/////////////////////////////////////////////////////////////////////////////////////////////////////////  
  if (millisCalc1 >= 30000)                           // timer  .... 30,000
  {  
    //SerialASC.println("Emic Speech SHOULD be activated ");
    while (Serial1.available()) 
    {
      //SerialASC.println("Emic serial IS available");
      emicSpeech1();
    }
    previousMillis1Core3=currentMillisCore3;                          // this is the only previousMillis reset!
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////// 
  i=0;
  y="";
  a="";
  b="";
  d="";
  lat="";
  lon="";
  latFonaString="";
  lonFonaString="";
  yawString="";
  
  if (millisCalc3 >= 2000)                            // timer
  {  
    previousMillis3Core3=currentMillisCore3; 

     //SerialASC.print("Initial intervalThree= ");SerialASC.println(intervalThree);
     //int radiusRatio =3;
     //intervalThree =  (1/(1/intervalThree) - (1/(intervalOne*radiusRatio))); // Makes right hand inside wheel drive a bit slower.
     //resultOne =  1/intervalThree;
     SerialASC.print("resultOne: ");SerialASC.println(resultOne,8);
     //resultTwo = intervalOne*radiusRatio;
     SerialASC.print("resultTwo: ");SerialASC.println(resultTwo);
     //resultThree = 1/resultTwo;
     SerialASC.print("resultThree: ");SerialASC.println(resultThree,8);
     //resultFour = resultTwo - resultThree;
     SerialASC.print("resultFour: ");SerialASC.println(resultFour,8);
     //intervalThree = 1/resultFour;
     SerialASC.print("IntervalThree= ");SerialASC.println(intervalThree,8);
    
  SerialASC.println("");
  SerialASC.print("LHS amps max:  ");SerialASC.print(runningmaxCurrentValueFive,2);
  SerialASC.print("  RHS amps max:  ");SerialASC.println(runningmaxCurrentValueSix,2);
  //SerialASC.print("  samples per cycle:  ");SerialASC.println(icMax);
  SerialASC.print("Data captured from Fona: ");SerialASC.println(fonaData);
  SerialASC.print("Integer latitude Fona:   ");SerialASC.println(latFona);
  SerialASC.print("Integer longitude Fona:   ");SerialASC.println(lonFona);
  SerialASC.print("Integer latitude UBLOX:  ");SerialASC.println(latitudeUblox);
  SerialASC.print("Integer longitude UBLOX:  ");SerialASC.println(longitudeUblox); 
  //SerialASC.print("Way point (phpPage)=     ");SerialASC.println(phpPage);
  //SerialASC.print("distanceMetres=          ");SerialASC.println(distanceMetres);
  //SerialASC.print("bearingInt=              ");SerialASC.println(bearingDegrees);
  SerialASC.print("Heading=                 ");SerialASC.println(bearingDegrees/100);
  SerialASC.print("Compass integer:         ");SerialASC.println(compassFloat);
  SerialASC.print("Way point (phpPage)=     ");SerialASC.println(phpPage);
  SerialASC.print("distanceMetres=          ");SerialASC.println(distanceMetres);
  SerialASC.println("");
  //SerialASC.print("phpPageInt=          ");SerialASC.println(phpPageInt);
  //SerialASC.print("previousPhpPageInt=  ");SerialASC.println(previousPhpPageInt);
  //SerialASC.print("phpPageString=       ");SerialASC.println(phpPageString); 
  //SerialASC.print("phpPageString[0]=    ");SerialASC.println(phpPageString[0]); 
  //SerialASC.print("phpPageString[1]=    ");SerialASC.println(phpPageString[1]); 
  //SerialASC.print("phpPageString[2]=    ");SerialASC.println(phpPageString[2]); 
  SerialASC.print("controlState Value= ");SerialASC.println(controlState);
  SerialASC.print("Final Steering Value= ");SerialASC.println(finalSteeringValue);
  SerialASC.print("Make Turn Value= ");SerialASC.println(makeTurnValue);
  //SerialASC.print("Previous steering Value= ");SerialASC.println(previousFinalSteeringValue);
  //SerialASC.print("Difference= ");SerialASC.println(difference);
  //SerialASC.print("wheelsPosition= ");SerialASC.println(wheelsPosition);
  SerialASC.print("Final Drive Value= ");SerialASC.println(finalDriveValue);
  SerialASC.print("Steering Value= ");SerialASC.println(steeringValue);
  //SerialASC.print("analogueRead A1= ");SerialASC.println(driveValue);
  SerialASC.print("IntervalOne= ");SerialASC.println(intervalOne);
  SerialASC.print("IntervalTwo= ");SerialASC.println(intervalTwo);
  SerialASC.print("IntervalThree= ");SerialASC.println(intervalThree);
  SerialASC.print("IntervalFour= ");SerialASC.println(intervalFour);
  //SerialASC.print("velocityControlLeft= ");SerialASC.println(velocityControlLeft);
  //SerialASC.print("velocityControlRight= ");SerialASC.println(velocityControlRight); 
  //SerialASC.print("ATSDState= ");SerialASC.println(ATSDState);
  SerialASC.print("Stationary state= ");SerialASC.println(stationary);
  
  
  if(forwards==HIGH)
    {
    SerialASC.println("FORWARDS");   
    }
  if(backwards==HIGH)
    {
    SerialASC.println("BACKWARDS");   
    }
  if(rightHandLock==HIGH)
    {
    SerialASC.println("RIGHT HAND LOCK");   
    }
  if(leftHandLock==HIGH)
    {
    SerialASC.println("LEFT HAND LOCK");   
    }
  if(clockW==HIGH)
    {
    SerialASC.println("GOING CLOCKWISE");   
    }
  if(antiClockW==HIGH)
    {
    SerialASC.println("GOING ANTI-CLOCKWISE");   
    }
  SerialASC.println("");
  } // millisCalc3
  
/////////////////////////////////////////////////////////////////////////////////////////

  Wire.requestFrom(26, 32);    // request 32 bytes from slave device #26 UBLOX Data // Dont use 25.
  a="";
  bearing="";
  bearingString="";
  distance="";
  distanceString="";
  longUblox="";
  laatUblox="";
  laatUbloxString="";
  longUbloxString="";
  compassString = "";
  compass="";
  //SerialASC.print("C:   ");
  while (Wire.available()) 
  {
    c = Wire.read(); // receive a byte as character
    //SerialASC.print(c);
    if (isalpha(c))         // analyse c for letters
    {
      a=a+c;                // string = string + character
      if (a=="LAAT")
      {
        longUblox="";
        bearing="";
     //   distance="";
        lat="";
        lon="";
        laatUblox=a;
        //SerialASC.print("Trigger word LAAT detected!: ");SerialASC.println(laatUblox);
        a="";
      } 
      if (a=="LOON")
      {
        lat="";
        lon="";
        laatUblox="";
        bearing="";
      //  distance="";
        longUblox=a;
        //SerialASC.print("Trigger word LOON detected!: ");SerialASC.println(longUblox);
        a="";
      } 
      if (a=="BEAR")
      {
        lat="";
        lon="";
        longUblox="";
        laatUblox="";
     //   distance="";
        bearing=a;    // String = string.
        //SerialASC.print("Trigger word BEAR detected!: ");SerialASC.println(bearingDegrees);
        a="";
      } 
      if (a=="DIST")
      {
        lat="";
        lon="";
        longUblox="";
        laatUblox="";
        bearing="";
        distance=a;    // String = string.
        //SerialASC.print("Trigger word DIST detected!: ");SerialASC.println(distanceString);
        a="";
      } 
      if (a=="HEAD")
      {
        lat="";
        lon="";
        longUblox="";
        laatUblox="";
        bearing="";
        distance="";
        compass=a;    // String = string.
        //SerialASC.print("Trigger word HEAD detected!: ");SerialASC.println(compass);
        a="";
      } 
    } 
    if (laatUblox=="LAAT")
    {
      if (isdigit(c))         // analyse c for numerical digit
      {
        laatUbloxString=laatUbloxString+c;                // string = string + character
      }
    }
    if (longUblox=="LOON")
    {
      if (isdigit(c))         // analyse c for numerical digit
      {
        longUbloxString=longUbloxString+c;                // string = string + character
      }
    }
    if (bearing=="BEAR")
    {
      if (isdigit(c))         // analyse c for numerical digit
      {
        bearingString=bearingString+c;                // string = string + character
      }
    }
    if (distance=="DIST")
    {
      if (isdigit(c))         // analyse c for numerical digit
      {
        distanceString=distanceString+c;                // string = string + character
      }
    }
    if (compass=="HEAD")
    {
      if (isdigit(c))         // analyse c for numerical digit
      {
        compassString=compassString+c;                // string = string + character
      }
    }
    if (laatUblox=="LAAT")
    {
      result=(laatUbloxString).toInt();
      latitudeUblox = result;
      //SerialASC.println("");
      //SerialASC.print("latitude Ublox: ");SerialASC.println(latitudeUblox);
    }
    if (longUblox=="LOON")
    {
      result=(longUbloxString).toInt();
      longitudeUblox = result;
      //SerialASC.println("");
      //SerialASC.print("Longitude Ublox: ");SerialASC.println(longitudeUblox);
    }
    if (bearing=="BEAR")
    {
      result=(bearingString).toInt();
      bearingDegrees = result;
      heading = bearingDegrees/100;
      emicBearing=(bearingString).toInt();
      //SerialASC.println("");
      //SerialASC.print("Heading: ");SerialASC.println(heading,2);
    }
    if (distance=="DIST")
    {
      result=(distanceString).toInt();
      distanceMetres = result;
      //SerialASC.println("");
      //SerialASC.print("distanceMetres integer: ");SerialASC.println(distanceMetres);
    }
    if (compass=="HEAD")
    {
      result=(compassString).toInt();
      compassFloat = result/100.00;
      //SerialASC.println("");
      //SerialASC.print("Compass integer: ");SerialASC.println(compassFloat);
    }
  } //////// End of I2C write
    
//  SerialASC.print("Latitude: ");SerialASC.println(latitude);
//  SerialASC.print("Longitude: ");SerialASC.println(longitude);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is where the distance data is compiled into a character .... Do we actually need this any more?
  distanceString =  initiator + distanceString ;
  int n = distanceString.length();
//    SerialASC.print("Distance string in character compile:     ");SerialASC.println(distanceString);   
//  SerialASC.print("Size of string:  ");SerialASC.println(n);
  // Builds the character:
      for (int aa=0;aa<=n;aa++)                    
      {
          distanceCharacter[aa] = distanceString[aa];
      }
//  SerialASC.println("");
//  SerialASC.print("Distance character in character compile:  ");SerialASC.println(distanceCharacter);
//  SerialASC.println("");
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
//  delay(100);
  
}
unsigned long printTftText() 
{
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
  tft.println(compassFloat); 
  tft.setCursor(185, 67);
  tft.println("o"); 
  tft.setCursor(0, 102);
  tft.println("Heading:");
  tft.setCursor(110, 102);
  tft.println(heading,2);
  tft.setCursor(185, 97);
  tft.println("o");
  tft.setCursor(0, 132);
  tft.println("Distance:        mm ");
  tft.setCursor(110, 132);
  tft.println(distanceMetres);
  tft.setCursor(0, 160);
  tft.println("Waypoint:            ");
  tft.setCursor(110, 160);
  tft.println(phpPage);
  rectangle1 ();
  tft.setCursor(0, 190);
  tft.println("Steering Val:     ");
  tft.setCursor(170, 190);
  tft.println(finalSteeringValue);
  
  tft.setCursor(0, 218);
  tft.println("Difference:     ");
  tft.setCursor(170, 218);
  tft.println(difference);

  tft.setCursor(0, 246);
  tft.println("Wheels Pos:     ");
  tft.setCursor(170, 246);
  tft.println(wheelsPosition);
  
  tft.setTextSize(1);
  tft.setCursor(0, 274);
  tft.println("FONA:   LAT             LON");
  tft.setCursor(70, 274);
  tft.println(latFona);
  tft.setCursor(170, 274);
  tft.println(lonFona);

  tft.setCursor(0, 284);
  tft.println("UBLOX:  LAT             LON");
  tft.setCursor(70, 284);
  tft.println(latitudeUblox);
  tft.setCursor(170, 284);
  tft.println(longitudeUblox);
  
  tft.setTextSize(2);    
  tft.setCursor(0, 302);
  tft.println("Direction:     ");
  tft.setCursor(170, 302);
  if(forwards==HIGH)
    {
    tft.println("FORWARDS"); 
    }
  if(backwards==HIGH)
    {
    tft.println("BACKWARDS");
    }
  tft.println(finalSteeringValue);
}
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
void flushSerial1()
{
  while (Serial1.available()) 
  {
    //Flush serial1
    int inByte = Serial1.read();
    //SerialASC.write(inByte);
  }
}
void emicDetect()
{
  // Check for response from Emic 2
  Serial1.print('\n');
  // Emic 2 returns ':' if ready for command
  while (Serial1.read() != ':'); 
  // Set volume to maximum
  Serial1.print('V18\n');
  delay(10);
  Serial1.flush(); 
  Serial1.print('N');
  Serial1.print(5); 
  Serial1.print('\n');
  Serial1.print('\n');
  while (Serial1.read() != ':');
  // 'S' command = say 
  cmd = "S"; 
  text1 = "an object has been detected.";
  Serial1.print(cmd + text1 + "\n");
  ; 
}
void emicIntro()
{
  // Check for response from Emic 2
  Serial1.print('\n');
  // Emic 2 returns ':' if ready for command
  while (Serial1.read() != ':'); 
  // Set volume to maximum
  Serial1.print('V18\n');
  delay(10);
  Serial1.flush(); 
  Serial1.print('N');
  Serial1.print(6); 
  Serial1.print('\n');
  Serial1.print('\n');
  while (Serial1.read() != ':');
  // 'S' command = say 
  cmd = "S"; 
  text1 = "hello world. welcome to the weedinator. Lets go smash the fuck out of some weeds.";
  Serial1.print(cmd + text1 + "\n");
  ; 
}
void emicSpeech1()
{
  int ss=30;
  for (int aa=0;aa<=ss;aa++)                    
  {
  fonaCharacter[aa] = fonaData[aa];
  }
  // Check for response from Emic 2
  Serial1.print('\n');
  // Emic 2 returns ':' if ready for command
  while (Serial1.read() != ':'); 
  delay(10);
  Serial1.flush(); 
  Serial1.print('\n');
  Serial1.print('\n');
  while (Serial1.read() != ':');
  // 'S' command = say 
  cmd = "S"; 
  text1 = " and this is the latitude data from the FONA module.";
  text2 = ".. and this is the distance for the weedinator to travel.";
  text3 = "and the bearing is.";
  textFonaData = fonaData;
  textDistanceData = distanceCharacter;
  //SerialASC.println("Emic Speech activated ");
  //SerialASC.print("Distance to travel: ");SerialASC.println(distanceMetres);
  if(makeTurn == HIGH)
  {
    text5 = " Make a clock wise turn. of ..";
  }
  else
  {
    text5 = " Make an anti clock wise turn. of ..";
  }
  text7 = "Start ";
  text4 = "We are now going to way point " + phpPageString + text2 + distanceMetres + " .. milli metres .. " 
  + text3 + emicBearing/100 + " .. degrees .. " + text5 + makeTurnValue + " .. degrees .. End of message .. ";
  text6 = "[:phone arpa speak on][:rate 190][:n0][ GAA<400,12>DD<200,15>B<200,10>LLEH<200,19>EH<500,22>S<200,18>AH<100,18>MEH<200,16>K<100,13>AH<200,12>][:n0]";
  //SerialASC.println(text4);
  Serial1.print(cmd + text4 + "\n");
  ; 
}
