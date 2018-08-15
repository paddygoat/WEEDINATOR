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
int pixyBarData;
int pixyBarcode;

EndOfUninitialised_LMURam_Variables

/* LMU uninitialised data */
StartOfInitialised_LMURam_Variables

#include "TC275.h"

float runningmaxCurrentValueFive = 1;
float runningmaxCurrentValueSix = 1;
float resultOne = 1.0000;
int ledStateOne = LOW;   
int ledStateTwo = LOW;   
int ledStateThree = LOW;
int ledStateFour = LOW;
int stateSeven = LOW;

unsigned long previousMicrosOne = 0;
unsigned long previousMicrosTwo = 0;
unsigned long previousMicrosThree = 0;
unsigned long previousMicrosFour = 0;
unsigned long previousMillisSeven = 0;

unsigned long previousMicrosTimerAsyncLeftA = 0; 
unsigned long previousMicrosTimerAsyncLeftB = 0; 
unsigned long intervalTimerAsyncLeftA = 0;
unsigned long intervalTimerAsyncLeftB = 0;
float speedTimerAsyncLeftA = 0.001;
float speedTimerAsyncLeftB = 0.001;
int asyncStateLeft = HIGH;
unsigned long previousMicrosTimerAsyncRightA = 0; 
unsigned long previousMicrosTimerAsyncRightB = 0; 
unsigned long intervalTimerAsyncRightA = 0;
unsigned long intervalTimerAsyncRightB = 0;
float speedTimerAsyncRightA = 0.001;
float speedTimerAsyncRightB = 0.001;
int asyncStateRight = HIGH;

float intervalOne = 1000; // Right hand wheel turn speed. Bigger is faster.
float intervalTwo = 1000; // Left hand wheel turn speed.
float intervalThree = 1000; // Right hand wheel drive speed. Is this left wheel?
float intervalFour = 1000; // Left hand wheel drive speed.
int intervalOperateCNC = 10000; // CNC timer.
int intervalMoveSlightlyForwards = 2000; // Move slightly forwards after CNC action timer.

int asyncCount =0;

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

int controlState = LOW;  // Manual or autonomous(HGH)
int navState=LOW;        // Use Pixy or GPS
int potentiometerState=LOW;    // Use barcodes or not. Drive potentiometer will be overridden when high.
//int pixyBarData=0;

int dirStateCNCX = LOW; 
int dirStateCNCY = LOW; 
int dirStateCNCZ = LOW;
int dirStateCNCR = LOW; 
int speedCNCX = 1000;
int speedCNCY = 1000;
int speedCNCZ = 1000;
int speedCNCR = 1000;
int stepStateCNCX = LOW;
int stepStateCNCY = LOW;
int stepStateCNCZ = LOW;
int stepStateCNCR = LOW;
uint8_t LSFXRHS = LOW;          // RHS Switch open x axis.
uint8_t LSFXLHS = LOW;          // LHS Switch open x axis.
uint8_t LSBX = HIGH;
uint8_t LSRY = HIGH;          // Switch open y axis. LSRY = limit switch right y.
uint8_t LSLY = HIGH;          // D43
uint8_t LSUZ = HIGH;          // D41
uint8_t LSDZ = HIGH;         // limit switch Z down.
unsigned long prevMicrosCNCXOne = 0; 
unsigned long prevMicrosCNCYOne = 0; 
unsigned long prevMicrosCNCZOne = 0; 
unsigned long prevMicrosCNCROne = 0; 
float intervalCNCXOne = 0;
float intervalCNCYOne = 0;
float intervalCNCZOne = 0;
float intervalCNCROne = 0;
int XZeroingState = LOW;
int YZeroingState = LOW;
int ZZeroingState = LOW;
long XZeroingStepsTotal = 1000;
long YZeroingStepsTotal = 1000;
long ZZeroingStepsTotal = 1000;
long XZeroingStep = 0;
long YZeroingStep = 0;
long ZZeroingStep = 0;

uint8_t setupStateX = LOW;           // Change to HIGH to skip CNC setups.
uint8_t setupStateY = LOW;
uint8_t setupStateZ = LOW;

String operationNumber = " no c n c operation ";
bool operationZero = false;
bool operationOne = false;
bool operationTwo = false;
bool operationThree = false;
bool operationFour = false;
bool operationFive = false;
bool operationSix = false;
bool operationSeven = false;
bool operationEight = false;
bool operationNine = false;
bool operationTen = false;
bool operationEleven = false;
bool operationTwelve = false;
bool operationThirteen = false;
bool operationFourteen = false;
bool operationFifteen = false;
bool operationSixteen = false;
bool operationSeventeen = false;
bool operationEighteen = false;
bool operationNineteen = false;
bool operationTwenty = false;
bool operationTwentyOne = false;
bool operationState = false;
bool weedingBegin = false;
bool move2ColumnsForwards = false;
bool bothXAxisLimitSwitches = false;
bool barcodeReached = false;
bool overideSteering = false;

int currentSensorValueRAxis =0;
int finalCurrentSensorValueRAxis =0;
long runningCurrentValueRAxis =0;
float RAxisCurrentSensorWeighting =0;

long moveStepsForwards =0;
long totalMoveStepsForwards =79601;             // 558.8 mm (22").

EndOfInitialised_LMURam_Variables


/*** Core 0 ***/
void setup()
{  
  delay(5000);
  pinMode(5,OUTPUT); //STEP Steer Motor
  pinMode(6,OUTPUT); //DIRECTION HIGH is clockwise
  pinMode(7,OUTPUT); //STEP Steer Motor
  pinMode(8,OUTPUT); //DIRECTION HIGH is clockwise
  pinMode(9,OUTPUT); //STEP Drive Motor
  pinMode(10,OUTPUT); //DIRECTION HIGH is clockwise
  pinMode(11,OUTPUT); //STEP Drive Motor
  pinMode(12,OUTPUT); //DIRECTION HIGH is clockwise

  pinMode(35, OUTPUT);   // Step x axis RHS
  digitalWrite(35,LOW);
  pinMode(13, OUTPUT);   // Step x axis LHS
  digitalWrite(13,LOW);
  pinMode(53, OUTPUT);   // Direction x axis, LOW is forwards
  digitalWrite(53,LOW);
  pinMode(33, OUTPUT);   // Step y axis
  digitalWrite(33,LOW);
  pinMode(51, OUTPUT);   // Direction y axis, LOW is forwards
  digitalWrite(51,LOW);
  pinMode(31, OUTPUT);   // Step z axis
  pinMode(27, OUTPUT);   // Direction z axis, LOW is forwards
  pinMode(29, OUTPUT);   // Step r axis
  
  pinMode(25,INPUT_PULLUP);    // Turn on/off drive motors
  pinMode(23,INPUT_PULLUP);    // Control state switch eg manual / autonomous
  //pinMode(39,INPUT_PULLUP);    // 
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
    operationNumber = " no c n c operation ";
    overideSteering=false;
  }
  else
  {
    overideSteering=true;
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  if((weedingBegin == true)&&(controlState==LOW))  // weedingBegin occurs in void opThree.
  {
    speedCNCZ = 2000;                                             // 1 = one Hz.
    ZZeroingStepsTotal = 20000;                                // ? mm
    dirStateCNCZ = LOW;                                        // HIGH is down.
    if(ZZeroingStepsTotal > ZZeroingStep)
    {
      CNC_TIMER_Z();
    }   
  }
  if((weedingBegin == true)&&(controlState==HIGH))
  {
    ZZeroingStep =0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //RAxisCurrentSensorWeighting = (finalCurrentSensorValueRAxis*finalCurrentSensorValueRAxis)/(520*520);
    if(finalCurrentSensorValueRAxis<525)
    {
      dirStateCNCZ = HIGH;                                     // HIGH is down.
      speedCNCZ = 1000;                                         // 1 = one Hz.
      CNC_TIMER_Z();
    }
    else
    {
      dirStateCNCZ = LOW;                                      // LOW is up.
      speedCNCZ = 1000;  //*RAxisCurrentSensorWeighting;            // 1 = one Hz.
      //DEBUG_PORT.print("RAxisCurrentSensorWeighting: ");DEBUG_PORT.println(RAxisCurrentSensorWeighting,4);
      //DEBUG_PORT.print("speedCNCZ:                   ");DEBUG_PORT.println(speedCNCZ,4);
      CNC_TIMER_Z();
    }     
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  }    // if((weedingBegin == true)&&(controlState==HIGH))
  
  if(operationState == true)                       // Repeats all operations.
  {
    operationZero = false;                          // Resets all CNC step counters to zero, except one and two, which are skipped.
    operationThree = false;
    operationFour = false;
    operationFive = false;
    operationSix = false;
    operationSeven = false;
    operationEight = false;
    operationNine = false;
    operationTen = false;
    operationEleven = false;
    operationTwelve = false;
    operationThirteen = false;
    operationFourteen = false;
    operationFifteen = false;
    operationState = false;
  }
  if(((setupStateX==LOW)||(setupStateY==LOW)||(setupStateZ==LOW))&&(controlState==HIGH))
  {
    CNC_SETUP_X();  // Else     digitalWrite(35,LOW);
    CNC_SETUP_Y();
    CNC_SETUP_Z();
  }
  else
  {
    if((setupStateX==HIGH)&&(setupStateY==HIGH)&&(setupStateZ==HIGH)&&(controlState==HIGH))             // Carry on with the next CNC routine here.
    {
////////////////////////////////////////////////////////////////////////////////////////////////////
      speedCNCR = 1500;                                          // Step Hertz 500 is good.
      //CNC_TIMER_R();
      opZero();
      opOne();
      opTwo();
      opThree();
      opFour();
      opFive();
      opSix();
      opSeven();
      opEight();
      opNine();
      opTen();
      opEleven();
      opTwelve();
      opThirteen();
      opFourteen();
      opFifteen();
////////////////////////////////////////////////////////////////////////////////////////////////////
    }      // if controlState == HIGH
  }        // else  
  // Might want to move digital reads to core1.
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
void ampflowMotor()
{
  if((finalDriveValue<=715)&&(finalDriveValue>=615))         // Dead zone is 100.
  {
    speedTimerAsyncLeftA = 665;
  }
  if(finalDriveValue<615)
  {
    speedTimerAsyncLeftA = (finalDriveValue - 665)/10 + 665;
    speedTimerAsyncRightA = speedTimerAsyncLeftA;
  }
  if(finalDriveValue>715)
  {
    speedTimerAsyncLeftA = (finalDriveValue - 665)/10 + 665;
    speedTimerAsyncRightA = speedTimerAsyncLeftA;
  }
  // finalDriveValue - forwards is less than 615, backwards is greater than 715.
  moveRightMotor();
  moveLeftMotor(); 
} // ampflowMotor
void moveMotors()
{
  unsigned long currentMicros = micros();
  //speedTimerAsyncLeftA = 665;                                     // Between 1000 and 500. Neutral is 665.
  intervalTimerAsyncLeftA = 1000000/speedTimerAsyncLeftA;
  speedTimerAsyncLeftB = 50;                                      // B MUST be slower than A. Speed of 50 Hz == 20 ms.
  intervalTimerAsyncLeftB = 1000000/speedTimerAsyncLeftB; 
  intervalTimerAsyncLeftB = intervalTimerAsyncLeftB + intervalTimerAsyncLeftA;

  if ((currentMicros - previousMicrosTimerAsyncLeftA) >= intervalTimerAsyncLeftA)
  {
    previousMicrosTimerAsyncLeftA = currentMicros;
    asyncStateLeft = LOW;
    if ((currentMicros - previousMicrosTimerAsyncLeftB) >= intervalTimerAsyncLeftB)
    {
      asyncStateLeft = HIGH;
      previousMicrosTimerAsyncLeftB = currentMicros;
    }
    digitalWrite(9,asyncStateLeft);
    digitalWrite(11,asyncStateLeft);
  }
}
void moveRightMotor()                                         // For steering when stationary.
{
  unsigned long currentMicros = micros();
  //speedTimerAsyncRightA = 655;                                     // Between 1000 and 500. Neutral is 665.
  intervalTimerAsyncRightA = 1000000/speedTimerAsyncRightA;
  speedDifferential();                                             // Adjusts inner and outer wheel speeds when steering turn speed is zero.
  speedTimerAsyncRightB = 50;                                      // B MUST be slower than A. Speed of 50 Hz == 20 ms.
  intervalTimerAsyncRightB = 1000000/speedTimerAsyncRightB; 
  intervalTimerAsyncRightB = intervalTimerAsyncRightB + intervalTimerAsyncRightA;

  if ((currentMicros - previousMicrosTimerAsyncRightA) >= intervalTimerAsyncRightA)
  {
    previousMicrosTimerAsyncRightA = currentMicros;
    asyncStateRight = LOW;
    if ((currentMicros - previousMicrosTimerAsyncRightB) >= intervalTimerAsyncRightB)
    {
      asyncStateRight = HIGH;
      previousMicrosTimerAsyncRightB = currentMicros;
    }
    digitalWrite(9,asyncStateRight);
  }
}
void moveLeftMotor()
{
  unsigned long currentMicros = micros();
  //speedTimerAsyncLeftA = 655;                                     // Between 1000 and 500. Neutral is 665.
  intervalTimerAsyncLeftA = 1000000/speedTimerAsyncLeftA;
  speedDifferential();                                            // Adjusts inner and outer wheel speeds when steering turn speed is zero.
  speedTimerAsyncLeftB = 50;                                      // B MUST be slower than A. Speed of 50 Hz == 20 ms.
  intervalTimerAsyncLeftB = 1000000/speedTimerAsyncLeftB; 
  intervalTimerAsyncLeftB = intervalTimerAsyncLeftB + intervalTimerAsyncLeftA;

  if ((currentMicros - previousMicrosTimerAsyncLeftA) >= intervalTimerAsyncLeftA)
  {
    previousMicrosTimerAsyncLeftA = currentMicros;
    asyncStateLeft = LOW;
    if ((currentMicros - previousMicrosTimerAsyncLeftB) >= intervalTimerAsyncLeftB)
    {
      asyncStateLeft = HIGH;
      previousMicrosTimerAsyncLeftB = currentMicros;
    }
    digitalWrite(11,asyncStateLeft);
  }
}
void torqueDifferential()
{
  // Make RH wheel turn slightly faster if current in LH wheel too high. Three and five is LH and Four and six is RH.
  // Weighting is 6:4.
  intervalFour = ((0.60*intervalFour) + (0.40*(intervalFour * (runningmaxCurrentValueSix / runningmaxCurrentValueFive))));
}
void speedDifferential()
{
  ferrets = map(wheelsPosition,-1,-15000,1,15000);  //Changes all negative values of wheelsPosition to positive.
  if((wheelsPosition>200)&&(stationary==LOW))       // NOT stationary.
  {
     intervalTimerAsyncLeftA =  intervalTimerAsyncLeftA +(wheelsPosition*intervalTimerAsyncLeftA)/900000; // Makes right hand inside wheel drive slower.
  }
  if((wheelsPosition<-200)&&(stationary==LOW))       // NOT stationary.
  {
     intervalTimerAsyncRightA =  intervalTimerAsyncRightA +(ferrets*intervalTimerAsyncRightA)/900000; // Makes left hand inside wheel drive slower.
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void turningWhenStationary() 
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
     speedTimerAsyncLeftA = 650;                                     // Between 1000 and 500. Neutral is 665.
     speedTimerAsyncRightA = 685;                                     // Between 1000 and 500. Neutral is 665.
     moveLeftMotor();
     moveRightMotor();
  }
  if((antiClockW==HIGH)&&(rightHandLock==HIGH)&&(stationary==HIGH)&&(actuallySteering==HIGH)) // Actually steering when stationary
  {
     digitalWrite(10,LOW); // Forwards
     digitalWrite(12,HIGH);  // Backwards
     speedTimerAsyncLeftA = 685;                                     // Between 1000 and 500. Neutral is 665.
     speedTimerAsyncRightA = 650;                                     // Between 1000 and 500. Neutral is 665.
     moveLeftMotor();
     moveRightMotor();
  }
  if((clockW==HIGH)&&(leftHandLock==HIGH)&&(stationary==HIGH)&&(actuallySteering==HIGH)) // Actually steering when stationary
  {
     digitalWrite(10,HIGH); // Backwards
     digitalWrite(12,LOW);  // Forwards
     speedTimerAsyncLeftA = 650;                                     // Between 1000 and 500. Neutral is 665.
     speedTimerAsyncRightA = 685;                                     // Between 1000 and 500. Neutral is 665.
     moveLeftMotor();
     moveRightMotor();
  }
  if((antiClockW==HIGH)&&(leftHandLock==HIGH)&&(stationary==HIGH)&&(actuallySteering==HIGH)) // Actually steering when stationary
  {
     digitalWrite(10,LOW); // Forwards
     digitalWrite(12,HIGH);  // Backwards
     speedTimerAsyncLeftA = 685;                                     // Between 1000 and 500. Neutral is 665.
     speedTimerAsyncRightA = 650;                                     // Between 1000 and 500. Neutral is 665.
     moveLeftMotor();
     moveRightMotor();
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
}
void loop1() 
{
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
    //DEBUG_PORT.print("finalCurrentSensorValueRAxis:  ");DEBUG_PORT.println(finalCurrentSensorValueRAxis);
    //DEBUG_PORT.print("currentSensorValueRAxis:       ");DEBUG_PORT.println(currentSensorValueRAxis);  
    //DEBUG_PORT.print("runningCurrentValueRAxis:      ");DEBUG_PORT.println(runningCurrentValueRAxis);
     
    runningmaxCurrentValueFive = (maxCurrentValueFive - 523)/80;
    runningmaxCurrentValueSix = (maxCurrentValueSix - 523)/80;
    maxCurrentValueFive = 0;
    maxCurrentValueSix = 0;
    //DEBUG_PORT.print("  LHS amps max:  ");DEBUG_PORT.print(runningmaxCurrentValueFive,2);
    //DEBUG_PORT.print("  RHS amps max:  ");DEBUG_PORT.println(runningmaxCurrentValueSix,2);
//////////////////////////////////////////////////////////////////////////////
  long driveValue=0;
  k=0;
  if(controlState==LOW)                // Autonomous mode disabled !!!!!!!!!!!!
  {
    while(k<100)
    { 
      driveValue = driveValue + analogRead(A1);
      k++;
    }
      finalDriveValue = driveValue/k;
  }
////////////////////////////////////////////////////////////////////////////////////////
  //ArduinoPwmFreq(4,390); // set PWM freq on pin 4 to 1 kHz
  //analogWrite(4,finalDriveValue/4);
///////////////////////////////////////////////////////////////////////////////////////  
  //controlState = digitalRead(23); 
  if (overideSteering==false)
  {
    if(controlState==HIGH)                   // Autonomous mode disabled !!!!!!!!!!!!
    {
      steeringValue = makeTurnValue*5 + 512; // Adjust steering to compass sensitivity
      finalSteeringValue = 30*steeringValue;
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
  }   // if (overideSteering==false)
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

int pixyPanData;

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
  digitalWrite(37,LOW);
  delay(2000);
  while (emicPort.available()) 
  {
    emicIntro();
  }
}

void loop2() 
{
  tone(2,pixyBarData*20);   // pin,pitch,duration
  if (ledBlueState == LOW) 
  {
      ledBlueState = HIGH;
  } else 
  {
      ledBlueState = LOW;
  }
  digitalWrite(37, ledBlueState);
if(navState==HIGH)
  { 
    makeTurnValue = pixyPanData - 180 -5;   // Pixy +10 = 60mm to right.
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
  if (millisCalc1 >= 10000)                           // timer  .... 30,000
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

     //DEBUG_PORT.print("Initial intervalThree= ");DEBUG_PORT.println(intervalThree);
     //int radiusRatio =3;
     //intervalThree =  (1/(1/intervalThree) - (1/(intervalOne*radiusRatio))); // Makes right hand inside wheel drive a bit slower.
     //resultOne =  1/intervalThree;
     //DEBUG_PORT.print("resultOne: ");DEBUG_PORT.println(resultOne,8);
     //resultTwo = intervalOne*radiusRatio;
     //DEBUG_PORT.print("resultTwo: ");DEBUG_PORT.println(resultTwo);
     //resultThree = 1/resultTwo;
     //DEBUG_PORT.print("resultThree: ");DEBUG_PORT.println(resultThree,8);
     //resultFour = resultTwo - resultThree;
     //DEBUG_PORT.print("resultFour: ");DEBUG_PORT.println(resultFour,8);
     //intervalThree = 1/resultFour;
     //DEBUG_PORT.print("IntervalThree= ");DEBUG_PORT.println(intervalThree,8);
    
  DEBUG_PORT.println();
  //DEBUG_PORT.print("bothXAxisLimitSwitches:  ");DEBUG_PORT.println(bothXAxisLimitSwitches);
  //DEBUG_PORT.print("LSFXLHS:  ");DEBUG_PORT.println(LSFXLHS);
  //DEBUG_PORT.print("maxCurrentValueFive:  ");DEBUG_PORT.println(maxCurrentValueFive,2);
  //DEBUG_PORT.print("maxCurrentValueSix:  ");DEBUG_PORT.println(maxCurrentValueSix,2);
  
  //DEBUG_PORT.print("LHS amps max:  ");DEBUG_PORT.print(runningmaxCurrentValueFive,2);
  //DEBUG_PORT.print("  RHS amps max:  ");DEBUG_PORT.println(runningmaxCurrentValueSix,2);
  //DEBUG_PORT.print("  samples per cycle:  ");DEBUG_PORT.println(icMax);
  //DEBUG_PORT.print("Integer latitude  UBLOX:  ");DEBUG_PORT.println(latitudeUblox);
  //DEBUG_PORT.print("Integer longitude UBLOX:  ");DEBUG_PORT.println(longitudeUblox); 
  //DEBUG_PORT.print("Bearing    = ");DEBUG_PORT.println(bearingDegrees);
  //DEBUG_PORT.print("Heading    = ");DEBUG_PORT.println(headingDegrees);
  //DEBUG_PORT.print("Way point  = ");DEBUG_PORT.println(navData.waypointID());
  //DEBUG_PORT.print("distanceMM = ");DEBUG_PORT.println(distanceMM);
  //DEBUG_PORT.println();
  DEBUG_PORT.print("controlState value = ");DEBUG_PORT.println(controlState);
//  DEBUG_PORT.print("Final Steering Value= ");DEBUG_PORT.println(finalSteeringValue);
//  DEBUG_PORT.print("Make Turn      Value= ");DEBUG_PORT.println(makeTurnValue);
  //DEBUG_PORT.print("Previous steering Value= ");DEBUG_PORT.println(previousFinalSteeringValue);
  //DEBUG_PORT.print("Difference= ");DEBUG_PORT.println(difference);
  //DEBUG_PORT.print("wheelsPosition= ");DEBUG_PORT.println(wheelsPosition);
  DEBUG_PORT.print("Final Drive Value = ");DEBUG_PORT.println(finalDriveValue);
  DEBUG_PORT.print("speedTimerAsyncLeftA =  ");DEBUG_PORT.println(speedTimerAsyncLeftA);
  DEBUG_PORT.print("intervalTimerAsyncRightA =  ");DEBUG_PORT.println(intervalTimerAsyncRightA);  
  DEBUG_PORT.print("intervalTimerAsyncLeftA =   ");DEBUG_PORT.println(intervalTimerAsyncLeftA); 
  DEBUG_PORT.print("overideSteering =   ");DEBUG_PORT.println(overideSteering);
  
//  DEBUG_PORT.print("Steering Value= ");DEBUG_PORT.println(steeringValue);
  //DEBUG_PORT.print("analogueRead A1= ");DEBUG_PORT.println(driveValue);
  //DEBUG_PORT.print("IntervalOne= ");DEBUG_PORT.println(intervalOne);
  //DEBUG_PORT.print("IntervalTwo= ");DEBUG_PORT.println(intervalTwo);
  //DEBUG_PORT.print("IntervalThree= ");DEBUG_PORT.println(intervalThree);
  //DEBUG_PORT.print("IntervalFour= ");DEBUG_PORT.println(intervalFour);
  //DEBUG_PORT.print("velocityControlLeft= ");DEBUG_PORT.println(velocityControlLeft);
  //DEBUG_PORT.print("velocityControlRight= ");DEBUG_PORT.println(velocityControlRight); 
  //DEBUG_PORT.print("ATSDState= ");DEBUG_PORT.println(ATSDState);
  DEBUG_PORT.print("Stationary state= ");DEBUG_PORT.println(stationary);
  DEBUG_PORT.print("Pixy pan data  = ");DEBUG_PORT.println(pixyPanData);
  DEBUG_PORT.print("Pixy bar data  = ");DEBUG_PORT.println(pixyBarData);
  DEBUG_PORT.print("Pixy barcode  = ");DEBUG_PORT.println(pixyBarcode);
  DEBUG_PORT.print("NavState  = ");DEBUG_PORT.println(navState);
  DEBUG_PORT.print("moveStepsForwards: ");DEBUG_PORT.println(moveStepsForwards);
  DEBUG_PORT.print("barcodeReached: ");DEBUG_PORT.println(barcodeReached);
  
  
//  if(forwards==HIGH)
//    {
//    DEBUG_PORT.println("FORWARDS");   
//    }
//  if(backwards==HIGH)
//    {
//    DEBUG_PORT.println("BACKWARDS");   
//    }
//  if(rightHandLock==HIGH)
//    {
//    DEBUG_PORT.println("RIGHT HAND LOCK");   
//    }
//  if(leftHandLock==HIGH)
//    {
//    DEBUG_PORT.println("LEFT HAND LOCK");   
//    }
//  if(clockW==HIGH)
//    {
//    DEBUG_PORT.println("GOING CLOCKWISE");   
//    }
//  if(antiClockW==HIGH)
//    {
//    DEBUG_PORT.println("GOING ANTI-CLOCKWISE");   
//    }
//  DEBUG_PORT.println("");
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
  tft.println("Heading:    "); 
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
  tft.setCursor(150, 160);
  tft.println(pixyBarcode);
  rectangle1 ();
  tft.setCursor(0, 190);
  tft.println("Pixy pan:    bar:  ");
  tft.setCursor(110, 190);
  tft.println(pixyPanData);
  tft.setCursor(210, 190);
  tft.println(pixyBarData);
    
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
  tft.setCursor(5, 215);
/////////////////////////////////////////////////////////////
  if(controlState==HIGH)
  {
    tft.println("    AUTO");
  }
  else
  {
    tft.println("   MANUAL");
  } 
  tft.setCursor(60, 215);  
  if(navState==HIGH)
  {
    tft.println("     * PIXY");
  }
  else
  {
    tft.println("     * GPS");
  }
//////////////////////////////////////////////////////////////
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
  text1 = "hello world. welcome to the weedinator. Lets go smash the crap out of some weeds.";
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
  emicPort.print("V18\n");
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

  text4 = "We are currently doing " + operationNumber ;

  text4 += " .. End of message .. ";
  text6 = "[:phone arpa speak on][:rate 190][:n0][ GAA<400,12>DD<200,15>B<200,10>LLEH<200,19>EH<500,22>S<200,18>AH<100,18>MEH<200,16>K<100,13>AH<200,12>][:n0]";
  //DEBUG_PORT.println(text4);
  emicPort.print(cmd + text4 + "\n");
  ; 
}

///////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
void CNC_SETUP_X()
{  
//////////////////////////////////////////////////////////////////////////////////////////
  speedCNCX = 2000;                                         // 1 = one Hz.
  XZeroingStepsTotal = 22768;                               // Middle.
  dirStateCNCX = LOW;                                       // LOW is Forwards.

  // controlState==HIGH is autonomous mode.
  // Initial movement forward for zeroing:
  if((controlState==HIGH)&&(XZeroingState==LOW))            // Forwards and backwards switch open.
  {
    operationNumber = "zeroing all axis ease";
    XZeroingStep=0;
    intervalCNCXOne = 1000000/speedCNCX;                       // interval is smaller for faster speed.
    CNC_TIMER_X();
  }
  else
  {
    digitalWrite(35,LOW);
    digitalWrite(13,LOW);
  }                                                           // Initial movement forward for zeroing ends
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  if((LSFXRHS==HIGH)&&(controlState==HIGH))                       // X axis switch is closed and autonomous(HGH)
  {
    dirStateCNCX = HIGH;                                       // go backwards along X axis.
    speedCNCX = 100;
    intervalCNCXOne = 1000000/speedCNCX;                       // interval is smaller for faster speed.
    CNC_TIMER_X();
    XZeroingState=HIGH;
  }
  if(controlState==LOW)
  {
    XZeroingState=LOW;
  }
////////////////////////////////////////////////////////////////////////////////////////
// Next, move X axis to middle and stop:
  if((XZeroingState==HIGH)&&(XZeroingStepsTotal > XZeroingStep))
  {
    dirStateCNCX = HIGH;                                     // LOW is Forwards.
    if((LSBX==HIGH)&&(controlState==HIGH))                   //  backwards switch open.
    {
      intervalCNCXOne = 1000000/speedCNCX;                      // interval is smaller for faster speed.
      CNC_TIMER_X();
    }
    else
    {
      digitalWrite(35,LOW);
      digitalWrite(13,LOW);
    }     
  }             // if(XZeroing == HIGH)
  if(XZeroingStepsTotal <= XZeroingStep)                     // Tells us that setup of X axis is complete.
  {
    setupStateX = HIGH;
    //XZeroingStep=0;
  }
////////////////////////////////////////////////////////////////////////////////////////
}               // CNC SETUP X
void CNC_SETUP_Y()
{  
//////////////////////////////////////////////////////////////////////////////////////////
  speedCNCY = 2000;                                         // 1 = one Hz.
  YZeroingStepsTotal = 33440;                               // Middle. (1 and half grids.)
  dirStateCNCY = LOW;                                       // LOW is Forwards.

  // controlState==HIGH is autonomous mode.
  // Initial movement forward for zeroing:
  if((controlState==HIGH)&&(YZeroingState==LOW))            // Forwards and backwards switch open.
  {
    YZeroingStep=0;
    intervalCNCYOne = 1000000/speedCNCY;                       // interval is smaller for faster speed.
    CNC_TIMER_Y();
  }
  else
  {
    digitalWrite(33,LOW);
  }                                                           // Initial movement forward for zeroing ends
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  if((LSLY==HIGH)&&(controlState==HIGH))                       // Y axis switch is closed and autonomous(HGH)
  {
    dirStateCNCY = HIGH;                                       // go right along Y axis.
    speedCNCY = 100;
    intervalCNCYOne = 1000000/speedCNCY;                       // interval is smaller for faster speed.
    CNC_TIMER_Y();
    YZeroingState=HIGH;
  }
  if(controlState==LOW)
  {
    YZeroingState=LOW;
  }
////////////////////////////////////////////////////////////////////////////////////////
// Next, stage 3, move Y axis to middle and stop:
  if((YZeroingState==HIGH)&&(YZeroingStepsTotal > YZeroingStep))
  {
    //DEBUG_PORT.println("YZeroingStep:  ");DEBUG_PORT.println(YZeroingStep);
    dirStateCNCY = HIGH;                                     // LOW is left.
    if(controlState==HIGH)
    {
      //DEBUG_PORT.println("We reached point 2");
      intervalCNCYOne = 1000000/speedCNCY;                      // interval is smaller for faster speed.
      CNC_TIMER_Y();
    }
    else
    {
      digitalWrite(33,LOW);
    }     
  }             // if(YZeroing == HIGH)
  if(YZeroingStepsTotal <= YZeroingStep)                     // Tells us that setup of Y axis is complete.
  {
    setupStateY = HIGH;
    //YZeroingStep=0;
  }
////////////////////////////////////////////////////////////////////////////////////////
}               // CNC SETUP Y
void CNC_SETUP_Z()
{  
//////////////////////////////////////////////////////////////////////////////////////////
  speedCNCZ = 3000;                                         // 1 = one Hz.
  ZZeroingStepsTotal = 10000;
  dirStateCNCZ = LOW;                                       // LOW is upwards ?

  // controlState==HIGH is autonomous mode.
  // Initial movement forward for zeroing:
  if((controlState==HIGH)&&(ZZeroingState==LOW))            // Upwards switch open.
  {
    ZZeroingStep=0;
    intervalCNCZOne = 1000000/speedCNCZ;                       // interval is smaller for faster speed.
    CNC_TIMER_Z();
  }
  else
  {
    digitalWrite(31,LOW);
  }                                                           // Initial movement forward for zeroing ends
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  if((LSUZ==HIGH)&&(controlState==HIGH))                       // Z axis switch is closed and autonomous(HGH)
  {
    dirStateCNCZ = HIGH;                                       // go up along Z axis.
    speedCNCZ = 40;
    intervalCNCZOne = 1000000/speedCNCZ;                       // interval is smaller for faster speed.
    CNC_TIMER_Z();
    ZZeroingState=HIGH;
  }
  if(controlState==LOW)
  {
    ZZeroingState=LOW;
  }
////////////////////////////////////////////////////////////////////////////////////////
// Next, stage 3, move Z axis to middle and stop:
  if((ZZeroingState==HIGH)&&(ZZeroingStepsTotal > ZZeroingStep))
  {
    //DEBUG_PORT.println("We reached point 1");
    dirStateCNCZ = HIGH;                                     // LOW is left.
    speedCNCZ = 2000;
    if(controlState==HIGH)
    {
      //DEBUG_PORT.println("We reached point 2");
      intervalCNCZOne = 1000000/speedCNCZ;                      // interval is smaller for faster speed.
      CNC_TIMER_Z();
    }
    else
    {
      digitalWrite(31,LOW);                                  // Step z axis.
    }     
  }             // if(ZZeroing == HIGH)
  if(ZZeroingStepsTotal <= ZZeroingStep)                     // Tells us that setup of Z axis is complete.
  {
    setupStateZ = HIGH;
    //ZZeroingStep=0;
  }
////////////////////////////////////////////////////////////////////////////////////////
}               // CNC SETUP Z

void CNC_TIMER_Z()
{
    unsigned long currentMicros = micros();
    intervalCNCZOne = 1000000/speedCNCZ;
    if ((currentMicros - prevMicrosCNCZOne) >= intervalCNCZOne)
    {
      ZZeroingStep++;
      LSUZ = Fast_digitalRead(41);                          // Limit switch Z up
      LSDZ = Fast_digitalRead(45);                          // Limit switch Z down
      if (LSDZ==HIGH)                                       // Limit switch Z down activated.
      {
        dirStateCNCZ=LOW;                                   // LOW is up.
      }
      if (LSUZ==HIGH)                                       // Limit switch Z up activated.
      {
        dirStateCNCZ=HIGH;                                  // LOW is up.
      }
      if (stepStateCNCZ == LOW)
      {
        stepStateCNCZ = HIGH;
      }
      else
      {
        stepStateCNCZ = LOW;
      }
      //DEBUG_PORT.print("ZZeroingState =    ");DEBUG_PORT.println(ZZeroingState);
      digitalWrite(27,dirStateCNCZ);
      digitalWrite(31,stepStateCNCZ);
      prevMicrosCNCZOne = currentMicros;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void CNC_TIMER_Y()
{
    unsigned long currentMicros = micros();
    intervalCNCYOne = 1000000/speedCNCY;                       // interval is smaller for faster speed.
    if ((currentMicros - prevMicrosCNCYOne) >= intervalCNCYOne)
    {
      YZeroingStep++;
      LSLY = Fast_digitalRead(43);                          // Limit switch Y left
      if (stepStateCNCY == LOW)
      {
        stepStateCNCY = HIGH;
      }
      else
      {
        stepStateCNCY = LOW;
      }
      //DEBUG_PORT.print("YZeroingState =    ");DEBUG_PORT.println(YZeroingState);
      digitalWrite(51,dirStateCNCY);
      digitalWrite(33,stepStateCNCY);
      prevMicrosCNCYOne = currentMicros;
    }
}
void CNC_TIMER_X()
{
    unsigned long currentMicros = micros();
    intervalCNCXOne = 1000000/speedCNCX; 
    if ((currentMicros - prevMicrosCNCXOne) >= intervalCNCXOne)
    {
      XZeroingStep++;
      LSFXRHS = Fast_digitalRead(49);                          // Limit switch X RHS forwards
      LSFXLHS = Fast_digitalRead(47);                          // Limit switch X LHS forwards
      if (stepStateCNCX == LOW)
      {
        stepStateCNCX = HIGH;
      }
      else
      {
        stepStateCNCX = LOW;
      }
      //DEBUG_PORT.print("XZeroingState =    ");DEBUG_PORT.println(XZeroingState);
      digitalWrite(53,dirStateCNCX);
//////////////////////////////////////////////////////////////////////////////
// Aligns left and right X axis saddles together:

      if ((LSFXLHS==LOW)||(bothXAxisLimitSwitches==true))    // HIGH means switch activated.
      {
        digitalWrite(13,stepStateCNCX);                      // LHS
      }
      if ((LSFXRHS==LOW)||(bothXAxisLimitSwitches==true))    // HIGH means switch activated.
      {
        digitalWrite(35,stepStateCNCX);                      // RHS
      }
      if ((LSFXLHS==HIGH)&&(LSFXRHS==HIGH))
      {
        bothXAxisLimitSwitches = true;
      }

///////////////////////////////////////////////////////////////////////////////
      prevMicrosCNCXOne = currentMicros;
    }
}
void CNC_TIMER_R()
{
    unsigned long currentMicros = micros();
    intervalCNCROne = 1000000/speedCNCR;                      // interval is smaller for faster speed.
    if ((currentMicros - prevMicrosCNCROne) >= intervalCNCROne)
    {
      if (stepStateCNCR == LOW)
      {
       stepStateCNCR = HIGH;
      }
      else
      {
        stepStateCNCR = LOW;
      }
      //digitalWrite(,dirStateCNCR);
      digitalWrite(29,stepStateCNCR);
      prevMicrosCNCROne = currentMicros;
    }
}
void opZero()
{
  if(operationZero==false)                                      // current operation not yet performed.
  {
    operationZero = true;
    XZeroingStep=0;
    YZeroingStep=0;
    ZZeroingStep=0;
  }
}
void opOne()
{
  speedCNCX = 2000;                                            // 1 = one Hz.
  if(operationOne==false)                                      // current operation not yet performed.
  {
    operationNumber = "operation one";
    XZeroingStepsTotal = 17881;                                // one grid
    dirStateCNCX = LOW;                                        // LOW is forwards.
    if(XZeroingStepsTotal > XZeroingStep)
    {
      CNC_TIMER_X();
    }   
  }             // if(operationOne == false)
  if((XZeroingStepsTotal <= XZeroingStep)&&(operationOne==false))                    // Tells us that operationOne is complete.
  {
    operationOne=true;
    XZeroingStep=0;
    YZeroingStep=0;
    ZZeroingStep=0;
  }
}
void opTwo()
{
  if(operationOne==true)                                         // Operation one has been completed.
  {
    if(finalCurrentSensorValueRAxis<530)   // This value (530) is higher than in next operation to get claw down faster.
    {
      dirStateCNCZ = HIGH;                                     // HIGH is down.
      speedCNCZ = 1000;                                         // 1 = one Hz.
      CNC_TIMER_Z();
    }
    else
    {
      dirStateCNCZ = LOW;                                      // LOW is up.
      speedCNCZ = 1000;                                        // 1 = one Hz.
      CNC_TIMER_Z();
    } 
    
    speedCNCY = 300;                                             // 1 = one Hz. Allow time for claw to lower.
    CNC_TIMER_R();                                               // R axis motor claw starts turning.
    if(operationTwo==false)
    {
      operationNumber = "operation two";
      YZeroingStepsTotal = 26822;                                // One and a half grids.
      dirStateCNCY = LOW;                                        // LOW is left.
      if(YZeroingStepsTotal > YZeroingStep)
      {
        CNC_TIMER_Y();
      }   
    }     // if(operationTwo == false)
    if((YZeroingStepsTotal <= YZeroingStep)&&(operationTwo==false))                    // Tells us that operationOne is complete.
    {
      operationTwo=true;
      YZeroingStep=0;
    }
  }      //if(operationOne==true)
}
/////////////////////////////////////////////////////////////////////////////////////////
void move2ColumnsForward()
{
  speedTimerAsyncRightA = 657;                                                                 // Move slowly forwards.
  speedTimerAsyncLeftA = speedTimerAsyncRightA;
  CNC_TIMER_R();
  if (move2ColumnsForwards == true)                                                            //This is made 'true' in op 15.
  {
    operationNumber = "move two columns forwards";
    overideSteering==false;                                                                    // Enable auto steering.
    if (pixyBarData < 34)                                                                      // This is where barcode reached is reset to false.
    {
      barcodeReached = false;                                                                  // Move slowly forwards.      
    }
    if ((navState==HIGH)&&(pixyBarData>=34)&&(pixyBarcode==3)&&(barcodeReached == false))      // Need to allow the machine to move away from a previous barcode so that it is no longer visible. State change?
    {
    //DEBUG_PORT.print("pixyBarData (line 1739) = ");DEBUG_PORT.println(pixyBarData);
      speedTimerAsyncRightA = 665;                                                             // Stops the machine at the new barcode position, if a barcode '3' is seen and pixy navigation (navState) is enabled.
      speedTimerAsyncLeftA = speedTimerAsyncRightA;
      barcodeReached = true;                                                                   // where is barcodeReached made false?
      move2ColumnsForwards = false;                                                            // move two columns forwards has been completed.
      overideSteering==true;                                                                   // Disable auto steering.
    }                                                                                          // Barcode needs to be somewhere near the 2 column mark for the above to work.
    moveRightMotor();
    moveLeftMotor();
  }    // if (move2ColumnsForwards == true) //Forwards.
}      // void move2ColumnsForwards()
//////////////////////////////////////////////////////////////////////////////////////////
// opThree is the start of the main weeding routine loop.
void opThree()
{
  if((operationTwo==true)&&(move2ColumnsForwards == true)&&(controlState==HIGH))      // move2ColumnsForwards is made true in op21.
  {
    //DEBUG_PORT.print("line 1765 reached. ");DEBUG_PORT.println(moveStepsForwards);
    //DEBUG_PORT.print("move2ColumnsForwards = ");DEBUG_PORT.println(moveStepsForwards);
    move2ColumnsForward();                                               // move forwards one column only after first CNC loop.
  }
  if((operationTwo==true)&&(move2ColumnsForwards == false))       // Operation two and move forwards has been completed.
  {
    //DEBUG_PORT.print("operationTwo = ");DEBUG_PORT.println(operationTwo);
    weedingBegin = true;
    CNC_TIMER_R();                                               // Enable claw rotation.
    speedCNCX = 1000;                                             // 1 = one Hz.
    if(operationThree==false)                                     // current operation not yet performed.
    {
      operationNumber = "operation three";
      XZeroingStepsTotal = 35763;                                // 2 grids.
      dirStateCNCX = HIGH;                                        // LOW is forwards.
      if(XZeroingStepsTotal > XZeroingStep)
      {
        CNC_TIMER_X();
      }   
    }     // if(operationThree == false)
    if((XZeroingStepsTotal <= XZeroingStep)&&(operationThree==false))                    // Tells us that operationThree is complete.
    {
      operationThree=true;
      XZeroingStep=0;
    }
  }      //if(operationOne==true)
}
void opFour()
{
  if(operationThree==true)                                       // Previous operation has been completed.
  {
    CNC_TIMER_R();                                               // Enable claw rotation.
    speedCNCY = 1000;                                             // 1 = one Hz.
    if(operationFour==false)                                     // current operation not yet performed.
    {
      operationNumber = "operation four";
      YZeroingStepsTotal = 53645;                                // 3 grid
      dirStateCNCY = HIGH;                                        // LOW is left.
      if(YZeroingStepsTotal > YZeroingStep)
      {
        CNC_TIMER_Y();
      }   
    }     // if(operationTwo == false)
    if((YZeroingStepsTotal <= YZeroingStep)&&(operationFour==false))                    // Tells us that current operation is complete.
    {
      operationFour=true;
      YZeroingStep=0;
    }
  }      //if(operationOne==true)
}
void opFive()
{
  if(operationFour==true)                                         // Previous operation has been completed.
  {
    CNC_TIMER_R();                                               // Enable claw rotation.
    speedCNCX = 1200;                                             // 1 = one Hz.
    if(operationFive==false)                                     // current operation not yet performed.
    {
      operationNumber = "operation five";
      XZeroingStepsTotal = 35763;                                // 2 grids.
      dirStateCNCX = LOW;                                        // LOW is forwards.
      if(XZeroingStepsTotal > XZeroingStep)
      {
        CNC_TIMER_X();
      }   
    }
    if((XZeroingStepsTotal <= XZeroingStep)&&(operationFive==false))                    // Tells us that current operation is complete.
    {
      operationFive=true;
      XZeroingStep=0;
    }
  }
}
void opSix()
{
  if(operationFive==true)                                       // Previous operation has been completed.
  {
    CNC_TIMER_R();                                               // Enable claw rotation.
    speedCNCY = 1000;                                             // 1 = one Hz.
    if(operationSix==false)                                     // current operation not yet performed.
    {
      operationNumber = "operation six";
      YZeroingStepsTotal = 53645;                                // 3 grids
      dirStateCNCY = LOW;                                        // LOW is left.
      if(YZeroingStepsTotal > YZeroingStep)
      {
        CNC_TIMER_Y();
      }   
    }     // if(operationTwo == false)
    if((YZeroingStepsTotal <= YZeroingStep)&&(operationSix==false))                    // Tells us that current operation is complete.
    {
      operationSix=true;
      YZeroingStep=0;
    }
  }      //if(operationOne==true)
}
void opSeven()
{
  if(operationSix==true)                                       // Previous operation has been completed.
  {
    CNC_TIMER_R();                                               // Enable claw rotation.
    speedCNCX = 1000;                                             // 1 = one Hz.
    if(operationSeven==false)                                     // current operation not yet performed.
    {
      operationNumber = "operation seven";
      XZeroingStepsTotal = 17881;                                // 1 grid.
      dirStateCNCX = HIGH;                                        // LOW is forwards.
      if(XZeroingStepsTotal > XZeroingStep)
      {
        CNC_TIMER_X();
      }   
    }
    if((XZeroingStepsTotal <= XZeroingStep)&&(operationSeven==false))                    // Tells us that current operation is complete.
    {
      operationSeven=true;
      XZeroingStep=0;
    }
  }      //if(operationOne==true)
}
void opEight()
{
  if(operationSeven==true)                                       // Previous operation has been completed.
  {
    CNC_TIMER_R();                                               // Enable claw rotation.
    speedCNCY = 1000;                                             // 1 = one Hz.
    if(operationEight==false)                                     // current operation not yet performed.
    {
      operationNumber = "operation eight";
      YZeroingStepsTotal = 53645;                                // 3 grids.
      dirStateCNCY = HIGH;                                        // LOW is left.
      if(YZeroingStepsTotal > YZeroingStep)
      {
        CNC_TIMER_Y();
      }   
    }
    if((YZeroingStepsTotal <= YZeroingStep)&&(operationEight==false))                    // Tells us that current operation is complete.
    {
      operationEight=true;
      YZeroingStep=0;
    }
  }
}
void opNine()
{
  if(operationEight==true)                                         // Previous operation has been completed.
  {
    CNC_TIMER_R();                                               // Enable claw rotation.
    speedCNCX = 1000;                                             // 1 = one Hz.
    if(operationNine==false)                                     // current operation not yet performed.
    {
      operationNumber = "operation nine";
      XZeroingStepsTotal = 17881;                                // 1 grids.
      dirStateCNCX = HIGH;                                        // LOW is forwards.
      if(XZeroingStepsTotal > XZeroingStep)
      {
        CNC_TIMER_X();
      }   
    }
    if((XZeroingStepsTotal <= XZeroingStep)&&(operationNine==false))                    // Tells us that current operation is complete.
    {
      operationNine=true;
      XZeroingStep=0;
    }
  }
}
void opTen()
{
  if(operationNine==true)                                       // Previous operation has been completed.
  {
    CNC_TIMER_R();                                               // Enable claw rotation.
    speedCNCY = 1000;                                             // 1 = one Hz.
    if(operationTen==false)                                     // current operation not yet performed.
    {
      operationNumber = "operation ten";
      YZeroingStepsTotal = 17881;                                // 1 grids.
      dirStateCNCY = LOW;                                        // LOW is left.
      if(YZeroingStepsTotal > YZeroingStep)
      {
        CNC_TIMER_Y();
      }   
    }
    if((YZeroingStepsTotal <= YZeroingStep)&&(operationTen==false))                    // Tells us that current operation is complete.
    {
      operationTen=true;
      YZeroingStep=0;
    }
  }
}
void opEleven()
{
  if(operationTen==true)                                       // Previous operation has been completed.
  {
    CNC_TIMER_R();                                               // Enable claw rotation.
    speedCNCX = 1000;                                             // 1 = one Hz.
    if(operationEleven==false)                                     // current operation not yet performed.
    {
      operationNumber = "operation eleven";
      XZeroingStepsTotal = 35763;                                  // two grids.
      dirStateCNCX = LOW;                                        // LOW is forwards.
      if(XZeroingStepsTotal > XZeroingStep)
      {
        CNC_TIMER_X();
      }   
    }
    if((XZeroingStepsTotal <= XZeroingStep)&&(operationEleven==false))                    // Tells us that current operation is complete.
    {
      operationEleven=true;
      XZeroingStep=0;
    }
  }
}
void opTwelve()
{
  if(operationEleven==true)                                       // Previous operation has been completed.
  {
    CNC_TIMER_R();                                               // Enable claw rotation.
    speedCNCY = 1000;                                             // 1 = one Hz.
    if(operationTwelve==false)                                     // current operation not yet performed.
    {
      operationNumber = "operation twelve";
      YZeroingStepsTotal = 17881;                                  // 1 grid.
      dirStateCNCY = LOW;                                        // LOW is left.
      if(YZeroingStepsTotal > YZeroingStep)
      {
        CNC_TIMER_Y();
      }   
    }
    if((YZeroingStepsTotal <= YZeroingStep)&&(operationTwelve==false))                    // Tells us that current operation is complete.
    {
      operationTwelve=true;
      YZeroingStep=0;
    }
  }
}
void opThirteen()
{
  if(operationTwelve==true)                                         // Previous operation has been completed.
  {
    CNC_TIMER_R();                                               // Enable claw rotation.
    speedCNCX = 1000;                                             // 1 = one Hz.
    if(operationThirteen==false)                                     // current operation not yet performed.
    {
      operationNumber = "operation thirteen";
      XZeroingStepsTotal = 35763;                                // 2 grids.
      dirStateCNCX = HIGH;                                        // LOW is forwards.
      if(XZeroingStepsTotal > XZeroingStep)
      {
        CNC_TIMER_X();
      }   
    }
    if((XZeroingStepsTotal <= XZeroingStep)&&(operationThirteen==false))                    // Tells us that current operation is complete.
    {
      operationThirteen=true;
      XZeroingStep=0;
    }
  }
}
void opFourteen()
{
  if(operationThirteen==true)                                       // Previous operation has been completed.
  {
    CNC_TIMER_R();                                               // Enable claw rotation.
    speedCNCY = 1000;                                             // 1 = one Hz.
    if(operationFourteen==false)                                     // current operation not yet performed.
    {
      operationNumber = "operation fourteen";
      YZeroingStepsTotal = 17881;                                  // 1 grid.
      dirStateCNCY = LOW;                                        // LOW is left.
      if(YZeroingStepsTotal > YZeroingStep)
      {
        CNC_TIMER_Y();
      }   
    }
    if((YZeroingStepsTotal <= YZeroingStep)&&(operationFourteen==false))                    // Tells us that current operation is complete.
    {
      operationFourteen=true;
      YZeroingStep=0;
    }
  }
}
void opFifteen()
{
  if(operationFourteen==true)                                         // Previous operation has been completed.
  {
    CNC_TIMER_R();                                               // Enable claw rotation.
    speedCNCX = 1000;                                             // 1 = one Hz.
    if(operationFifteen==false)                                     // current operation not yet performed.
    {
      operationNumber = "operation fifteen";
      XZeroingStepsTotal = 35763;                                // 2 grids.
      dirStateCNCX = LOW;                                        // LOW is forwards.
      if(XZeroingStepsTotal > XZeroingStep)
      {
        CNC_TIMER_X();
      }   
    }
    if((XZeroingStepsTotal <= XZeroingStep)&&(operationFifteen==false))                    // Tells us that current operation is complete.
    {
      move2ColumnsForwards = true;                                 // prepare for machine moving one column forwards.
      operationFifteen=true;
      XZeroingStep=0;
      operationState=true;                                    // All operations have finished.
      DEBUG_PORT.print("move2ColumnsForwards = ");DEBUG_PORT.println(move2ColumnsForwards);
    }
  }
}
