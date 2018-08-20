#include "drive.h"
#include "TC275.h"

int finalDriveValue =0;

unsigned long previousMicrosTimerAsyncLeftA = 0; 
unsigned long previousMicrosTimerAsyncLeftB = 0;
unsigned long previousMicrosTimerAsyncRightA = 0; 
unsigned long previousMicrosTimerAsyncRightB = 0;

unsigned long intervalTimerAsyncLeftA = 0;
unsigned long intervalTimerAsyncLeftB = 0;
unsigned long intervalTimerAsyncRightA = 0;
unsigned long intervalTimerAsyncRightB = 0;

int asyncStateRight = HIGH;
int asyncStateLeft = HIGH;

float intervalOne = 1000; // Right hand wheel turn speed. Bigger is faster.
float intervalTwo = 1000; // Left hand wheel turn speed.
float intervalThree = 1000; // Right hand wheel drive speed. Is this left wheel?
float intervalFour = 1000; // Left hand wheel drive speed.

float runningmaxCurrentValueFive = 1;
float runningmaxCurrentValueSix = 1;

long ferrets=0;
long wheelsPosition=0;
int stationary=LOW;

float headingDegrees = 0;
float bearingDegrees = 0;
long distanceMM =0;
long latitudeUblox =0;
long longitudeUblox =0;

////////////////////////////////////////////////////////////////

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
