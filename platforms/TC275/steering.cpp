#include "steering.h"
#include "TC275.h"

int rightHandLock =LOW;
int leftHandLock=LOW;
int clockW=LOW;
int antiClockW=LOW;
int actuallySteering =LOW;
int difference=0;
int previousFinalSteeringValue=15300;

long finalSteeringValue =0;
int forwards = LOW;
int backwards =LOW;

float resultOne = 1.0000;
float resultTwo = 0;
float resultThree = 0;
float resultFour = 0;

unsigned long previousMicrosOne = 0;
unsigned long previousMicrosTwo = 0;
unsigned long previousMicrosThree = 0;
unsigned long previousMicrosFour = 0;

int ledStateOne = LOW;   
int ledStateTwo = LOW; 
int ledStateThree = LOW;
int ledStateFour = LOW;
////////////////////////////////////////////////////////////////

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
} // turning when stationary
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
void differential()
{
  unsigned long currentMicrosOne = micros();
  unsigned long currentMicrosTwo = micros(); 
  unsigned long currentMicrosThree = micros();
  unsigned long currentMicrosFour = micros();
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
}    // void differential
