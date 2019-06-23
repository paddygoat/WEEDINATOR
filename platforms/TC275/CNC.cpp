/*
Upon flicking switch on left to 'on', control state becomes 'true' and CNC routine is started.
Auto steering is disengaged by setting 'preventSteering' to 'true' to stop constant 'hunting' of the steering.
CNC goes through an initial setup to find it's position on x,y and z by means of triggering limits switches.
CNC ten goes through 15 stages of weeding through predetermined distances on x and y axes.
Once finished, 'move2ColumnsForwards becomes 'true' and 'preventSteering' becomes 'false'.
Steering is now determined via 'makeTurnValue' by the position of the right hand switch, currently GPS by 'false' and Jetson by 'true'.
When 'barcodeReached' becomes 'true', the CNC routine is started again, but from number 3 and steering is prevented once more.
'move2ColumnsForwards' and 'barcodeReached' are set back to 'false'.
*/
#include "CNC.h"
#include "encoder.h"
#include "TC275.h"
////////////////////////////////////////////////////////////////
float intervalCNCXOne = 0;
float intervalCNCYOne = 0;
float intervalCNCZOne = 0;
float intervalCNCROne = 0;
int speedCNCX = 1000;
int speedCNCY = 1000;
int speedCNCZ = 1000;
int speedCNCR = 1000;
unsigned long prevMicrosCNCXOne = 0; 
unsigned long prevMicrosCNCYOne = 0; 
unsigned long prevMicrosCNCZOne = 0; 
unsigned long prevMicrosCNCROne = 0; 
int XZeroingState = LOW;
int YZeroingState = LOW;
int ZZeroingState = LOW;
long XZeroingStepsTotal = 1000;
long YZeroingStepsTotal = 1000;
long ZZeroingStepsTotal = 1000;
long XZeroingStep = 0;
long YZeroingStep = 0;
long ZZeroingStep = 0;
uint8_t LSFXRHS = LOW;          // RHS Switch open x axis.
uint8_t LSFXLHS = LOW;          // LHS Switch open x axis.
uint8_t LSBX = HIGH;
uint8_t LSRY = HIGH;          // Switch open y axis. LSRY = limit switch right y.
uint8_t LSLY = HIGH;          // D43
uint8_t LSUZ = HIGH;          // D41
uint8_t LSDZ = HIGH;         // limit switch Z down.
int dirStateCNCX = LOW; 
int dirStateCNCY = LOW; 
int dirStateCNCZ = LOW;
int dirStateCNCR = LOW; 

int stepStateCNCX = LOW;
int stepStateCNCY = LOW;
int stepStateCNCZ = LOW;
int stepStateCNCR = LOW;

uint8_t setupStateX = LOW;           // Change to HIGH to skip CNC setups.
uint8_t setupStateY = LOW;
uint8_t setupStateZ = LOW;

String operationNumber = "NULL";

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

int controlState = LOW;  // Manual or autonomous(HGH)
int finalCurrentSensorValueRAxis =0;

float speedTimerAsyncRightA = 0.001;
float speedTimerAsyncRightB = 0.001;
float speedTimerAsyncLeftA = 0.001;
float speedTimerAsyncLeftB = 0.001;

bool preventSteering = false;
bool barcodeReached = false;

int pixyPanData =0;
int pixyBarData =0;
int pixyBarcode =0;

int navState=LOW;        // Use Pixy or GPS
int currentEncoderValue = 0;
int previousEncoderValue = 0;
////////////////////////////////////////////////////////////////

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
        digitalWrite(12,stepStateCNCX);                      // LHS
      }
      if ((LSFXRHS==LOW)||(bothXAxisLimitSwitches==true))    // HIGH means switch activated.
      {
        digitalWrite(35,stepStateCNCX);                      // RHS
      }
      if ((LSFXLHS==HIGH)&&(LSFXRHS==HIGH))
      {
        bothXAxisLimitSwitches = true;
      }
      //DEBUG_PORT.print("Both limit switches activated = ");DEBUG_PORT.println(bothXAxisLimitSwitches);
      //DEBUG_PORT.print("LSFXLHS = ");DEBUG_PORT.println(LSFXLHS);
      //DEBUG_PORT.print("LSFXRHS = ");DEBUG_PORT.println(LSFXRHS);
///////////////////////////////////////////////////////////////////////////////
      prevMicrosCNCXOne = currentMicros;
    }
} // CNC timer X
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
} // void CNC timer Y
////////////////////////////////////////////////////////////////
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
} // void CNC timer Z
/////////////////////////////////////////////////////////////
void CNC_TIMER_R()
{
  speedCNCR = 2000; 
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
}  // void CNC timer R
//////////////////////////////////////////////////////////////
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
    XZeroingStep=0;
    intervalCNCXOne = 1000000/speedCNCX;                       // interval is smaller for faster speed.
    CNC_TIMER_X();
  }
  else
  {
    digitalWrite(35,LOW);
    digitalWrite(12,LOW);
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
      digitalWrite(12,LOW);
    }     
  }             // if(XZeroing == HIGH)
  if(XZeroingStepsTotal <= XZeroingStep)                     // Tells us that setup of X axis is complete.
  {
    setupStateX = HIGH;
    //XZeroingStep=0;
  }
}               // CNC SETUP X
////////////////////////////////////////////////////////////////
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
}               // CNC SETUP Y
///////////////////////////////////////////////////////////////
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
}               // CNC SETUP Z
////////////////////////////////////////////////////////////////
void opZero()
{
  if(operationZero==false)                                      // current operation not yet performed.
  {
    //DEBUG_PORT.println("operationZero in progress.");
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
    //DEBUG_PORT.println("operationOne in progress.");
    operationNumber = "one";
    XZeroingStepsTotal = 17881;                                // one grid
    dirStateCNCX = LOW;                                        // LOW is forwards.
    if((XZeroingStep > 2000)||(XZeroingStep < 15000))          // Acceleration
    {
      speedCNCX = 3000;
    }
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
      //DEBUG_PORT.println("operationTwo in progress.");
      operationNumber = "two";
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
////////////////////////////////////////////////////////////////
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
    speedCNCX = 500;                                             // 1 = one Hz.
    if(operationThree==false)                                     // current operation not yet performed.
    {
      operationNumber = "three";
      XZeroingStepsTotal = 35763;                                // 2 grids.
      if((XZeroingStep > 2000)||(XZeroingStep < 30000))          // Acceleration
      {
      speedCNCX = 1500;
      }
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
      //DEBUG_PORT.println("operationFour in progress.");
      operationNumber = "four";
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
      operationNumber = "five";
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
      operationNumber = "six";
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
      operationNumber = "seven";
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
      operationNumber = "eight";
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
      operationNumber = "nine";
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
      operationNumber = "ten";
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
      operationNumber = "eleven";
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
      operationNumber = "twelve";
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
      operationNumber = "thirteen";
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
      operationNumber = "fourteen";
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
      operationNumber = "fifteen";
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
      currentEncoderValue = 0;
      previousEncoderValue = encoderCount;
      operationFifteen=true;
      XZeroingStep=0;
      operationState=true;                                    // All operations have finished.
      //DEBUG_PORT.print("move2ColumnsForwards = ");DEBUG_PORT.println(move2ColumnsForwards);
    }
  }
}
//////////////////////////////////////////////////////////////
void move2ColumnsForward()
{
  speedTimerAsyncRightA = 657;                                                                 // Move slowly forwards.
  speedTimerAsyncLeftA = speedTimerAsyncRightA;
  CNC_TIMER_R();
  if (move2ColumnsForwards == true)                                                            //This is made 'true' in op 15.
  {
    currentEncoderValue = encoderCount - previousEncoderValue;
    preventSteering = false;                                                                   // Enable auto steering.
    if (currentEncoderValue < 10000)                                                                      // This is where barcode reached is reset to false.
    {
      barcodeReached = false;                                                                  // Move slowly forwards.      
    }
    if ((navState==HIGH)&&(currentEncoderValue > 10000)&&(barcodeReached == false))      // Need to allow the machine to move away from a previous barcode so that it is no longer visible. State change?
    {
    //DEBUG_PORT.print("pixyBarData (line 1739) = ");DEBUG_PORT.println(pixyBarData);
      speedTimerAsyncRightA = 665;                                                             // Stops the machine at the new barcode position, if a barcode '3' is seen and pixy navigation (navState) is enabled.
      speedTimerAsyncLeftA = speedTimerAsyncRightA;
      barcodeReached = true;                                                                   // where is barcodeReached made false?
      move2ColumnsForwards = false;                                                            // move two columns forwards has been completed.
      preventSteering = true;                                                                  // Disable auto steering.
    }                                                                                          // Barcode needs to be somewhere near the 2 column mark for the above to work.
    moveRightMotor();
    moveLeftMotor();
  }    // if (move2ColumnsForwards == true) //Forwards.
}      // void move2ColumnsForwards()
////////////////////////////////////////////////////////////////
void weeding()
{
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
                                               // Step Hertz 500 is good.
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
}   // void weeding
void weedingBegins()
{
  if((controlState==LOW)&&(weedingBegin == false))
  {
    digitalWrite(35,LOW);  // Step x RHS axis
    digitalWrite(12,LOW);  // Step x LHS axis
    digitalWrite(33,LOW);  // Step y axis
    digitalWrite(31,LOW);  // Step z axis
    digitalWrite(29,LOW);  // Step r axis
  }
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
                                               // Step Hertz 500 is good.
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
}          // WeedingBegin
