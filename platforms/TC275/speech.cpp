#include "speech.h"
#include "TC275.h"

int emicCount = 0;
String cmd = "S";
int emicTimerInterval = 40000;
String text4 ="z";
String text1 ="z";
String text2 ="z";
String text3 ="z";
String text6 ="z";
String text7 ="z";

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
  emicCount++;
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
  //text4 +=  pixyBarData + " .. stop .. ";
  if (controlState==LOW)                                        // Manual
  {
    emicTimerInterval = 40000;
    text4 = "Where are the god damn weeds .. has any body seen any weeds any where?";
  }
  else
  {
    emicTimerInterval = 10000;
    text4 = "We are currently doing operation " + operationNumber +  " .. of fifteen steps .. ";
  }
  if (emicCount > 3)
  {
    text4 = "I need to kill some weeds .. I need to kill something .. I need to kill them .. now";
  }
  if (emicCount > 6)
  {
    text4 = "Get off my land .. I repeat .. Get .. off .. my .. land .. and dont come back";
    emicCount = 0;
  }
  if(((setupStateX==LOW)||(setupStateY==LOW)||(setupStateZ==LOW))&&(controlState==HIGH))
  {
    text4 = "We are currently setting up the CNC machinery .. weeding will commence shortly";
  }
  if (move2ColumnsForwards == true) 
  {
    emicTimerInterval = 10000;
    text4 = "Everybody get out of the way .. We are moving forwards to the next grid of weeds" ; 
  }
  text6 = "[:phone arpa speak on][:rate 190][:n0][ GAA<400,12>DD<200,15>B<200,10>LLEH<200,19>EH<500,22>S<200,18>AH<100,18>MEH<200,16>K<100,13>AH<200,12>][:n0]";
  //DEBUG_PORT.println(text4);
  emicPort.print(cmd + text4 + "\n");
  ; 
}
