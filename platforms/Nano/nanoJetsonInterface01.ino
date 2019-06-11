#include <Wire.h>
long y[4][4];
int a;
int b;
int c;
int d;
int i;
int j;
int numberOfBoxes;
int xMax;
int dataAffirmative = 0;
int x;
// Buzzer on pin 10
// LED pin 6
// LED pin 7

void setup() 
{
  tone(10,1000,1000);
  Wire.begin(0x70);                // join i2c bus with address
  Wire.onReceive(receiveEvent); // register event
  //Wire.begin(0x50);                // join i2c bus with address
  //Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
  Serial.println("START");
}

void loop() 
{
  //Serial.print("  Integer:  ");Serial.println(x);         // print the integer
  delay(100);
  if (dataAffirmative == 1)
  {
    if (x==1)
    {
      Serial.print("  Turn LEFT, drive FORWARDS:  ");Serial.println(x);
      tone(10,1000,50);
    }
    if (x==2)
    {
      Serial.print("  Turn RIGHT, drive FORWARDS:  ");Serial.println(x);
      tone(10,950,50);
    }
    if (x==3)
    {
      Serial.print("  STOP:  ");Serial.println(x);
      tone(10,900,100);
    }
    if (x==4)
    {
      Serial.print("  Turn LEFT, drive BACKWARDS:  ");Serial.println(x);
      tone(10,850,50);
    }
    if (x==5)
    {
      Serial.print("  Turn RIGHT, drive BACKWARDS:  ");Serial.println(x);
      tone(10,800,50);
    }
    dataAffirmative = 0;
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) 
{
  dataAffirmative = 1;
  //delay(50);
  x = Wire.read();    // receive byte as an integer
  //Serial.print("  Integer:  ");Serial.println(x);         // print the integer
}
