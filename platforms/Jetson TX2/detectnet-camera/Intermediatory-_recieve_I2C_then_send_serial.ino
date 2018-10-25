#include <Wire.h>

void setup() 
{
  Wire.begin(0x70);                // join i2c bus with address
  Wire.onReceive(receiveEvent);    // register event
  Serial.begin(115200);            // start serial for output
}

void loop() 
{
  delay(100); // Must have delay here.
}

void receiveEvent(int howMany) 
{
  int x = Wire.read();    // receive byte as an integer
  Serial.write(x);        // send a byte
}
