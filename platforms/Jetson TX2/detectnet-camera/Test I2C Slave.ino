#include <Wire.h>
long y[4][4];
int a;
int b;
int c;
int d;
long x =0;
int i;
int j;
int numberOfBoxes;
int xMax;

void setup() 
{
  Wire.begin(0x70);                // join i2c bus with address
  Wire.onReceive(receiveEvent); // register event
  //Wire.begin(0x50);                // join i2c bus with address
  //Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
}

void loop() 
{
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) 
{
  //delay(50);
  int x = Wire.read();    // receive byte as an integer
  //Serial.print("  Integer:  ");Serial.println(x);         // print the integer


  if(x>199)
  {
    numberOfBoxes = x-200;
  }
  if((x>139)&&(x<200))
  { 
    j=x-140;Serial.print("Number of boxes: ");Serial.print(numberOfBoxes);Serial.print(", Box number: ");Serial.println(j); 
  }
  if(x==120){ i =-1; }
  if(i==0){ y[0][0] = x*1000; }
  if(i==1){ y[0][1] = x*100; }
  if(i==2){ y[0][2] = x*10; }
  if(i==3){ y[0][3] = x;}
  a= y[0][0]+y[0][1]+y[0][2]+y[0][3];

  if(x==121){ i = 4;  Serial.print("  corner a:  ");Serial.println(a);}
  if(i==5){ y[1][0] = x*1000; }
  if(i==6){ y[1][1] = x*100; }
  if(i==7){ y[1][2] = x*10; }
  if(i==8){ y[1][3] = x; }
  b = y[1][0]+y[1][1]+y[1][2]+y[1][3];

  if(x==122){ i = 9;  Serial.print("  corner b:  ");Serial.println(b);}
  if(i==10){ y[2][0] = x*1000; }
  if(i==11){ y[2][1] = x*100; }
  if(i==12){ y[2][2] = x*10; }
  if(i==13){ y[2][3] = x;   }
  c= y[2][0]+y[2][1]+y[2][2]+y[2][3];

  if(x==123){ i = 14;  Serial.print("  corner c:  ");Serial.println(c);}
  if(i==15){ y[3][0] = x*1000; }
  if(i==16){ y[3][1] = x*100; }
  if(i==17){ y[3][2] = x*10; }
  if(i==18){ y[3][3] = x;  }
  d= y[3][0]+y[3][1]+y[3][2]+y[3][3];
  if(i==18){  Serial.print("  corner d:  ");Serial.println(d);Serial.println("");}
  
  i++;
}
