int incomingByte = 0;   // for incoming serial data
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

int numClasses;
int confidence;
int imageClass;

void setup() 
{
        Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
        Serial.println("TEST ");
}

void loop()
{
  if (Serial.available() > 0) 
  {
    x = Serial.read();  // read the incoming byte:
/////////////////////////////////////////////////////////////////////////////////
    if(x>199)
    {
      numberOfBoxes = x-200;
    }
    if((x>139)&&(x<200))
    { 
      j=x-140;Serial.print("Number of boxes: ");Serial.print(numberOfBoxes);Serial.print(", Box number: ");Serial.println(j); 
    }
////////////////////////////////// Confidence:
    if ((x>=10) && (x<=19))
    {
      for(int n=10; n<19; n++)
      {
        if(x == n)
        {
          confidence = (x-10)*10;
          break;
        }
      }
    }
////////////////////////////////// Number of classes::
    if ((x>=100) && (x<=109))
    {
      for(int n=100; n<109; n++)
      {
        if(x == n)
        {
          numClasses = x-100;
          break;
        }
      }
    }
////////////////////////////////// Image class:
    if ((x>=20) && (x<=99))
    {
      for(int n=20; n<99; n++)
      {
        if(x == n)
        {
          imageClass = n-20;
          Serial.print("Number of classes: ");Serial.print(numClasses);
          Serial.print(",   Image class: ");Serial.print(imageClass);
          Serial.print(",   Confidence: ");Serial.println(confidence);
          break;
        }
      }
    }
/////////////////////////////////
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
}
