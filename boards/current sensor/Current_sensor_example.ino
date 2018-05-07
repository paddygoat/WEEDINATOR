int i=0;
int sensorValueOne;
int sensorValueTwo;
float maxValueOne;
float maxValueTwo;
float runningMaxValueOne = 0;
float runningMaxValueTwo = 0;
unsigned long currentMillis = 0;
unsigned long previousMillisOne = 0;
unsigned long previousMillisTwo = 0;
int intervalOne = 0;
const int intervalTwo = 1000;
int millisCalcOne =0;
int millisCalcTwo =0;
int ACFrequency = 50; // Hertz

void setup() 
{
  Serial.begin(115200);
  intervalOne = 1000 / ACFrequency; // Capture data over one complete AC cycle.
}

void loop() 
{
  currentMillis = millis();
  millisCalcOne = currentMillis - previousMillisOne;
  millisCalcTwo = currentMillis - previousMillisTwo;  
  i++;
  //Serial.print("  i:  ");Serial.println(i);
  sensorValueOne = analogRead(A0);
  sensorValueTwo = analogRead(A1);
  if(sensorValueOne > maxValueOne)
  {
    maxValueOne = sensorValueOne;
  }
  if(sensorValueTwo > maxValueTwo)
  {
    maxValueTwo = sensorValueTwo;
  }
//////////////////////////////////////////////////////////////////////////////
  if (millisCalcOne > intervalOne)
  {
    runningMaxValueOne = (maxValueOne - 523)/80;
    runningMaxValueTwo = (maxValueTwo - 523)/80;
    //Serial.print("  LHS amps max:  ");Serial.print(runningMaxValueOne,2);Serial.print("  RHS amps max:  ");Serial.println(runningMaxValueTwo,2);
    maxValueOne = 0;
    maxValueTwo = 0;
    i=0;
    previousMillisOne = currentMillis;
  }
  if (millisCalcTwo > intervalTwo)
  {
    //Serial.print("sensorValueOne:  ");Serial.print(sensorValueOne); 
    Serial.print("  LHS amps max:  ");Serial.print(runningMaxValueOne,2);Serial.print("  RHS amps max:  ");Serial.println(runningMaxValueTwo,2);
    previousMillisTwo = currentMillis;
  }
}
