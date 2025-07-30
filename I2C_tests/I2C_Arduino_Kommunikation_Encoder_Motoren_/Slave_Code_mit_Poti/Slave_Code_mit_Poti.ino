//i2c Slave Code
#include <Wire.h>

int potiValue = 0;
int targetPulses = 0;
int poti_Pin = A1;

void setup()
{
  Wire.begin(0x05);
  Wire.onRequest(requestEvent);
}

void loop()
{
  delay(100);
}

void requestEvent()
{
  potiValue = analogRead(poti_Pin);
  targetPulses = map(potiValue,0,1023,0,130);
  Wire.write(targetPulses);// diese Daten sollen übermittelt werden
}