// https://iotspace.dev/arduino-i2c-scanner-sketch-und-anleitung/
// Arduino I2C Scanner
// Baudrate 9600

#include <Wire.h>

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  delay(500);
  Serial.println("\n=== I2C Scanner ===");
}

void loop()
{
  byte error, address;
  int nDevices;
  Serial.println("Starte Scanvorgang");
  nDevices = 0;
  
  for (address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C Gerät gefunden - Adresse: 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unbekannter Fehler an Adresse: 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("Keine I2C Geräte gefunden\n");
  else
    Serial.println("Scanvorgang Abgeschlossen\n");

  delay(10000);
}