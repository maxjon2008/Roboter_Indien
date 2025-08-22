#include <Wire.h>
#include <SoftwareSerial.h>
#include "VL53L0X.h"


int targetPulses_gesamt = 0; //Zielgeschwindigkeit von Roboter
int targetPulses_Motor1 = 0; //Zielgeschwindigkeit von Motor1
int targetPulses_Motor2 = 0; //Zielgeschwindigkeit von Motor2
int targetPulses_Motor3 = 0; //Zielgeschwindigkeit von Motor3
int targetPulses_Motor4 = 0; //Zielgeschwindigkeit von Motor4

// serial data output interface (bluetooth)
SoftwareSerial mySerial(5, 4);

// VL53L0X Sensor
VL53L0X TOF_sensor;

void setup() {
  Wire.begin();//0x05
  Serial.begin(9600);

  //Initialize the Serial Port for data output
  mySerial.begin(9600);

  // VL53L0X Sensor starten
  TOF_sensor.init();
  //TOF_sensor.setTimeout(500);

  //Long Range Mode aktivieren
  // TOF_sensor.setSignalRateLimit(0.1); // Standard 0.25 → kleinerer Wert = empfindlicher
  // TOF_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  // TOF_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  // Messzeit verlängern (Timing Budget)
  // TOF_sensor.setMeasurementTimingBudget(200000); // 200ms pro Messung

  Serial.println("VL53L0X Long Range Mode gestartet");
  mySerial.println("VL53L0X Long Range Mode gestartet");  
}

void loop() {

  uint16_t distance = TOF_sensor.readRangeSingleMillimeters();

  // Messbereich prüfen
  if (TOF_sensor.timeoutOccurred() || distance >= 8190 || distance == 0) {
      Serial.println("Außerhalb des Messbereichs");
      mySerial.println("Außerhalb des Messbereichs");
  } else {
      Serial.print("Abstand: ");
      Serial.print(distance);
      Serial.println(" mm");

      mySerial.print("Abstand: ");
      mySerial.print(distance);
      mySerial.println(" mm");
  }

  // Zielpulse setzen
  int16_t pulses = (distance >= 100) ? 20 : 0;  //if... else... Ersatz: Bedingung ? Wert_wenn_wahr : Wert_wenn_falsch
  targetPulses_Motor1 = pulses;
  targetPulses_Motor2 = pulses;
  targetPulses_Motor3 = pulses;
  targetPulses_Motor4 = pulses;



  sendtoSlave(0x01, targetPulses_Motor1);
  sendtoSlave(0x02, targetPulses_Motor2);
  sendtoSlave(0x03, targetPulses_Motor3);
  sendtoSlave(0x04, targetPulses_Motor4);

  delay(200);

}

void sendtoSlave(uint8_t adresse, int16_t wert, int16_t anzahl){
  byte buffer[2];
  buffer[0] = wert;
  buffer[1] = anzahl;
  Wire.beginTransmission(adresse);
  Wire.write(buffer, 2);
  Wire.endTransmission();
  
  mySerial.print("targetPulses_Motor_");
  mySerial.print(adresse);
  mySerial.print(": ");
  mySerial.println(wert);

}