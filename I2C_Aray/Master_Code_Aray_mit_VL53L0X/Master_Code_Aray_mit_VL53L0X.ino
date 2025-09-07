#include <Wire.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>
#include <roboter_master.h>

int targetPulses_gesamt = 0; //Zielgeschwindigkeit von Roboter
int targetPulses_Motor1 = 30; //Zielgeschwindigkeit von Motor1
int targetPulses_Motor2 = 30; //Zielgeschwindigkeit von Motor2
int targetPulses_Motor3 = 30; //Zielgeschwindigkeit von Motor3
int targetPulses_Motor4 = 30; //Zielgeschwindigkeit von Motor4

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

  //checkDistanceAndSetPulses_on_off(10); //10 P/0,1s bei on status
  
  sendtoSlave(0x08, -targetPulses_Motor1, 200);
  sendtoSlave(0x09, targetPulses_Motor2, 200);
  sendtoSlave(0x0A, -targetPulses_Motor3, 200);
  sendtoSlave(0x0B, targetPulses_Motor4, 200);

  delay(1000);
  
  sendtoSlave(0x08, targetPulses_Motor1, 200);
  sendtoSlave(0x09, -targetPulses_Motor2, 200);
  sendtoSlave(0x0A, targetPulses_Motor3, 200);
  sendtoSlave(0x0B, -targetPulses_Motor4, 200);

  delay(1000);

  sendtoSlave(0x08, -targetPulses_Motor1); //max_pulse werden nicht berücksichtig und in Ruhe gelassen
  sendtoSlave(0x09, targetPulses_Motor2);
  sendtoSlave(0x0A, -targetPulses_Motor3);
  sendtoSlave(0x0B, targetPulses_Motor4);

  delay(1000);
  
  sendtoSlave(0x08, targetPulses_Motor1);
  sendtoSlave(0x09, -targetPulses_Motor2);
  sendtoSlave(0x0A, targetPulses_Motor3);
  sendtoSlave(0x0B, -targetPulses_Motor4);

  delay(1000);
}