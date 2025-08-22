#include "roboter_master.h"

extern SoftwareSerial mySerial; // kommt aus dem Sketch
extern VL53L0X TOF_sensor; // kommt aus dem Sketch

extern int16_t targetPulses_Motor1;
extern int16_t targetPulses_Motor2;
extern int16_t targetPulses_Motor3;
extern int16_t targetPulses_Motor4;

void checkDistanceAndSetPulses_on_off(int Pulse_an) {
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
    int16_t pulses = (distance >= 300) ? Pulse_an : 0;  //if... else... Ersatz: Bedingung ? Wert_wenn_wahr : Wert_wenn_falsch
    targetPulses_Motor1 = pulses;
    targetPulses_Motor2 = pulses;
    targetPulses_Motor3 = pulses;
    targetPulses_Motor4 = pulses;
}

void sendtoSlave(uint8_t adresse, int16_t pps = 100, int16_t pulse = 99999999){
  int16_t daten[2];
  daten[0] = pps;
  daten[1] = pulse;
  
  Wire.beginTransmission(adresse);
  Wire.write((byte*)daten, sizeof(daten));
  Wire.endTransmission();
  
  mySerial.print("_Motor_");
  mySerial.print(adresse);
  mySerial.print(": ");
  mySerial.println(daten[0]);
  mySerial.print(", ");
  mySerial.print(daten[1]);

}