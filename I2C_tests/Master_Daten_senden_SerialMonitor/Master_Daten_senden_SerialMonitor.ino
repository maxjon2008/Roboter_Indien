#include <Wire.h>
#include <SoftwareSerial.h>
//#include <roboter_master.h>

unsigned long lastTime = 0;
unsigned long lastTime_M = 0;

int targetPulses_gesamt = 0; //Zielgeschwindigkeit von Roboter
int targetPulses_Motor1 = 30; //Zielgeschwindigkeit von Motor1
int targetPulses_Motor2 = 30; //Zielgeschwindigkeit von Motor2
int targetPulses_Motor3 = 30; //Zielgeschwindigkeit von Motor3
int targetPulses_Motor4 = 30; //Zielgeschwindigkeit von Motor4


void setup() {
  Wire.begin();//0x05
  Serial.begin(9600);
  Serial.println("Tippe einen Zielwert für den Motor und drücke Enter:");

  }

void loop() {

  // Prüfen, ob der Serial-Monitor einen Wert gesendet hat
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n'); // Zeilenende
    int16_t target = inputString.toInt();             // in int16 umwandeln

    sendtoSlave(0x08, target);

    Serial.print("Gesendet an Slave: ");
    Serial.println(target);
  }
}

void sendtoSlave(uint8_t adresse, int16_t pps){
  int16_t daten[2];
  daten[0] = pps;
  daten[1] = 0;
  
  Wire.beginTransmission(adresse);
  Wire.write((byte*)daten, sizeof(daten));
  Wire.endTransmission();
  
  Serial.print("_Motor_");
  Serial.print(adresse);
  Serial.print(": ");
  Serial.print(daten[0]);
  Serial.print(", ");
  Serial.println(daten[1]);

}