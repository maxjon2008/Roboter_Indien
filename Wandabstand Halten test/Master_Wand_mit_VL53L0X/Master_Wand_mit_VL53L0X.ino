#include <Wire.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>
#include <roboter_master.h>

int targetPulses_gesamt = 0; //Zielgeschwindigkeit von Roboter
int targetPulses_Motor1 = 30; //Zielgeschwindigkeit von Motor1
int targetPulses_Motor2 = 30; //Zielgeschwindigkeit von Motor2
int targetPulses_Motor3 = 30; //Zielgeschwindigkeit von Motor3
int targetPulses_Motor4 = 30; //Zielgeschwindigkeit von Motor4

int distance_TOF = 0;
int solldistance = 200;
int baseSpeed = 0;

float Kp = 0.05;
int max_correction = 10;

unsigned long lastTime = 0;
int intervall = 200;

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
  // TOF_sensor.setSignalRateLimit(0.3); // Standard 0.25 → kleinerer Wert = empfindlicher
  // TOF_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  // TOF_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  // Messzeit verlängern (Timing Budget)
  TOF_sensor.setMeasurementTimingBudget(200000); // 200ms pro Messung

  Serial.println("VL53L0X Long Range Mode gestartet");
  mySerial.println("VL53L0X Long Range Mode gestartet");  


}

void loop() {

  if ((millis() - lastTime) >= intervall) //To stream without using additional timers
  {
    lastTime = millis();

    getdistance_TOF();
    //mySerial.println(distance_TOF);

    int fehler = distance_TOF - solldistance;

    fehler = constrain(Kp * fehler, -max_correction, max_correction);

    mySerial.print("Speed: ");
    mySerial.print(baseSpeed);
    mySerial.print("solldistance geändert: ");
    mySerial.print(solldistance);
    mySerial.print("Kp geändert: ");
    mySerial.print(Kp);
    mySerial.print("max_correction geändert: ");
    mySerial.println(max_correction);



    sendtoSlave(0x08, constrain(baseSpeed + fehler, -120, 120), 200);
    sendtoSlave(0x09, constrain(baseSpeed - fehler, -120, 120), 200);
    sendtoSlave(0x0A, constrain(baseSpeed + fehler, -120, 120), 200);
    sendtoSlave(0x0B, constrain(baseSpeed - fehler, -120, 120), 200);

  }

  if(mySerial.available()){
    String input = mySerial.readStringUntil('\n');
    input.trim();

      if (input.startsWith("S=")) {
        baseSpeed = input.substring(2).toInt();
        mySerial.print("Speed geändert: ");
        mySerial.println(baseSpeed);
      }

      else if (input.startsWith("D=")) {
        solldistance = input.substring(2).toInt();
        mySerial.print("solldistance geändert: ");
        mySerial.println(solldistance);
      }
      else if (input.startsWith("K=")) {
        Kp = input.substring(2).toFloat();
        mySerial.print("Kp geändert: ");
        mySerial.println(Kp);
      }
      else if (input.startsWith("C=")) {
        max_correction = input.substring(2).toFloat();
        mySerial.print("max_correction geändert: ");
        mySerial.println(max_correction);
      }
      else if (input.startsWith("I=")) {
        intervall = input.substring(2).toFloat();
        mySerial.print("intervall geändert: ");
        mySerial.println(intervall);
      }
      else {
        mySerial.print("Unbekannter Befehl: ");
        mySerial.println(input);
      }
      delay(1000);
    }
  
}