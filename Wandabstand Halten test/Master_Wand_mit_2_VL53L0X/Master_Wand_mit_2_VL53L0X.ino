#include <Wire.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>
#include <roboter_master.h>

#define XSHUT_RECHTS_VORNE 12
#define XSHUT_RECHTS_HINTEN 13

int targetPulses_gesamt = 0; //Zielgeschwindigkeit von Roboter
int targetPulses_Motor1 = 30; //Zielgeschwindigkeit von Motor1
int targetPulses_Motor2 = 30; //Zielgeschwindigkeit von Motor2
int targetPulses_Motor3 = 30; //Zielgeschwindigkeit von Motor3
int targetPulses_Motor4 = 30; //Zielgeschwindigkeit von Motor4

int distance_TOF_rechts_hinten = 0;
int distance_TOF_rechts_vorne = 0;
int solldistance = 200;
int baseSpeed = 0;

float Kp = 1;
float kp_angle = 0.1;
float max_correction_angle = 1.0;
float target_Angle = 0.0;
float current_Angle = 0.0;

unsigned long lastTime = 0;
int intervall = 200;

// serial data output interface (bluetooth)
SoftwareSerial mySerial(5, 4);

// VL53L0X Sensor
VL53L0X tof_rechts_hinten;
VL53L0X tof_rechts_vorne;

void setup() {
  Wire.begin();//0x05
  Serial.begin(9600);

  //Initialize the Serial Port for data output
  mySerial.begin(9600);

  init_2_VL53L0X();

  // // VL53L0X Sensor starten
  // TOF_sensor.init();
  // //TOF_sensor.setTimeout(500);

  // //Long Range Mode aktivieren
  // // TOF_sensor.setSignalRateLimit(0.3); // Standard 0.25 → kleinerer Wert = empfindlicher
  // // TOF_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  // // TOF_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  // // Messzeit verlängern (Timing Budget)
  // TOF_sensor.setMeasurementTimingBudget(200000); // 200ms pro Messung

  // Serial.println("VL53L0X Long Range Mode gestartet");
  // mySerial.println("VL53L0X Long Range Mode gestartet");  


}

void loop() {

  if ((millis() - lastTime) >= intervall) //To stream without using additional timers
  {
    lastTime = millis();

    distance_TOF_rechts_vorne = tof_rechts_vorne.readRangeContinuousMillimeters();
    distance_TOF_rechts_hinten = tof_rechts_hinten.readRangeContinuousMillimeters();

    // Messbereich prüfen
    if (tof_rechts_hinten.timeoutOccurred() || tof_rechts_vorne.timeoutOccurred() || distance_TOF_rechts_vorne >= 8190 || distance_TOF_rechts_vorne == 0 || distance_TOF_rechts_hinten >= 8190 || distance_TOF_rechts_hinten == 0) {
        Serial.println("Außerhalb des Messbereichs");
        mySerial.println("Außerhalb des Messbereichs");
    
        distance_TOF_rechts_vorne = 0;
        distance_TOF_rechts_hinten = 0;
    } 
    else {
        distance_TOF_rechts_vorne = distance_TOF_rechts_vorne;
        distance_TOF_rechts_hinten = distance_TOF_rechts_hinten;

        
        mySerial.print("TOF_vorne: ");
        mySerial.println(distance_TOF_rechts_vorne);
        mySerial.print("TOF_hinten: ");
        mySerial.println(distance_TOF_rechts_hinten);
    }
     
    // current_Angle = atan2((distance_TOF_rechts_vorne - distance_TOF_rechts_hinten), 232); //Abstand zwischen Sensoren anpassen!!! // aktuelle Winkel berechnen
    current_Angle = -atan2((float)(distance_TOF_rechts_vorne - distance_TOF_rechts_hinten), 232.0) * 180.0 / PI; // jetzt in Grad


    int error_distance = (distance_TOF_rechts_hinten + distance_TOF_rechts_vorne)/2 - solldistance; //mittleren Anstand berechnung und error berechnen

    target_Angle =  constrain(kp_angle * error_distance, -max_correction_angle, max_correction_angle); //P-Regler für Korrektur falls zu weit weg/nah

    float error_angle = current_Angle - target_Angle; //korrektur Winkel berechnen

    int steer = Kp * error_angle; //P-Regler für korrektur Winkel; maximalen Korrektur Winkel beschränken

    sendtoSlave(0x08, constrain(baseSpeed - steer, -120, 120), 200);
    sendtoSlave(0x09, constrain(baseSpeed + steer, -120, 120), 200);
    sendtoSlave(0x0A, constrain(baseSpeed - steer, -120, 120), 200);
    sendtoSlave(0x0B, constrain(baseSpeed + steer, -120, 120), 200);

    mySerial.print("Speed: ");
    mySerial.print(baseSpeed);
    mySerial.print(" solldistance geändert: ");
    mySerial.print(solldistance);
    mySerial.print(" Kp geändert: ");
    mySerial.print(Kp);
    mySerial.print(" kp_angle geändert: ");
    mySerial.print(kp_angle);    
    mySerial.print(" max_correction_angle geändert: ");
    mySerial.print(max_correction_angle);
    mySerial.print(" Intervall geändert: ");
    mySerial.println(intervall);

  }

  if(mySerial.available()){ //Steuerung der einzelnen Parameter über Bluetoth
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
      else if (input.startsWith("A=")) {
        kp_angle = input.substring(2).toFloat();
        mySerial.print("kp_angle geändert: ");
        mySerial.println(kp_angle);
      }
      else if (input.startsWith("C=")) {
        max_correction_angle = input.substring(2).toFloat();
        mySerial.print("max_correction_angle geändert: ");
        mySerial.println(max_correction_angle);
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

void init_2_VL53L0X(){
  pinMode(XSHUT_RECHTS_VORNE, OUTPUT);
  pinMode(XSHUT_RECHTS_HINTEN, OUTPUT);

  // alle Sensoren ausschalten
  digitalWrite(XSHUT_RECHTS_VORNE, LOW);
  digitalWrite(XSHUT_RECHTS_HINTEN, LOW);
  delay(10);

  // Sensor vorne einschalten + initialisieren + Adresse setzen
  digitalWrite(XSHUT_RECHTS_VORNE, HIGH);
  delay(10);
  while (!tof_rechts_vorne.init()){
    Serial.println("Retry Init Rechts vorne ❌");
    delay(3000);
  }
  tof_rechts_vorne.setAddress(0x30);
  tof_rechts_vorne.startContinuous();

  // Sensor links einschalten + initialisieren + Adresse setzen
  digitalWrite(XSHUT_RECHTS_HINTEN, HIGH);
  delay(10);
  while (!tof_rechts_hinten.init()){
    Serial.println("Retry Init Rechts hinten ❌");
    delay(3000);
  }
  tof_rechts_hinten.setAddress(0x31);
  tof_rechts_hinten.startContinuous();
  
  Serial.println("Alle Sensoren bereit ✅");
}