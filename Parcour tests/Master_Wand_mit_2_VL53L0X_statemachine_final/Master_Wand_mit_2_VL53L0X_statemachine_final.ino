#include <Wire.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>
#include <roboter_master.h>
#include <StateMachine.h>

#define XSHUT_VORNE 11
#define MEGA_EINGANG_PIN 7 //PIN von MEGA

// int targetPulses_gesamt = 0;   //Zielgeschwindigkeit von Roboter
// int targetPulses_Motor1 = 30;  //Zielgeschwindigkeit von Motor1
// int targetPulses_Motor2 = 30;  //Zielgeschwindigkeit von Motor2
// int targetPulses_Motor3 = 30;  //Zielgeschwindigkeit von Motor3
// int targetPulses_Motor4 = 30;  //Zielgeschwindigkeit von Motor4

// int distance_TOF_rechts_hinten = 0;
// int distance_TOF_rechts_vorne = 0;
int distance_TOF_vorne = 0;
// int solldistance = 200;
int baseSpeed = 35;
int max_pulse = 100;
int pulse = 2000; //wie viele Pulse für 1,5m

// float Kp = 1;
// float kp_angle = 0.1;
// float max_correction_angle = 1.0;
// float target_Angle = 0.0;
// float current_Angle = 0.0;
int steer = 2;

String input = "";

int status1 = 0;
int status2 = 0;

unsigned long lastTime = 0;
int intervall = 200;

// Hilfsgrößen für den Zustandswechsel
unsigned long zustand_lastTime = millis();
int zustand_intervall = 2000;

// serial data output interface (bluetooth)
SoftwareSerial mySerial(5, 4);

// VL53L0X Sensor
// VL53L0X tof_rechts_hinten;
// VL53L0X tof_rechts_vorne;
VL53L0X tof_vorne;

// Hilfsgrößen für den Zustandswechsel
// unsigned long zustand_lastTime = millis();
// int zustand_intervall = 5000;

// Zustandsautomat erzeugen
StateMachine machine = StateMachine();

// Aktionen in den Zuständen

void drehung_links() {

  if(machine.executeOnce){
    status1 = 0;
    sendtoSlave(0x08, -50, -max_pulse);
    sendtoSlave(0x09, 50, max_pulse);
    sendtoSlave(0x0A, -50, -max_pulse);
    sendtoSlave(0x0B, 50, max_pulse);
    delay(5000);
    sendtoSlave(0x08, 0, 0);
    sendtoSlave(0x09, 0, 0);
    sendtoSlave(0x0A, 0, 0);
    sendtoSlave(0x0B, 0, 0);
    delay(1000);
    status1 = 1;
  }
}

void drehung_rechts() {

  if(machine.executeOnce){
    status1 = 0;
    sendtoSlave(0x08, 50, max_pulse);
    sendtoSlave(0x09, -50, -max_pulse);
    sendtoSlave(0x0A, 50, max_pulse);
    sendtoSlave(0x0B, -50, -max_pulse);
    delay(5000);
    sendtoSlave(0x08, 0, 0);
    sendtoSlave(0x09, 0, 0);
    sendtoSlave(0x0A, 0, 0);
    sendtoSlave(0x0B, 0, 0);
    delay(1000);
    status1 = 1;
  }
}

void zurueck_setzen() {
  status2 = 0;
  sendtoSlave(0x08, -20, 1000);
  sendtoSlave(0x09, -20, 1000);
  sendtoSlave(0x0A, -20, 1000);
  sendtoSlave(0x0B, -20, 1000);
  delay(4000);
  status2 = 1;
  sendtoSlave(0x08, 0, 1000);
  sendtoSlave(0x09, 0, 1000);
  sendtoSlave(0x0A, 0, 1000);
  sendtoSlave(0x0B, 0, 1000);
}

void gerade_aus() {

  distance_TOF_vorne = tof_vorne.readRangeSingleMillimeters();
  if (tof_vorne.timeoutOccurred() || distance_TOF_vorne >= 8190 || distance_TOF_vorne == 0) {

    distance_TOF_vorne = 0;
  }

  if (distance_TOF_vorne >= 1200){
    sendtoSlave(0x08, constrain(baseSpeed, -120, 120), 0);
    sendtoSlave(0x09, constrain(baseSpeed, -120, 120), 0);
    sendtoSlave(0x0A, constrain(baseSpeed, -120, 120), 0);
    sendtoSlave(0x0B, constrain(baseSpeed, -120, 120), 0);
  }

  else{
    sendtoSlave(0x08, 20, 0);
    sendtoSlave(0x09, 20, 0);
    sendtoSlave(0x0A, 20, 0);
    sendtoSlave(0x0B, 20, 0);
  }

}

// Wand entlang fahren mit steer
void wand_entlang_rechts() {
    if ((millis() - lastTime) >= intervall)  //To stream without using additional timers
  {
    lastTime = millis();

    distance_TOF_vorne = tof_vorne.readRangeSingleMillimeters();

    if (distance_TOF_vorne >= 1000 && distance_TOF_vorne <= 8000){
      sendtoSlave(0x08, constrain(baseSpeed + steer, -120, 120), 1000);
      sendtoSlave(0x09, constrain(baseSpeed - steer, -120, 120), 1000);
      sendtoSlave(0x0A, constrain(baseSpeed + steer, -120, 120), 1000);
      sendtoSlave(0x0B, constrain(baseSpeed - steer, -120, 120), 1000);
    }

    else {
      sendtoSlave(0x08, 20 + steer, 1000);
      sendtoSlave(0x09, 20 - steer, 1000);
      sendtoSlave(0x0A, 20 + steer, 1000);
      sendtoSlave(0x0B, 20 - steer, 1000);
    }

    mySerial.print("Speed: ");
    mySerial.println(baseSpeed);
    mySerial.print("Steer: ");
    mySerial.println(steer);
  }
}



// Wand entlang fahren mit steer
void wand_entlang_links() {
    if ((millis() - lastTime) >= intervall)  //To stream without using additional timers
  {
    lastTime = millis();

    distance_TOF_vorne = tof_vorne.readRangeSingleMillimeters();

    if (distance_TOF_vorne >= 1000 && distance_TOF_vorne <= 8000){
      sendtoSlave(0x08, constrain(baseSpeed + steer, -120, 120), 1000);
      sendtoSlave(0x09, constrain(baseSpeed - steer, -120, 120), 1000);
      sendtoSlave(0x0A, constrain(baseSpeed + steer, -120, 120), 1000);
      sendtoSlave(0x0B, constrain(baseSpeed - steer, -120, 120), 1000);
    }

    else {
      sendtoSlave(0x08, 20 + steer, 1000);
      sendtoSlave(0x09, 20 - steer, 1000);
      sendtoSlave(0x0A, 20 + steer, 1000);
      sendtoSlave(0x0B, 20 - steer, 1000);
    }

    mySerial.print("Speed: ");
    mySerial.println(baseSpeed);
    mySerial.print("Steer: ");
    mySerial.println(steer);
  }
}

// Roboter anhalten
void roboter_anhalten() {
  sendtoSlave(0x08, 0, 1000);
  sendtoSlave(0x09, 0, 1000);
  sendtoSlave(0x0A, 0, 1000);
  sendtoSlave(0x0B, 0, 1000);
}

void vorwaerts_begrenzt_rechts() {
  if(machine.executeOnce){
    sendtoSlave(0x08, 20 + steer, pulse);
    sendtoSlave(0x09, 20 - steer, pulse);
    sendtoSlave(0x0A, 20 + steer, pulse);
    sendtoSlave(0x0B, 20 - steer, pulse);
  }
}

void vorwaerts_begrenzt_links() {
  if(machine.executeOnce){
    sendtoSlave(0x08, 20 - steer, pulse);
    sendtoSlave(0x09, 20 + steer, pulse);
    sendtoSlave(0x0A, 20 - steer, pulse);
    sendtoSlave(0x0B, 20 + steer, pulse);
  }
}

void ende(){
  sendtoSlave(0x08, 0, 1000);
  sendtoSlave(0x09, 0, 1000);
  sendtoSlave(0x0A, 0, 1000);
  sendtoSlave(0x0B, 0, 1000);  
}


// Funktionen der Zustandsübergänge
bool timer_1(){
  if(machine.executeOnce){
    zustand_lastTime = millis();
  }
  if ((millis() - zustand_lastTime) >= zustand_intervall){
    zustand_lastTime = millis();
    return true;
  }
  else
    return false;
}

bool timer_2(){
  if(machine.executeOnce){
    zustand_lastTime = millis();
  }
  if ((millis() - zustand_lastTime) >= 10000){
    zustand_lastTime = millis();
    return true;
  }
  else
    return false;
}

// Kollision erkannt?
bool kollision_erkannt() {
  // if ((millis() - zustand_lastTime) >= zustand_intervall){
  //   zustand_lastTime = millis();
  //   return true;
  // }
  // else
  //   return false;
  distance_TOF_vorne = tof_vorne.readRangeSingleMillimeters();
  mySerial.println(distance_TOF_vorne);
  if (tof_vorne.timeoutOccurred() || distance_TOF_vorne >= 8190 || distance_TOF_vorne == 0) {

    distance_TOF_vorne = 5000;
  }

  if (distance_TOF_vorne <= 500){
    return true;
    }
  else{
    return false;
  }
}

bool status1_done() {
  if(status1 == 1){
    return true;
  }
  else{
    return false;
  }
}
bool status2_done() {
  if(status2 == 1){
    return true;
  }
  else{
    return false;
  }
}

bool signal(){
  int wasser = digitalRead(MEGA_EINGANG_PIN);
  return (wasser == HIGH);
}

// Definition der Zustände
// Initial ist der erste Zustände
State* DREHUNG_LINKS_1 = machine.addState(&drehung_links);
State* DREHUNG_LINKS_2 = machine.addState(&drehung_links);
State* DREHUNG_LINKS_3 = machine.addState(&drehung_links);
State* DREHUNG_RECHTS_1 = machine.addState(&drehung_rechts);
State* DREHUNG_RECHTS_2 = machine.addState(&drehung_rechts);
State* DREHUNG_RECHTS_3 = machine.addState(&drehung_rechts);
State* WAND_ENTLANG_LINKS_1 = machine.addState(&wand_entlang_links);
State* WAND_ENTLANG_LINKS_2 = machine.addState(&wand_entlang_links);
State* WAND_ENTLANG_RECHTS_1 = machine.addState(&wand_entlang_rechts);
State* WAND_ENTLANG_RECHTS_2 = machine.addState(&wand_entlang_rechts);
State* ROBOTER_ANHALTEN_1 = machine.addState(&roboter_anhalten);
State* ROBOTER_ANHALTEN_2 = machine.addState(&roboter_anhalten);
State* ZURUECK_SETZEN_1 = machine.addState(&zurueck_setzen);
State* ZURUECK_SETZEN_2 = machine.addState(&zurueck_setzen);
State* ZURUECK_SETZEN_3 = machine.addState(&zurueck_setzen);
State* GERADE_AUS = machine.addState(&gerade_aus);
State* VORWAERTS_BEGRENZT_RECHTS = machine.addState(&vorwaerts_begrenzt_rechts);
State* VORWAERTS_BEGRENZT_LINKS = machine.addState(&vorwaerts_begrenzt_links);
State* ENDE = machine.addState(&ende);

// Initialisierung
void setup() {
  Wire.begin();  //0x05
  Serial.begin(9600);

  //Initialize the Serial Port for data output
  mySerial.begin(9600);

  // init_2_VL53L0X();

  // // VL53L0X Sensor starten
  // tof_vorne.init();
  while (!tof_vorne.init()) {
    Serial.println("Retry Init vorne ❌");
    delay(3000);
  }
  // //TOF_sensor.setTimeout(500);

  // //Long Range Mode aktivieren
  // // TOF_sensor.setSignalRateLimit(0.3); // Standard 0.25 → kleinerer Wert = empfindlicher
  // // TOF_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  // // TOF_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  // // Messzeit verlängern (Timing Budget)
  // TOF_sensor.setMeasurementTimingBudget(200000); // 200ms pro Messung

  // Serial.println("VL53L0X Long Range Mode gestartet");
  mySerial.println("VL53L0X Long Range Mode gestartet");

// Schleife läuft, bis eine gültige Eingabe empfangen wird
  while (true) {
    if (mySerial.available()) {
      input = mySerial.readStringUntil('\n');
      input.trim();

      // Filter: Nur A oder B sind erlaubt
      if (input == "1A" || input == "1B" || input == "2A" || input == "2B") {
        break;  // gültige Eingabe → while beenden
      } else {
        Serial.print("Ungültige Eingabe empfangen: ");
        Serial.println(input);
        input = ""; // Eingabe verwerfen
      }
    }
  }

  Serial.print("Gültige Eingabe empfangen: ");
  Serial.println(input);

  if (input == "1A"){
    WAND_ENTLANG_RECHTS_1->addTransition(&kollision_erkannt, ROBOTER_ANHALTEN_1);
    ROBOTER_ANHALTEN_1->addTransition(&timer_1, DREHUNG_LINKS_1);
    DREHUNG_LINKS_1->addTransition(&status1_done, ZURUECK_SETZEN_1);
    ZURUECK_SETZEN_1->addTransition(&status2_done, WAND_ENTLANG_RECHTS_2);

    WAND_ENTLANG_RECHTS_2->addTransition(&kollision_erkannt, ROBOTER_ANHALTEN_2);
    ROBOTER_ANHALTEN_2->addTransition(&timer_1, DREHUNG_LINKS_2);
    DREHUNG_LINKS_2->addTransition(&status1_done, ZURUECK_SETZEN_2);
    ZURUECK_SETZEN_2->addTransition(&status2_done, VORWAERTS_BEGRENZT_RECHTS);

    VORWAERTS_BEGRENZT_RECHTS->addTransition(&timer_2, DREHUNG_LINKS_3);
    DREHUNG_LINKS_3->addTransition(&status1_done, ZURUECK_SETZEN_3);
    ZURUECK_SETZEN_3->addTransition(&status2_done, GERADE_AUS);

    GERADE_AUS->addTransition(&signal, ENDE);
  }
  if (input == "1B"){

    WAND_ENTLANG_RECHTS_2->addTransition(&kollision_erkannt, ROBOTER_ANHALTEN_2);
    ROBOTER_ANHALTEN_2->addTransition(&timer_1, DREHUNG_LINKS_2);
    DREHUNG_LINKS_2->addTransition(&status1_done, ZURUECK_SETZEN_2);
    ZURUECK_SETZEN_2->addTransition(&status2_done, VORWAERTS_BEGRENZT_RECHTS);

    VORWAERTS_BEGRENZT_RECHTS->addTransition(&timer_2, DREHUNG_LINKS_3);
    DREHUNG_LINKS_3->addTransition(&status1_done, ZURUECK_SETZEN_3);
    ZURUECK_SETZEN_3->addTransition(&status2_done, GERADE_AUS);

    GERADE_AUS->addTransition(&signal, ENDE);
  }
  if (input == "2A"){
    WAND_ENTLANG_LINKS_1->addTransition(&kollision_erkannt, ROBOTER_ANHALTEN_1);
    ROBOTER_ANHALTEN_1->addTransition(&timer_1, DREHUNG_RECHTS_1);
    DREHUNG_RECHTS_1->addTransition(&status1_done, ZURUECK_SETZEN_1);
    ZURUECK_SETZEN_1->addTransition(&status2_done, WAND_ENTLANG_LINKS_2);

    WAND_ENTLANG_LINKS_2->addTransition(&kollision_erkannt, ROBOTER_ANHALTEN_2);
    ROBOTER_ANHALTEN_2->addTransition(&timer_1, DREHUNG_RECHTS_2);
    DREHUNG_RECHTS_2->addTransition(&status1_done, ZURUECK_SETZEN_2);
    ZURUECK_SETZEN_2->addTransition(&status2_done, VORWAERTS_BEGRENZT_LINKS);

    VORWAERTS_BEGRENZT_LINKS->addTransition(&timer_2, DREHUNG_RECHTS_3);
    DREHUNG_RECHTS_3->addTransition(&status1_done, ZURUECK_SETZEN_3);
    ZURUECK_SETZEN_3->addTransition(&status2_done, GERADE_AUS);

    GERADE_AUS->addTransition(&signal, ENDE);
  }
  if (input == "2B"){
    WAND_ENTLANG_LINKS_2->addTransition(&kollision_erkannt, ROBOTER_ANHALTEN_2);
    ROBOTER_ANHALTEN_2->addTransition(&timer_1, DREHUNG_RECHTS_2);
    DREHUNG_RECHTS_2->addTransition(&status1_done, ZURUECK_SETZEN_2);
    ZURUECK_SETZEN_2->addTransition(&status2_done, VORWAERTS_BEGRENZT_LINKS);

    VORWAERTS_BEGRENZT_LINKS->addTransition(&timer_2, DREHUNG_RECHTS_3);
    DREHUNG_RECHTS_3->addTransition(&status1_done, ZURUECK_SETZEN_3);
    ZURUECK_SETZEN_3->addTransition(&status2_done, GERADE_AUS);

    GERADE_AUS->addTransition(&signal, ENDE);
  }

}


// void init_2_VL53L0X() {
//   pinMode(XSHUT_RECHTS_VORNE, OUTPUT);
//   pinMode(XSHUT_RECHTS_HINTEN, OUTPUT);

//   // alle Sensoren ausschalten
//   digitalWrite(XSHUT_RECHTS_VORNE, LOW);
//   digitalWrite(XSHUT_RECHTS_HINTEN, LOW);
//   delay(10);

//   // Sensor vorne einschalten + initialisieren + Adresse setzen
//   digitalWrite(XSHUT_RECHTS_VORNE, HIGH);
//   delay(10);
//   while (!tof_rechts_vorne.init()) {
//     Serial.println("Retry Init Rechts vorne ❌");
//     delay(3000);
//   }
//   tof_rechts_vorne.setAddress(0x30);
//   tof_rechts_vorne.startContinuous();

//   // Sensor links einschalten + initialisieren + Adresse setzen
//   digitalWrite(XSHUT_RECHTS_HINTEN, HIGH);
//   delay(10);
//   while (!tof_rechts_hinten.init()) {
//     Serial.println("Retry Init Rechts hinten ❌");
//     delay(3000);
//   }
//   tof_rechts_hinten.setAddress(0x31);
//   tof_rechts_hinten.startContinuous();

//   Serial.println("Alle Sensoren bereit ✅");


// }

// Loop
void loop() {
  machine.run();

  if (mySerial.available()) {  //Steuerung der einzelnen Parameter über Bluetoth
    String input = mySerial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("S=")) {
      baseSpeed = input.substring(2).toInt();
      mySerial.print("Speed geändert: ");
      mySerial.println(baseSpeed);

    } else if (input.startsWith("ST=")) {
      steer = input.substring(2).toFloat();
      mySerial.print("Steer geändert: ");
      mySerial.println(steer);

    } else if (input.startsWith("P=")) {
      max_pulse = input.substring(2).toFloat();
      mySerial.print("max_pulse geändert: ");
      mySerial.println(max_pulse);

    } else if (input.startsWith("I=")) {
      intervall = input.substring(2).toFloat();
      mySerial.print("intervall geändert: ");
      mySerial.println(intervall);
    } else {
      mySerial.print("Unbekannter Befehl: ");
      mySerial.println(input);
    }
    delay(1000);
  }
}