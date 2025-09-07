#include <Wire.h>
#include <VL53L0X.h>

#define XSHUT_VORNE 11
#define XSHUT_LINKS 12
#define XSHUT_RECHTS 13

VL53L0X tof_vorne;
VL53L0X tof_links;
VL53L0X tof_rechts;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  init_3_VL53L0X();
}

void loop() {
  
  int distance_vorne  = tof_vorne.readRangeContinuousMillimeters();
  int distance_links  = tof_links.readRangeContinuousMillimeters();
  int distance_rechts = tof_rechts.readRangeContinuousMillimeters();

  Serial.print("Vorne: ");  Serial.print(distance_vorne);  
  Serial.print(" | Links: ");  Serial.print(distance_links);
  Serial.print(" | Rechts: "); Serial.println(distance_rechts);

  delay(100);

}

void init_3_VL53L0X(){
  pinMode(XSHUT_VORNE, OUTPUT);
  pinMode(XSHUT_LINKS, OUTPUT);
  pinMode(XSHUT_RECHTS, OUTPUT);

  // alle Sensoren ausschalten
  digitalWrite(XSHUT_VORNE, LOW);
  digitalWrite(XSHUT_LINKS, LOW);
  digitalWrite(XSHUT_RECHTS, LOW);
  delay(10);

  // Sensor vorne einschalten + initialisieren + Adresse setzen
  digitalWrite(XSHUT_VORNE, HIGH);
  delay(10);
  while (!tof_vorne.init()){
    Serial.println("Retry Init vorne ❌");
    delay(3000);
  }
  tof_vorne.setAddress(0x30);
  tof_vorne.startContinuous();

  // Sensor links einschalten + initialisieren + Adresse setzen
  digitalWrite(XSHUT_LINKS, HIGH);
  delay(10);
  while (!tof_links.init()){
    Serial.println("Retry Init links ❌");
    delay(3000);
  }
  tof_links.setAddress(0x31);
  tof_links.startContinuous();

  // Sensor rechts einschalten + initialisieren + Adresse setzen
  digitalWrite(XSHUT_RECHTS, HIGH);
  delay(10);
  while (!tof_rechts.init()){
    Serial.println("Retry Init rechts ❌");
    delay(3000);
  }
  tof_rechts.setAddress(0x32);
  tof_rechts.startContinuous();

  Serial.println("Alle Sensoren bereit ✅");
}