#include <Wire.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>
#include <roboter_master.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//#define DEBUG

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool offsetsSet = false;

unsigned long lastTime = 0;
unsigned long lastTime_M = 0;

int targetPulses_gesamt = 0; //Zielgeschwindigkeit von Roboter
int targetPulses_Motor1 = 30; //Zielgeschwindigkeit von Motor1
int targetPulses_Motor2 = 30; //Zielgeschwindigkeit von Motor2
int targetPulses_Motor3 = 30; //Zielgeschwindigkeit von Motor3
int targetPulses_Motor4 = 30; //Zielgeschwindigkeit von Motor4


// serial data output interface (bluetooth)
SoftwareSerial mySerial(5, 4);

//Orientierung:
float targetAngle = 0.0;

int basespeed = 0;

// PID-Parameter (Startwerte, dann tunen)
float Kp = 0.1;


void setup() {
  Wire.begin();//0x05
  Serial.begin(9600);

  //Initialize the Serial Port for data output
  mySerial.begin(9600);

  while (!Serial) delay(10);  // wait for serial port to open!

  mySerial.println("Orientation Sensor Test"); mySerial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    mySerial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  mySerial.println("Warte auf Kalibrierung...");
  uint8_t system, gyro, accel, mag;
  do {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    mySerial.print("Calib Status -> Sys=");
    mySerial.print(system);
    mySerial.print(" Gyro=");
    mySerial.print(gyro);
    mySerial.print(" Accel=");
    mySerial.print(accel);
    mySerial.print(" Mag=");
    mySerial.println(mag);
    delay(500);
  } while (!(system == 3 && gyro == 3 && accel == 3 && mag == 3));

  // adafruit_bno055_offsets_t offsets;
  // bno.getSensorOffsets(offsets);
  // bno.setSensorOffsets(offsets);

  mySerial.println("Kalibrierung abgeschlossen!");


  mySerial.println("set targetAngle ...");
  unsigned long start = millis();
  while (millis() - start < 10000) {
    delay(500);
    //Update Euler data into the structure
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    mySerial.print("Angle: ");
    mySerial.println(orientationData.orientation.x);
    }
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  targetAngle = (orientationData.orientation.x);
  mySerial.print("targetAngle: ");
  mySerial.println(targetAngle);
}

void loop() {

  if ((millis() - lastTime) >= 200) //To stream without using additional timers
  {
    lastTime = millis();

    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);


    mySerial.print(" targetAngle: ");
    mySerial.print(targetAngle);

    /* Display the floating point data */
    mySerial.print(" X: ");
    mySerial.println(orientationData.orientation.x, 2);
    
    float error = wrap180(targetAngle - orientationData.orientation.x); // -> [-180..180]

    error = Kp * error; // P-Regler

    targetPulses_Motor1 = constrain(basespeed + error, -120, 120);
    targetPulses_Motor2 = constrain(basespeed - error, -120, 120);
    targetPulses_Motor3 = constrain(basespeed + error, -120, 120);
    targetPulses_Motor4 = constrain(basespeed - error, -120, 120);


    sendtoSlave(0x08, targetPulses_Motor1, 9999);
    sendtoSlave(0x09, targetPulses_Motor2, 9999);
    sendtoSlave(0x0A, targetPulses_Motor3, 9999);
    sendtoSlave(0x0B, targetPulses_Motor4, 9999);

    if(mySerial.available()){
      String input = mySerial.readStringUntil('\n');
      input.trim();

      if (input.startsWith("S=")) {
        basespeed = input.substring(2).toInt();
        mySerial.print("Speed geändert: ");
        mySerial.println(basespeed);
      }

      else if (input.startsWith("A=")) {
        targetAngle = input.substring(2).toInt();
        mySerial.print("targetAngle geändert: ");
        mySerial.println(targetAngle);
      }
      else if (input.startsWith("K=")) {
        Kp = input.substring(2).toFloat();
        mySerial.print("Kp geändert: ");
        mySerial.println(Kp);
      }
      else {
        mySerial.print("Unbekannter Befehl: ");
        mySerial.println(input);
      }
      delay(1000);
    }

  }

}


float wrap180(float error_angle){
  while (error_angle > 180) error_angle -= 360;
  while (error_angle < -180) error_angle += 360;
  return error_angle;
}