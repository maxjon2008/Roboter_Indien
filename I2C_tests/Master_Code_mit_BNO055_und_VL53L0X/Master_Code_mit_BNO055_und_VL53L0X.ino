//i2c Slave Code
#include <Wire.h>
#include "BNO055_support.h"		//Contains the bridge code between the API and Arduino
#include <SoftwareSerial.h>
#include "VL53L0X.h"

//#define DEBUG

//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data
struct bno055_quaternion myQuaternionData; // Structure to hold Quaternion Data

// variables hold calibration status
unsigned char accelCalibStatus = 0;	
unsigned char magCalibStatus = 0;	
unsigned char gyroCalibStatus = 0;	
unsigned char sysCalibStatus = 0;	
unsigned char calibStatus = 0; 

byte error, address; 

int targetPulses_gesamt = 0; //Zielgeschwindigkeit von Roboter
int targetPulses_Motor1 = 0; //Zielgeschwindigkeit von Motor1
int targetPulses_Motor2 = 0; //Zielgeschwindigkeit von Motor2

float angle = 0;
float target_angle = 180; //anzupeilender Winkel
float delta_angle = 0;
int correction = 0;

// serial data output interface (bluetooth)
SoftwareSerial mySerial(5, 4);
char status = '0';
char aktuellerStatus = '1'; // '1' = aktiv, '2' = deaktiviert

// VL53L0X Sensor
VL53L0X sensor;

void setup()
{
  Wire.begin();//0x05
  Serial.begin(9600);


  //Initialization of the BNO055r
  BNO_Init(&myBNO); //Assigning the stucture to hold information about the device

  //Configuration to operation mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  //bno055_set_operation_mode(OPERATION_MODE_COMPASS);

  delay(1);

  //Initialize the Serial Port for data output
  mySerial.begin(9600);

  // initialize calib_status
  calibStatus = 0;
  
  // wait until every sensor calibration returns 3
  while(calibStatus != 3)
  {
    // Get sensor calibration status
		bno055_get_accelcalib_status(&accelCalibStatus); //To read out the Accelerometer Calibration Status (0-3)
		bno055_get_magcalib_status(&magCalibStatus); //To read out the Magnetometer Calibration Status (0-3)
		bno055_get_gyrocalib_status(&gyroCalibStatus); //To read out the Gyroscope Calibration Status (0-3)
		bno055_get_syscalib_status(&sysCalibStatus); //To read out the System Calibration Status (0-3)

    // Message to serial monitor
    mySerial.println("waiting for sensor calibration");

#ifndef DEBUG    
    // print sensor calibration status
		mySerial.print("accelCalibStatus:");
		mySerial.print(accelCalibStatus);
    mySerial.print(",");
		mySerial.print("magCalibStatus:");
		mySerial.print(magCalibStatus);
    mySerial.print(",");
		mySerial.print("gyroCalibStatus:");
		mySerial.print(gyroCalibStatus);
    mySerial.print(",");
		mySerial.print("sysCalibStatus:");
		mySerial.println(sysCalibStatus);

    // print sensor calibration status
		Serial.print("accelCalibStatus:");
		Serial.print(accelCalibStatus);
    Serial.print(",");
		Serial.print("magCalibStatus:");
		Serial.print(magCalibStatus);
    Serial.print(",");
		Serial.print("gyroCalibStatus:");
		Serial.print(gyroCalibStatus);
    Serial.print(",");
		Serial.print("sysCalibStatus:");
		Serial.println(sysCalibStatus);

#else 
    // print sensor calibration status
		Serial.print("accelCalibStatus:");
		Serial.print(accelCalibStatus);
    Serial.print(",");
		Serial.print("magCalibStatus:");
		Serial.print(magCalibStatus);
    Serial.print(",");
		Serial.print("gyroCalibStatus:");
		Serial.print(gyroCalibStatus);
    Serial.print(",");
		Serial.print("sysCalibStatus:");
		Serial.println(sysCalibStatus);
#endif
    // 1 second pause
    delay(1000);

    // bitwise AND of sensor calibration status
    calibStatus = accelCalibStatus & magCalibStatus & gyroCalibStatus & sysCalibStatus;
  }

  // VL53L0X Sensor starten
  sensor.init();
  sensor.setTimeout(500);

  //Long Range Mode aktivieren
  sensor.setSignalRateLimit(0.1); // Standard 0.25 → kleinerer Wert = empfindlicher
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  // Messzeit verlängern (Timing Budget)
  sensor.setMeasurementTimingBudget(500000); // 200ms pro Messung

  Serial.println("VL53L0X Long Range Mode gestartet");
  mySerial.println("VL53L0X Long Range Mode gestartet");  
  
}

void loop()
{
  uint16_t distance = sensor.readRangeSingleMillimeters();

  if (sensor.timeoutOccurred() || distance >= 8190 || distance == 0) {//8190
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
  
  //Update Euler data into the structure
  bno055_read_euler_hrp(&myEulerData);

  angle = (float(myEulerData.h) / 16.00);
  if(distance >= 100){
    delta_angle = angle - target_angle;

    correction = 0.5 * delta_angle;
    Serial.println(correction);
    mySerial.print("correction: ");
    mySerial.println(correction);
    targetPulses_Motor1 = constrain(targetPulses_gesamt + correction, -130, 130);
    targetPulses_Motor2 = constrain(targetPulses_gesamt - correction, -130, 130);
  }
  else{
    targetPulses_Motor1 = 0;
    targetPulses_Motor2 = 0;
  }
  // angepasste Geschwindigkeiten je nach Winkel
  // targetPulses_Motor1 = map((float(myEulerData.h) / 16.00),0,360,-130,130);
  // targetPulses_Motor2 = -targetPulses_Motor1;
  mySerial.print("targetPulses_Motor1: ");
  mySerial.print(targetPulses_Motor1);
  mySerial.print(" targetPulses_Motor2: ");
  mySerial.println(targetPulses_Motor2);
  sendeZuSlave(0x01, targetPulses_Motor1);
  sendeZuSlave(0x02, targetPulses_Motor2);


}

void sendeZuSlave(uint8_t adresse, int16_t wert) {
  Wire.beginTransmission(adresse);
  Wire.write((byte*)&wert, sizeof(wert));
  Wire.endTransmission();
}