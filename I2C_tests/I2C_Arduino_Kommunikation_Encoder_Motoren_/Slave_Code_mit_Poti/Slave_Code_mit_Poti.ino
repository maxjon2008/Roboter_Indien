//i2c Slave Code
#include <Wire.h>
#include "BNO055_support.h"		//Contains the bridge code between the API and Arduino
#define DEBUG

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

int potiValue = 0;
int targetPulses = 0;
int poti_Pin = A1;

void setup()
{
  Wire.begin(0x05);
  Serial.begin(9600);
  Wire.onRequest(requestEvent);

  //Initialization of the BNO055r
  BNO_Init(&myBNO); //Assigning the stucture to hold information about the device

  //Configuration to operation mode
   bno055_set_operation_mode(OPERATION_MODE_NDOF);
  //bno055_set_operation_mode(OPERATION_MODE_COMPASS);

  delay(1);

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

#ifndef DEBUG    

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
}

void loop()
{
    //Update Euler data into the structure
  bno055_read_euler_hrp(&myEulerData);

  targetPulses = map((float(myEulerData.h) / 16.00),0,360,0,130);
  Wire.beginTransmission(0x01);//Adresse des ersten Unos
  Wire.write((byte*)&targetPulses, sizeof(targetPulses));// diese Daten sollen übermittelt werden
  Wire.endTransmission();
  delay(200);
}