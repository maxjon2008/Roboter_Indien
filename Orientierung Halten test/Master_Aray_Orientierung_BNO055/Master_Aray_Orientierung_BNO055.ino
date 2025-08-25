#include <Wire.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>
#include <roboter_master.h>
#include "BNO055_support.h"

//#define DEBUG

unsigned long lastTime = 0;
unsigned long lastTime_M = 0;

int targetPulses_gesamt = 0; //Zielgeschwindigkeit von Roboter
int targetPulses_Motor1 = 30; //Zielgeschwindigkeit von Motor1
int targetPulses_Motor2 = 30; //Zielgeschwindigkeit von Motor2
int targetPulses_Motor3 = 30; //Zielgeschwindigkeit von Motor3
int targetPulses_Motor4 = 30; //Zielgeschwindigkeit von Motor4

 //BNO055
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data
struct bno055_quaternion myQuaternionData; // Structure to hold Quaternion Data

// variables hold calibration status
unsigned char accelCalibStatus = 0;	
unsigned char magCalibStatus = 0;	
unsigned char gyroCalibStatus = 0;	
unsigned char sysCalibStatus = 3;	
unsigned char calibStatus = 0; 

byte error_BNO, address_BNO; 

// serial data output interface (bluetooth)
SoftwareSerial mySerial(5, 4);

//Orientierung:
float targetAngle = 0.0;

// PID-Parameter (Startwerte, dann tunen)
float Kp = 0.3;

void setup() {
  Wire.begin();//0x05
  Serial.begin(9600);

  //Initialize the Serial Port for data output
  mySerial.begin(9600);
  Serial.println("test");
  initialize_BNO055();
  Serial.println("test2");
  mySerial.println("set targetAngle ...");
  delay(10000);



  targetAngle = (float(myEulerData.h) / 16.00);
  mySerial.print("targetAngle: ");
  mySerial.println(targetAngle);
}

void loop() {
  if ((millis() - lastTime) >= 500) //To stream at 5Hz without using additional timers
  {
    lastTime = millis();

    //Update Euler data into the structure
    bno055_read_euler_hrp(&myEulerData);

    mySerial.print("TimeStamp:");				//To read out the Time Stamp
    mySerial.print(lastTime);
    mySerial.print(",");
    mySerial.print("Heading(Yaw):");				//To read out the Heading (Yaw)
    mySerial.println(float(myEulerData.h) / 16.00);		//Convert to degrees
    
    float error = wrap180(targetAngle - float(myEulerData.h) / 16.00); // -> [-180..180]

    error = Kp * error; // P-Regler

    targetPulses_Motor1 = constrain(30 + error, -120, 120);
    targetPulses_Motor2 = constrain(30 - error, -120, 120);
    targetPulses_Motor3 = constrain(30 + error, -120, 120);
    targetPulses_Motor4 = constrain(30 - error, -120, 120);


    sendtoSlave(0x08, -targetPulses_Motor1, 9999);
    sendtoSlave(0x09, targetPulses_Motor2, 9999);
    sendtoSlave(0x0A, -targetPulses_Motor3, 9999);
    sendtoSlave(0x0B, targetPulses_Motor4, 9999);
  }







}


float wrap180(float error_angle){
  while (error_angle > 180) error_angle -= 360;
  while (error_angle < -180) error_angle += 360;
  return error_angle;
}


void initialize_BNO055() {
    // check I2C transmission
  address_BNO = 0x28;
  Wire.beginTransmission(address_BNO);
  error_BNO = Wire.endTransmission();
  if (error_BNO == 0)
  {
    mySerial.print("I2C device found at address 0x");
    mySerial.print(address_BNO, HEX);
    mySerial.println("  !");
  }
  else
  {
    mySerial.println("I2C Device error or I2C device not found");
  }
  delay(5000);

  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

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

    // bitwise AND of sensor calibration status
    calibStatus = accelCalibStatus & magCalibStatus & gyroCalibStatus & sysCalibStatus;

    // 1 second pause
    delay(1000);
  }
}