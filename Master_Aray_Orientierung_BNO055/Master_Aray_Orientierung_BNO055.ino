#include <Wire.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>
#include <roboter_master.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#define DEBUG

unsigned long lastTime = 0;
unsigned long lastTime_M = 0;

int targetPulses_gesamt = 0; //Zielgeschwindigkeit von Roboter
int targetPulses_Motor1 = 30; //Zielgeschwindigkeit von Motor1
int targetPulses_Motor2 = 30; //Zielgeschwindigkeit von Motor2
int targetPulses_Motor3 = 30; //Zielgeschwindigkeit von Motor3
int targetPulses_Motor4 = 30; //Zielgeschwindigkeit von Motor4


//BNO055:

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

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


  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.print("test");
  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);


  // mySerial.println("set targetAngle ...");
  // delay(10000);



  // targetAngle = (float(myEulerData.h) / 16.00);
  // mySerial.print("targetAngle: ");
  // mySerial.println(targetAngle);
}

void loop() {
  if ((millis() - lastTime) >= 500) //To stream at 5Hz without using additional timers
  {
    lastTime = millis();

    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    /* Display the floating point data */
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);

    /* Optional: Display calibration status */
    displayCalStatus();

    /* Optional: Display sensor status (debug only) */
    //displaySensorStatus();

    /* New line for the next sample */
    Serial.println("");

    
    float error = wrap180(targetAngle - event.orientation.x); // -> [-180..180]

    error = Kp * error; // P-Regler

    targetPulses_Motor1 = constrain(20 + error, -120, 120);
    targetPulses_Motor2 = constrain(20 - error, -120, 120);
    targetPulses_Motor3 = constrain(20 + error, -120, 120);
    targetPulses_Motor4 = constrain(20 - error, -120, 120);


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



/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}