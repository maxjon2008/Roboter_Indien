#include <Wire.h>
#include <SoftwareSerial.h>
#include <roboter_master.h>

int pwm_base = 100;
int correction = 0;

// serial data output interface (bluetooth)
SoftwareSerial mySerial(5, 4);

void setup() {
  Wire.begin();//0x05
  Serial.begin(9600);

  //Initialize the Serial Port for data output
  mySerial.begin(9600);

}

void loop() {
  sendtoSlave(0x08, 0 , 0, constrain(pwm_base - correction, -255, 255));
  sendtoSlave(0x09, 0 , 0, constrain(pwm_base + correction, -255, 255));
  sendtoSlave(0x0A, 0 , 0, constrain(pwm_base - correction, -255, 255));
  sendtoSlave(0x0B, 0 , 0, constrain(pwm_base + correction, -255, 255));

  mySerial.print("pwm_base geändert: ");
  mySerial.print(pwm_base);
  mySerial.print(" correction geändert: ");
  mySerial.println(correction);

  delay(200);
}
