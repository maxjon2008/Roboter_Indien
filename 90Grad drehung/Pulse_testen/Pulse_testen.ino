#include <Wire.h>
#include <SoftwareSerial.h>
#include <roboter_master.h>

unsigned long lastTime = 0;
int intervall = 5000;

int speed = 0;
int max_pulse = 200;
int status = 1;

// serial data output interface (bluetooth)
SoftwareSerial mySerial(5, 4);

void setup() {
  Wire.begin();//0x05
  Serial.begin(9600);

  //Initialize the Serial Port for data output
  mySerial.begin(9600);

}

void loop() {

  if ((millis() - lastTime) >= intervall) //To stream without using additional timers
  {
    Serial.println("if läuft! ");
    lastTime = millis();
   
    if(status == 0){
      sendtoSlave(0x08, -speed, -max_pulse);
      sendtoSlave(0x09, speed, max_pulse);
      sendtoSlave(0x0A, -speed, -max_pulse);
      sendtoSlave(0x0B, speed, max_pulse);

      Serial.println("Motoren gesendet! ");

      status = 1;
    }

    mySerial.print("status geändert: ");
    mySerial.print(status);
    mySerial.print(" speed geändert: ");
    mySerial.print(speed);
    mySerial.print(" max_pulse geändert: ");
    mySerial.println(max_pulse);

  }

  if(mySerial.available()){ //Steuerung der einzelnen Parameter über Bluetoth
    String input = mySerial.readStringUntil('\n');
    input.trim();

      if (input.startsWith("S=")) {
        speed = input.substring(2).toInt();
        mySerial.print("Speed geändert: ");
        mySerial.println(speed);
        status = 0;
        mySerial.print("neu gestartet ✅");
      }
      else if (input.startsWith("P=")) {
        max_pulse = input.substring(2).toInt();
        mySerial.print("max_pulse geändert: ");
        mySerial.println(max_pulse);
        status = 0;
        mySerial.print("neu gestartet ✅");
      }
      else if (input.startsWith("N=")) {
        status = 0;
        mySerial.print("neu gestartet ✅");
      }
      else {
        mySerial.print("Unbekannter Befehl: ");
        mySerial.println(input);
      }
      delay(1000);
    }

}
