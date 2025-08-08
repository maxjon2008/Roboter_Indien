#include <Wire.h>

int Motor1_A = 5;
int Motor1_B = 6;

int encoder_Motor1 = 2;

volatile int pulse_Motor1 = 0;

unsigned long previousMillis = 0;
long intervall = 100;

int targetPulses = 0;

int pwm_Motor1 = 0;


void setup() {
  pinMode(encoder_Motor1,INPUT);
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(encoder_Motor1),counter_Motor1,RISING);

  Wire.begin(0x02); //change for every Slave
  Wire.onReceive(receiveEvent);
}

void loop() {
  unsigned long currentMillis = millis();

  if((currentMillis - previousMillis) >= intervall)
  {
    previousMillis = currentMillis;

    Serial.println(targetPulses);

    // === Regelung ===
    pwm_Motor1 += 0.8 * (abs(targetPulses) - pulse_Motor1);

    // Begrenzen auf gültige Werte
    pwm_Motor1 = constrain(pwm_Motor1, 0, 255);

    if(targetPulses > 0){ //Vorwärts fahren
      analogWrite(Motor1_A, pwm_Motor1);
      analogWrite(Motor1_B, 0);
    }
    else if(targetPulses < 0){ //Rückwärts fahren
      analogWrite(Motor1_A, 0);
      analogWrite(Motor1_B, pwm_Motor1);  
    }
    else{
      analogWrite(Motor1_A, 0);
      analogWrite(Motor1_B, 0);   
    }

    Serial.print("Motor 1: P/0,1s: ");
    Serial.println(pulse_Motor1);
    pulse_Motor1 = 0;
  }



}

void receiveEvent(int bytes) {
  if (bytes >= sizeof(int)) {
    Wire.readBytes((byte*)&targetPulses, sizeof(int));
    Serial.println(targetPulses);
  }
}

// void receiveEvent(int bytes) {
//   for (int i = 0; i < sizeof(int); i++) {
//     ((byte*)&targetPulses)[i] = Wire.read();
//   }
// }

void counter_Motor1(){
  pulse_Motor1++;
}