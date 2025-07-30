#include <Wire.h>

int Motor1_A = 5;
int Motor1_B = 6;
int Motor2_A = 10;
int Motor2_B = 9;

int encoder_Motor1 = 2;
int encoder_Motor2 = 3;

volatile int pulse_Motor1 = 0;
volatile int pulse_Motor2 = 0;

unsigned long previousMillis = 0;
long intervall = 100;

int targetPulses = 0;

int pwm_Motor1 = 0;
int pwm_Motor2 = 0;

//Poti einbetten
int potiPin = A1;

void setup() {
  pinMode(encoder_Motor1,INPUT);
  pinMode(encoder_Motor2,INPUT);
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(encoder_Motor1),counter_Motor1,RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_Motor2),counter_Motor2,RISING);

  Wire.begin();
}

void loop() {
  unsigned long currentMillis = millis();

  if((currentMillis - previousMillis) >= intervall)
  {
    previousMillis = currentMillis;

    Wire.requestFrom(0x05,3);
    
    int targetPulses = Wire.read();
    Serial.println(targetPulses);

    // === Regelung ===
    pwm_Motor1 += 0.8 * (targetPulses - pulse_Motor1);
    pwm_Motor2 += 0.8 * (targetPulses - pulse_Motor2);

    // Begrenzen auf gültige Werte
    pwm_Motor1 = constrain(pwm_Motor1, 0, 255);
    pwm_Motor2 = constrain(pwm_Motor2, 0, 255);

    analogWrite(Motor1_A, pwm_Motor1);
    analogWrite(Motor1_B, 0);
    analogWrite(Motor2_A, pwm_Motor2);
    analogWrite(Motor2_B, 0);

    Serial.print("Motor 1: P/0,1s: ");
    Serial.print(pulse_Motor1);
    Serial.print(" Motor 2: P/0,1s: ");
    Serial.println(pulse_Motor2);
    pulse_Motor1 = 0;
    pulse_Motor2 = 0;
  }

  int potiValue = analogRead(potiPin);
  targetPulses = map(potiValue,0,1023,0,130);

}

void counter_Motor1(){
  pulse_Motor1++;
}

void counter_Motor2(){
  pulse_Motor2++;
}
