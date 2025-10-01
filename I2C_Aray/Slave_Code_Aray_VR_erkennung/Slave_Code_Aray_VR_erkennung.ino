#include <Wire.h>
#include <roboter_slave.h>

#define SLAVE_ADRESSE 0x08 //change for every Slave (Motor1: 0x08; Motor2: 0x09; Motor3: 0x0A; Motor4: 0x0B)

int Motor_A = 5;
int Motor_B = 6;

int encoder_Motor_1 = 2;
int encoder_Motor_2 = 3;

volatile int pulse_Motor = 0;
volatile int gesammt_pulse = 0;

unsigned long previousMillis = 0;
long intervall = 100;

//angekommene Werte:
int max_pulse = 0;
int targetPulses = 0;
int16_t daten[2];


void setup() {
  pinMode(encoder_Motor_1,INPUT);
  pinMode(encoder_Motor_2,INPUT);
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(encoder_Motor_1),counter_Motor,RISING);

  Wire.begin(SLAVE_ADRESSE); 
  Wire.onReceive(receiveEvent);
}

void loop() {

  unsigned long currentMillis = millis();

  if((currentMillis - previousMillis) >= intervall)
  {
    previousMillis = currentMillis;

    Serial.print("Motor: P/0,1s: ");
    Serial.println(pulse_Motor);
    pulse_Motor = 0;
  }

}

