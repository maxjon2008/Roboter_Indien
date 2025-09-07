#include <Wire.h>
#include <roboter_slave.h>

#define SLAVE_ADRESSE 0x08 //change for every Slave (Motor1: 0x08; Motor2: 0x09; Motor3: 0x0A; Motor4: 0x0B)

int Motor_A = 5;
int Motor_B = 6;

int encoder_Motor = 2;

volatile int pulse_Motor = 0;
volatile int gesammt_pulse = 0;

unsigned long previousMillis = 0;
long intervall = 100;

//angekommene Werte:
int max_pulse = 0;
int targetPulses = 0;
int16_t daten[2];

int pwm_Motor = 0;


void setup() {
  pinMode(encoder_Motor,INPUT);
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(encoder_Motor),counter_Motor,RISING);

  Wire.begin(SLAVE_ADRESSE); 
  Wire.onReceive(receiveEvent);
}

void loop() {

  unsigned long currentMillis = millis();

  if((currentMillis - previousMillis) >= intervall && gesammt_pulse <= max_pulse)
  {
    previousMillis = currentMillis;

    Serial.println(targetPulses);
    Serial.print("gesammt Pulse: ");
    Serial.println(gesammt_pulse);

    // === Regelung ===
    pwm_Motor += 0.8 * (abs(targetPulses) - pulse_Motor);

    // Begrenzen auf gültige Werte
    pwm_Motor = constrain(pwm_Motor, 0, 255);

    if(targetPulses > 0){ //Vorwärts fahren
      Serial.print("v");
      analogWrite(Motor_A, pwm_Motor);
      analogWrite(Motor_B, 0);
    }
    else if(targetPulses < 0){ //Rückwärts fahren
      Serial.print("r");
      analogWrite(Motor_A, 0);
      analogWrite(Motor_B, pwm_Motor);  
    }
    else{
      Serial.print("0");
      analogWrite(Motor_A, 0);
      analogWrite(Motor_B, 0);   
    }

    Serial.print("Motor: P/0,1s: ");
    Serial.println(pulse_Motor);
    pulse_Motor = 0;
  }

  else if((currentMillis - previousMillis) >= intervall && gesammt_pulse >= max_pulse)
  {

    Serial.println("aus da max_pulse erreicht");
    analogWrite(Motor_A, 0);
    analogWrite(Motor_B, 0);

    Serial.print("gesammt Pulse: ");
    Serial.println(gesammt_pulse);
  
  }

}

