#include <Wire.h>
#include <roboter_slave.h>

#define SLAVE_ADRESSE 0x0B //change for every Slave (Motor1: 0x08; Motor2: 0x09; Motor3: 0x0A; Motor4: 0x0B)

int Motor_A = 5;
int Motor_B = 6;

int encoder_Motor = 2;
int encoder_Motor_2 = 3;

volatile int pulse_Motor = 0;
volatile int gesammt_pulse = 0;

unsigned long previousMillis = 0;
long intervall = 50;

//angekommene Werte:
int max_pulse = 0;
int targetPulses = 0;
int16_t daten[2];

int pwm_Motor = 0;
int pwm_Motor_geregelt = 0;


void setup() {
  pinMode(encoder_Motor,INPUT);
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(encoder_Motor),counter_Motor,RISING);

  Wire.begin(SLAVE_ADRESSE); 
  Wire.onReceive(receiveEvent);
}

void loop() {

  unsigned long currentMillis = millis();

  if((currentMillis - previousMillis) >= intervall && (max_pulse == 0 || gesammt_pulse <= max_pulse))
    // Code hier läuft entweder:
    // - immer, wenn max_pulse == 0
    // - oder nur solange gesammt_pulse <= max_pulse
  {
    previousMillis = currentMillis;

    Serial.println(targetPulses);
    Serial.print("gesammt Pulse: ");
    Serial.println(gesammt_pulse);

    // === Regelung ===
    pwm_Motor_geregelt += 0.10 * (targetPulses - pulse_Motor*2);

    

    if (abs(pulse_Motor) <= 10){
      pwm_Motor_geregelt = constrain(pwm_Motor_geregelt, -100, 100);
    }
    pwm_Motor_geregelt = constrain(pwm_Motor_geregelt, -255, 255);

    if (abs(targetPulses) < 10 && pulse_Motor == 0){
      pwm_Motor_geregelt = 0;
    }

    // Begrenzen auf gültige Werte
    pwm_Motor = abs(pwm_Motor_geregelt);

    if(pwm_Motor_geregelt > 0){ //Vorwärts fahren
      Serial.print("v");
      analogWrite(Motor_A, pwm_Motor);
      analogWrite(Motor_B, 0);
    }
    else if(pwm_Motor_geregelt < 0){ //Rückwärts fahren
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
    Serial.println(pulse_Motor*2);
    pulse_Motor = 0;
  }

  else if((currentMillis - previousMillis) >= intervall && gesammt_pulse >= max_pulse && max_pulse != 0)
  {

    Serial.println("aus da max_pulse erreicht");
    analogWrite(Motor_A, 0);
    analogWrite(Motor_B, 0);

    Serial.print("gesammt Pulse: ");
    Serial.println(gesammt_pulse);
  
  }

}

