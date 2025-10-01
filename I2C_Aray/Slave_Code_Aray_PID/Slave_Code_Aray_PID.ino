#include <Wire.h>
#include <roboter_slave.h>
#include <PID_v1.h>

#define SLAVE_ADRESSE 0x08 //change for every Slave (Motor1: 0x08; Motor2: 0x09; Motor3: 0x0A; Motor4: 0x0B)

int Motor_A = 5;
int Motor_B = 6;

int encoder_Motor = 2;
int encoder_Motor_2 = 3;

volatile int pulse_Motor = 0;
volatile int gesammt_pulse = 0;

unsigned long previousMillis = 0;
long intervall = 100;

//angekommene Werte:
int max_pulse = 0;
int targetPulses = 0;
int16_t daten[2];

int pwm_Motor = 0;

//PID:

// PID Variablen
double Setpoint;    // Sollwert (z.B. 0..1023 oder physikalisch)
double Input;       // Istwert (vom Sensor)
double Output;      // Stellgröße vom PID (0..255 für PWM)

// PID Tuning (Startwerte, anpassen!)
double Kp = 0.8;
double Ki = 0.0;
double Kd = 0.0;

// PID Objekt: Reihenfolge (&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, DIRECT)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, DIRECT);


void setup() {
  pinMode(encoder_Motor,INPUT);
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(encoder_Motor),counter_Motor,RISING);

  Wire.begin(SLAVE_ADRESSE);
  Wire.onReceive(receiveEvent);

  // Sample time in ms (wie oft PID neu rechnet)
  myPID.SetSampleTime(50); // 200 ms -> 5 Hz

  // Output Limits passend für analogWrite (0..255)
  myPID.SetOutputLimits(-255, 255);

  // PID einschalten
  myPID.SetMode(AUTOMATIC);
}

void loop() {

  unsigned long currentMillis = millis();

  if((currentMillis - previousMillis) >= intervall)
  {
    previousMillis = currentMillis;

    Serial.println(targetPulses);
    Serial.print("gesammt Pulse: ");
    Serial.println(gesammt_pulse);
    
    Input = pulse_Motor;
    Setpoint = targetPulses;
    myPID.Compute();           // neuen Output berechnen

    // === Regelung ===
    pwm_Motor = abs((int)round(Output));

    // Begrenzen auf gültige Werte
    pwm_Motor = constrain(pwm_Motor, 0, 255);

    if(Output < 0){ //Vorwärts fahren
      Serial.print("v");
      analogWrite(Motor_A, pwm_Motor);
      analogWrite(Motor_B, 0);
    }
    else if(Output > 0){ //Rückwärts fahren
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

  // else if((currentMillis - previousMillis) >= intervall && gesammt_pulse >= max_pulse)
  // {

  //   Serial.println("aus da max_pulse erreicht");
  //   analogWrite(Motor_A, 0);
  //   analogWrite(Motor_B, 0);

  //   Serial.print("gesammt Pulse: ");
  //   Serial.println(gesammt_pulse);
  
  // }

}

