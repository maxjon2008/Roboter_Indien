#include <Wire.h>

int Motor1_A = 5;
int Motor1_B = 6;

int encoder_Motor1 = 2;

volatile int pulse_Motor1 = 0;
volatile int gesammt_pulse = 0;

unsigned long previousMillis = 0;
long intervall = 100;

int targetPulses = 0;

int pwm_Motor1 = 0;


void setup() {
  pinMode(encoder_Motor1,INPUT);
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(encoder_Motor1),counter_Motor1,RISING);

  Wire.begin(0x01); //change for every Slave
  Wire.onReceive(receiveEvent);
}

void loop() {

  static char buffer[10];  // Platz für bis zu 9 Zeichen + Nullterminator
  static byte index = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') { // Eingabe abgeschlossen
      buffer[index] = '\0';  // String terminieren
      targetPulses = atoi(buffer);
      index = 0; // Buffer zurücksetzen
    } 
    else if (c != '\r' && index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    }
  }

  unsigned long currentMillis = millis();

  if((currentMillis - previousMillis) >= intervall)
  {
    previousMillis = currentMillis;

    Serial.println(targetPulses);
    Serial.print("gesammt Pulse: ");
    Serial.println(gesammt_pulse);

    // === Regelung ===
    pwm_Motor1 += 0.8 * (abs(targetPulses) - pulse_Motor1);

    // Begrenzen auf gültige Werte
    pwm_Motor1 = constrain(pwm_Motor1, 0, 255);

    if(targetPulses > 0){ //Vorwärts fahren
      Serial.print("v");
      analogWrite(Motor1_A, pwm_Motor1);
      analogWrite(Motor1_B, 0);
    }
    else if(targetPulses < 0){ //Rückwärts fahren
      Serial.print("r");
      analogWrite(Motor1_A, 0);
      analogWrite(Motor1_B, pwm_Motor1);  
    }
    else{
      Serial.print("0");
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

    if(targetPulses > 0){ //Vorwärts fahren
      gesammt_pulse++;
    }
    else if(targetPulses < 0){ //Rückwärts fahren
      gesammt_pulse--;
    }
    
}