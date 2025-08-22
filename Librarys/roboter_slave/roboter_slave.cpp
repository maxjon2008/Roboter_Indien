#include "roboter_slave.h"

extern int pulse_Motor;
extern int gesammt_pulse;
extern int targetPulses;
extern int max_pulse;
extern int16_t daten[2];

void counter_Motor(){
  pulse_Motor++;

  gesammt_pulse++;

    
}

void receiveEvent(int bytes) {
  if (bytes >= sizeof(daten)) {
    Wire.readBytes((byte*)daten, sizeof(daten));
    targetPulses = daten[0];
    max_pulse = daten[1];
    gesammt_pulse = 0;
  }
}
