#ifndef ROBOTER_MASTER_H
#define ROBOTER_MASTER_H

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "VL53L0X.h" 

void sendtoSlave(uint8_t adresse, int16_t pps, int16_t pulse);
void checkDistanceAndSetPulses_on_off(int pulse_an);

#endif