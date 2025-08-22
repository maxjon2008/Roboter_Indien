#ifndef ROBOTER_SLAVE_H
#define ROBOTER_SLAVE_H

#include <Arduino.h>
#include <Wire.h>

void receiveEvent(int bytes);
void counter_Motor();

#endif