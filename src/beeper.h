#ifndef BEEPER_H
#define BEEPER_H

#include <Arduino.h>

#define BEEPER_PIN 4

void beeperInit();
void setBeeper(bool enabled);
void toggleBeeper();

#endif
