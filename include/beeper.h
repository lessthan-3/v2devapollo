#ifndef BEEPER_H
#define BEEPER_H

#include <Arduino.h>
#include "config.h"

// Module-local pin alias (from config.h)
#define BEEPER_PIN  PIN_BEEPER

void beeperInit();
void setBeeper(bool enabled);
void toggleBeeper();

#endif
