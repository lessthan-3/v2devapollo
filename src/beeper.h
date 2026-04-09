#ifndef BEEPER_H
#define BEEPER_H

#include <Arduino.h>
#include "config.h"

void beeperInit();
void setBeeper(bool enabled);
void toggleBeeper();

#endif
