#include "beeper.h"

static bool beeperInitialized = false;

bool toggleBeeperState = false;

void beeperInit() {
    if (beeperInitialized) {
        return;
    }

    pinMode(BEEPER_PIN, OUTPUT);
    digitalWrite(BEEPER_PIN, LOW);
    beeperInitialized = true;
}

void setBeeper(bool enabled) {
    if (!beeperInitialized) {
        beeperInit();
    }

    digitalWrite(BEEPER_PIN, enabled ? HIGH : LOW);
}

void toggleBeeper() {
    toggleBeeperState = !toggleBeeperState;
    setBeeper(toggleBeeperState);
}