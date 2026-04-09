/**
 * @file job_timer.cpp
 * @brief Job elapsed-time tracking implementation
 */

#include "job_timer.h"

// Persistent accumulator — loaded from / saved to NVS by storage helpers
uint32_t totalJobTimeTenths = 0;

static uint32_t jobStartMillis      = 0;  // millis() when timer last started
static uint32_t jobElapsedMillis    = 0;  // Accumulated elapsed time (excludes pauses)
static uint32_t jobPauseStartMillis = 0;  // millis() when current pause began (0 = not paused)
static uint32_t jobBaseSeconds      = 0;  // Pre-seeded offset loaded from flash
static bool     jobTimerActive      = false;

void startJobTimer(uint32_t initialSeconds) {
    jobStartMillis      = millis();
    jobElapsedMillis    = 0;
    jobPauseStartMillis = 0;
    jobBaseSeconds      = initialSeconds;
    jobTimerActive      = true;
}

void pauseJobTimer(void) {
    if (jobTimerActive && jobPauseStartMillis == 0) {
        jobPauseStartMillis = millis();
    }
}

void resumeJobTimer(void) {
    if (jobPauseStartMillis > 0) {
        jobElapsedMillis   += millis() - jobPauseStartMillis;
        jobPauseStartMillis = 0;
    }
}

uint32_t getJobTimeSeconds(void) {
    if (!jobTimerActive) {
        return 0;
    }

    uint32_t totalElapsed;
    if (jobPauseStartMillis > 0) {
        // Currently paused — don't count time since pause started
        totalElapsed = jobPauseStartMillis - jobStartMillis - jobElapsedMillis;
    } else {
        totalElapsed = millis() - jobStartMillis - jobElapsedMillis;
    }

    return jobBaseSeconds + (totalElapsed / 1000);
}

void resetJobTimeTenths(void) {
    totalJobTimeTenths = 0;
}
