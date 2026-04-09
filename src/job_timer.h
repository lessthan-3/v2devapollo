/**
 * @file job_timer.h
 * @brief Job elapsed-time tracking
 *
 * The job timer starts when the runtime screen is entered and tracks
 * the total active time, excluding any periods spent in Power Pause.
 */

#ifndef JOB_TIMER_H
#define JOB_TIMER_H

#include <Arduino.h>

/**
 * @brief Persistent accumulated job time in tenths of an hour.
 *        Loaded from / saved to NVS by storage.h helpers.
 *        Incremented every 6 minutes (0.1 hr) while the motor is running.
 */
extern uint32_t totalJobTimeTenths;

/**
 * @brief Start the job timer, optionally pre-seeded with already-elapsed seconds
 *        loaded from flash (totalJobTimeTenths converted to seconds).
 *        Call when entering the runtime screen.
 *
 * @param initialSeconds Seconds to add as a base offset (default 0).
 */
void startJobTimer(uint32_t initialSeconds = 0);

/**
 * @brief Pause the job timer (e.g. when Power Pause activates).
 *        Calling while already paused has no effect.
 */
void pauseJobTimer(void);

/**
 * @brief Resume the job timer after a pause.
 *        The paused duration is excluded from the elapsed total.
 */
void resumeJobTimer(void);

/**
 * @brief Get the total active job time in whole seconds.
 *        Paused time is not counted.
 *
 * @return Elapsed seconds, or 0 if the timer has never been started.
 */
uint32_t getJobTimeSeconds(void);

/**
 * @brief Reset the persistent job time accumulator to zero.
 *        Does NOT reset the current in-session timer — call from About screen reset action.
 */
void resetJobTimeTenths(void);

#endif // JOB_TIMER_H
