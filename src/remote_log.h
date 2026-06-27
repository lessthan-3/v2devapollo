/**
 * @file remote_log.h
 * @brief Remote telemetry logging over WiFi.
 *
 * Sends a single JSON snapshot to the log endpoint defined by OTA_LOG_URL
 * whenever the device connects to WiFi during an OTA update session.
 *
 * The payload is extensible: callers pass an event label and the function
 * always includes firmware version, uptime, all persisted settings, and
 * all timer values.  Future log types can be added by calling
 * sendRemoteLog("my_event") from any WiFi-connected context.
 *
 * Server endpoint (logserver/log_server.py):
 *   POST OTA_LOG_URL  — X-Api-Key: OTA_LOG_API_KEY  — body: JSON
 *
 * Failure is non-fatal: the function logs to Serial and returns false;
 * the caller (otaTask) continues normally regardless.
 */

#ifndef REMOTE_LOG_H
#define REMOTE_LOG_H

#include <Arduino.h>

/**
 * @brief POST a telemetry JSON snapshot to OTA_LOG_URL.
 *        WiFi must already be connected when this is called.
 *
 * @param event  Short snake_case label for the triggering event.
 *               Defaults to "ota_connect".
 * @return true on HTTP 2xx response, false on any network or server error.
 */
bool sendRemoteLog(const char *event = "ota_connect");

#endif // REMOTE_LOG_H
