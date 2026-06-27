/**
 * @file remote_log.cpp
 * @brief Remote telemetry logging implementation.
 *
 * JSON payload structure (all fields present on every call):
 * {
 *   "event":       "ota_connect",          // caller-supplied label
 *   "fw_version":  "2.2.0",               // FIRMWARE_VERSION
 *   "uptime_ms":   123456,                 // millis() at send time
 *   "settings": {
 *     "idle_dev":            0.12,         // power pause sensitivity band (PSI)
 *     "power_pause_sec":     60,           // power pause timeout (seconds)
 *     "beeper_enabled":      true,
 *     "display_units":       "imperial",   // "imperial" | "metric"
 *     "light_theme":         false,
 *     "pp_sensitivity_pct":  100,          // spike threshold multiplier (%)
 *     "pid_kp":  5.0,
 *     "pid_ki":  3.5,
 *     "pid_kd":  1.6
 *   },
 *   "timers": {
 *     "filter_hours_tenths": 1234,         // resettable filter hour meter
 *     "system_hours_tenths": 5678,         // lifetime non-resettable hours
 *     "job_hours_tenths":    90            // current job timer
 *   }
 * }
 *
 * The server stamps "_received_at" before persisting; the device does not
 * need a real-time clock.
 *
 * To add a new field in the future: extend the JsonDocument population
 * below — no changes required to the server or this header.
 */

#include "remote_log.h"
#include "config.h"
#include "storage.h"
#include "job_timer.h"
#include "dual_core_motor.h"

#include <WiFiClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

bool sendRemoteLog(const char *event)
{
    WiFiClient client;
    HTTPClient http;

    Serial.printf("[LOG] Posting telemetry: event='%s'  url=%s\n", event, OTA_LOG_URL);

    if (!http.begin(client, OTA_LOG_URL)) {
        Serial.println("[LOG] Failed to initialise HTTP client");
        return false;
    }

    http.setTimeout(8000);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-Api-Key",    OTA_LOG_API_KEY);

    // -------------------------------------------------------------------------
    // Build JSON payload
    // -------------------------------------------------------------------------
    JsonDocument doc;

    doc["event"]      = event;
    doc["fw_version"] = FIRMWARE_VERSION;
    doc["uptime_ms"]  = (uint32_t)millis();

    // --- Settings ------------------------------------------------------------
    JsonObject settings = doc["settings"].to<JsonObject>();
    settings["idle_dev"]            = settingsIdleDev;
    settings["power_pause_sec"]     = powerPauseSeconds;
    settings["beeper_enabled"]      = powerPauseBeeperEnabled;
    settings["display_units"]       = (displayUnits == UNITS_IMPERIAL) ? "imperial" : "metric";
    settings["light_theme"]         = lightThemeEnabled;
    settings["pp_sensitivity_pct"]  = powerPauseSensitivityPct;

    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    getPidGainsSafe(&kp, &ki, &kd);
    settings["pid_kp"] = kp;
    settings["pid_ki"] = ki;
    settings["pid_kd"] = kd;

    // --- Timers --------------------------------------------------------------
    JsonObject timers = doc["timers"].to<JsonObject>();
    timers["filter_hours_tenths"] = totalRuntimeTenths;
    timers["system_hours_tenths"] = totalSystemTimeTenths;
    timers["job_hours_tenths"]    = totalJobTimeTenths;

    // -------------------------------------------------------------------------
    // Serialise and POST
    // -------------------------------------------------------------------------
    String body;
    serializeJson(doc, body);

    int httpCode = http.POST(body);
    http.end();

    if (httpCode >= 200 && httpCode < 300) {
        Serial.printf("[LOG] Telemetry accepted (%d)\n", httpCode);
        return true;
    }

    Serial.printf("[LOG] Telemetry rejected (%d) — OTA will continue\n", httpCode);
    return false;
}
