/**
 * @file ota_update.cpp
 * @brief OTA firmware update implementation.
 *
 * Version check and firmware download use a plain JSON manifest file hosted
 * on any HTTP (or HTTPS) server, removing the dependency on the GitHub API
 * and the IDF certificate bundle.
 *
 * Manifest format (manifest.json):
 *   { "version": "2.1.0", "url": "http://192.168.1.x:8080/firmware.bin" }
 *
 * For local testing:
 *   1. Build:   pio run
 *   2. Copy:    cp .pio/build/esp32-s3-devkitc-1/firmware.bin ./firmware.bin
 *   3. Serve:   python3 -m http.server 8080
 *   4. Edit manifest.json with your PC's LAN IP and bump the version string.
 *   5. Set OTA_MANIFEST_URL in config.h to http://<pc-ip>:8080/manifest.json
 *
 * Rollback model (unchanged):
 *   An NVS flag ("ota_state" / "pending") is written to 1 before the post-OTA
 *   restart.  The startup code shows a 60-second confirmation popup on the
 *   first boot after an update.
 */

#include "ota_update.h"
#include "config.h"
#include "dual_core_motor.h"
#include "remote_log.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "nvs.h"

// ---------------------------------------------------------------------------
// Public status struct
// ---------------------------------------------------------------------------
OtaStatus otaStatus = {};

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------
static char            s_ssid[64]         = {};
static char            s_pass[64]         = {};
static volatile bool   s_credsReceived    = false;
static DNSServer       s_dnsServer;
static WebServer       s_webServer(OTA_PORTAL_PORT);
static TaskHandle_t    s_otaTaskHandle    = nullptr;

// ---------------------------------------------------------------------------
// Captive portal HTML (stored in flash)
// ---------------------------------------------------------------------------
static const char HTML_FORM[] PROGMEM = R"html(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta charset="UTF-8">
  <title>Apollo - Firmware Update</title>
  <style>
    body{font-family:Arial,sans-serif;background:#111;color:#fff;
         max-width:420px;margin:40px auto;padding:20px}
    h1{color:#e02020;text-align:center;font-size:1.5em;margin-bottom:6px}
    p{color:#bbb;font-size:.9em;text-align:center;margin:4px 0 18px}
    label{display:block;margin-bottom:4px;color:#ccc;font-size:.95em}
    input{width:100%;padding:12px;margin-bottom:14px;border-radius:6px;
          border:1px solid #444;background:#222;color:#fff;
          font-size:1em;box-sizing:border-box}
    button{width:100%;padding:14px;background:#e02020;color:#fff;
           border:none;border-radius:6px;font-size:1.1em;
           cursor:pointer;font-weight:bold}
    button:active{background:#b01010}
  </style>
</head>
<body>
  <h1>Apollo Firmware Update</h1>
  <p>Enter your home WiFi credentials to check for updates.</p>
  <form action="/connect" method="POST">
    <label>WiFi Network Name (SSID)</label>
    <input type="text"     name="ssid" autocomplete="username"
           placeholder="Network name" required>
    <label>Password</label>
    <input type="password" name="pass" autocomplete="current-password"
           placeholder="Leave blank if open network">
    <button type="submit">Connect &amp; Check Update</button>
  </form>
</body>
</html>
)html";

static const char HTML_OK[] PROGMEM = R"html(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Apollo - Connecting</title>
  <style>
    body{font-family:Arial,sans-serif;background:#111;color:#fff;
         text-align:center;padding:60px 20px}
  </style>
</head>
<body>
  <h2 style="color:#4f4">Connecting&#8230;</h2>
  <p style="color:#bbb">
    Credentials received.<br>
    Close this page and watch your Apollo display for progress.
  </p>
</body>
</html>
)html";

// ---------------------------------------------------------------------------
// Semantic version comparison
//   Returns: -1 if a < b, 0 if equal, 1 if a > b
//   Handles optional leading 'v' / 'V'.
// ---------------------------------------------------------------------------
static int compareVersions(const char *a, const char *b)
{
    if (*a == 'v' || *a == 'V') a++;
    if (*b == 'v' || *b == 'V') b++;
    int aMaj = 0, aMin = 0, aPat = 0;
    int bMaj = 0, bMin = 0, bPat = 0;
    sscanf(a, "%d.%d.%d", &aMaj, &aMin, &aPat);
    sscanf(b, "%d.%d.%d", &bMaj, &bMin, &bPat);
    if (aMaj != bMaj) return (aMaj > bMaj) ? 1 : -1;
    if (aMin != bMin) return (aMin > bMin) ? 1 : -1;
    if (aPat != bPat) return (aPat > bPat) ? 1 : -1;
    return 0;
}

// ---------------------------------------------------------------------------
// NVS helpers for rollback flag
// ---------------------------------------------------------------------------
#define OTA_NVS_NAMESPACE  "ota_state"
#define OTA_NVS_KEY        "pending"

static void nvsSetPending(uint8_t value)
{
    nvs_handle_t h;
    if (nvs_open(OTA_NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, OTA_NVS_KEY, value);
        nvs_commit(h);
        nvs_close(h);
    }
}

// ---------------------------------------------------------------------------
// Captive portal web server setup
// ---------------------------------------------------------------------------
static void startCaptivePortal()
{
    s_webServer.onNotFound([]() {
        s_webServer.sendHeader("Location", "http://192.168.4.1/");
        s_webServer.send(302, "text/plain", "");
    });
    s_webServer.on("/hotspot-detect.html", HTTP_GET, []() {
        s_webServer.sendHeader("Location", "http://192.168.4.1/");
        s_webServer.send(302, "text/plain", "");
    });
    s_webServer.on("/generate_204", HTTP_GET, []() {
        s_webServer.sendHeader("Location", "http://192.168.4.1/");
        s_webServer.send(302, "text/plain", "");
    });
    s_webServer.on("/ncsi.txt", HTTP_GET, []() {
        s_webServer.sendHeader("Location", "http://192.168.4.1/");
        s_webServer.send(302, "text/plain", "");
    });
    s_webServer.on("/", HTTP_GET, []() {
        s_webServer.send_P(200, "text/html", HTML_FORM);
    });
    s_webServer.on("/connect", HTTP_POST, []() {
        s_webServer.arg("ssid").toCharArray(s_ssid, sizeof(s_ssid));
        s_webServer.arg("pass").toCharArray(s_pass, sizeof(s_pass));
        s_webServer.send_P(200, "text/html", HTML_OK);
        s_credsReceived = true;
    });
    s_webServer.begin();
    s_dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    s_dnsServer.start(OTA_DNS_PORT, "*", IPAddress(192, 168, 4, 1));
}

static void stopCaptivePortal()
{
    s_dnsServer.stop();
    s_webServer.stop();
}

// ---------------------------------------------------------------------------
// Fetch and parse manifest.json
// Returns true and populates otaStatus fields on success.
// ---------------------------------------------------------------------------
static bool fetchManifest()
{
    WiFiClient client;
    HTTPClient http;

    Serial.printf("[OTA] Fetching manifest: %s\n", OTA_MANIFEST_URL);

    if (!http.begin(client, OTA_MANIFEST_URL)) {
        strlcpy(otaStatus.errorMessage, "Failed to init HTTP for manifest",
                sizeof(otaStatus.errorMessage));
        otaStatus.state = OTA_STATE_FAILED;
        return false;
    }

    http.setTimeout(10000);
    http.setUserAgent("Apollo-HVLP/" FIRMWARE_VERSION);

    int httpCode = http.GET();
    if (httpCode != HTTP_CODE_OK) {
        snprintf(otaStatus.errorMessage, sizeof(otaStatus.errorMessage),
                 "Manifest HTTP error: %d", httpCode);
        http.end();
        otaStatus.state = OTA_STATE_FAILED;
        return false;
    }

    String payload = http.getString();
    http.end();

    Serial.printf("[OTA] Manifest payload (%u bytes): %s\n",
                  payload.length(), payload.c_str());

    // Parse JSON — only two small fields needed
    JsonDocument filter;
    filter["version"] = true;
    filter["url"]     = true;

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload,
                                               DeserializationOption::Filter(filter));
    if (err) {
        snprintf(otaStatus.errorMessage, sizeof(otaStatus.errorMessage),
                 "Manifest JSON error: %s", err.c_str());
        otaStatus.state = OTA_STATE_FAILED;
        return false;
    }

    const char *ver = doc["version"];
    const char *url = doc["url"];

    if (!ver || !url) {
        strlcpy(otaStatus.errorMessage, "Manifest missing 'version' or 'url'",
                sizeof(otaStatus.errorMessage));
        otaStatus.state = OTA_STATE_FAILED;
        return false;
    }

    strlcpy(otaStatus.latestVersion, ver, sizeof(otaStatus.latestVersion));
    strlcpy(otaStatus.downloadUrl,   url, sizeof(otaStatus.downloadUrl));

    if (compareVersions(otaStatus.latestVersion, FIRMWARE_VERSION) <= 0) {
        otaStatus.state = OTA_STATE_VERSION_CURRENT;
    } else {
        otaStatus.state = OTA_STATE_UPDATE_AVAILABLE;
    }

    Serial.printf("[OTA] Installed: %s  Manifest: %s  -> %s\n",
                  FIRMWARE_VERSION, otaStatus.latestVersion,
                  otaStatus.state == OTA_STATE_VERSION_CURRENT ? "up to date" : "UPDATE AVAILABLE");
    return true;
}

// ---------------------------------------------------------------------------
// Download and flash via HTTPUpdate
// ---------------------------------------------------------------------------
static bool downloadAndFlash()
{
    otaStatus.downloadProgress = 0;

    // Progress callback — called by HTTPUpdate during download
    httpUpdate.onProgress([](int current, int total) {
        if (total > 0) {
            otaStatus.downloadProgress = (current * 100) / total;
        }
    });

    // Disable automatic reboot so we can set the rollback flag first
    httpUpdate.rebootOnUpdate(false);

    WiFiClient client;
    t_httpUpdate_return result = httpUpdate.update(client, otaStatus.downloadUrl);

    switch (result) {
        case HTTP_UPDATE_OK:
            otaStatus.downloadProgress = 100;
            Serial.println("[OTA] HTTPUpdate OK");
            return true;

        case HTTP_UPDATE_FAILED:
            snprintf(otaStatus.errorMessage, sizeof(otaStatus.errorMessage),
                     "Download failed (%d): %s",
                     httpUpdate.getLastError(),
                     httpUpdate.getLastErrorString().c_str());
            Serial.printf("[OTA] %s\n", otaStatus.errorMessage);
            otaStatus.state = OTA_STATE_FAILED;
            return false;

        case HTTP_UPDATE_NO_UPDATES:
            strlcpy(otaStatus.errorMessage, "Server reported no update",
                    sizeof(otaStatus.errorMessage));
            otaStatus.state = OTA_STATE_FAILED;
            return false;

        default:
            strlcpy(otaStatus.errorMessage, "Unknown HTTPUpdate result",
                    sizeof(otaStatus.errorMessage));
            otaStatus.state = OTA_STATE_FAILED;
            return false;
    }
}

// ---------------------------------------------------------------------------
// Main OTA background task
// ---------------------------------------------------------------------------
static void otaTask(void *param)
{
    (void)param;

    // ---- Phase 1: Start captive portal AP ----
    otaStatus.state = OTA_STATE_STARTING_AP;
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(OTA_AP_SSID, nullptr, OTA_AP_CHANNEL);
    startCaptivePortal();
    Serial.printf("[OTA] AP started: SSID='%s'  IP=%s\n",
                  OTA_AP_SSID, WiFi.softAPIP().toString().c_str());

    otaStatus.state = OTA_STATE_WAITING_CREDS;
    s_credsReceived = false;

    // ---- Phase 2: Poll until credentials arrive or cancel ----
    while (!s_credsReceived && !otaStatus.cancelRequested) {
        s_dnsServer.processNextRequest();
        s_webServer.handleClient();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    stopCaptivePortal();

    if (otaStatus.cancelRequested) {
        Serial.println("[OTA] Cancelled during captive portal");
        goto cleanup;
    }

    // ---- Phase 3: Connect to user's WiFi ----
    otaStatus.state = OTA_STATE_CONNECTING_STA;
    Serial.printf("[OTA] Connecting STA: SSID='%s'\n", s_ssid);

    WiFi.begin(s_ssid, s_pass[0] ? s_pass : nullptr);
    {
        unsigned long deadline = millis() + OTA_WIFI_TIMEOUT_MS;
        while (WiFi.status() != WL_CONNECTED && millis() < deadline &&
               !otaStatus.cancelRequested) {
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }

    if (otaStatus.cancelRequested || WiFi.status() != WL_CONNECTED) {
        if (!otaStatus.cancelRequested) {
            strlcpy(otaStatus.errorMessage, "WiFi join failed (check SSID/password)",
                    sizeof(otaStatus.errorMessage));
            otaStatus.state = OTA_STATE_FAILED;
        }
        Serial.println("[OTA] WiFi STA connection failed");
        goto cleanup;
    }
    Serial.printf("[OTA] WiFi connected: IP=%s\n", WiFi.localIP().toString().c_str());

    // ---- Phase 3.5: Send remote telemetry log (fire-and-forget) ----
    sendRemoteLog("ota_connect");

    // ---- Phase 4: Fetch manifest ----
    otaStatus.state = OTA_STATE_CHECKING_VERSION;
    if (!fetchManifest()) {
        goto cleanup;
    }

    if (otaStatus.state == OTA_STATE_VERSION_CURRENT) {
        // Don't immediately exit — wait for the user to choose Return or Reinstall.
        while (!otaStatus.cancelRequested && !otaStatus.forceInstallRequested) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        if (otaStatus.cancelRequested) goto cleanup;
        // User requested force-reinstall: proceed with download of current version.
        otaStatus.state         = OTA_STATE_UPDATE_AVAILABLE;
        otaStatus.updateConfirmed = true;
    }

    // ---- Phase 5: Wait for user confirmation ----
    while (!otaStatus.updateConfirmed && !otaStatus.cancelRequested) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    if (otaStatus.cancelRequested) {
        Serial.println("[OTA] Cancelled before download");
        goto cleanup;
    }

    // ---- Phase 6: Download and flash ----
    // Suspend the motor task so Core 0 is fully available to the WiFi stack.
    // Motor is already disabled (speed=0) at this point.
    otaStatus.state = OTA_STATE_DOWNLOADING;
    {
        TaskHandle_t motorTask = getMotorTaskHandle();
        if (motorTask) vTaskSuspend(motorTask);
        bool flashOk = downloadAndFlash();
        if (motorTask) vTaskResume(motorTask);
        if (!flashOk) goto cleanup;
    }

    // ---- Phase 7: Set rollback flag and reboot ----
    Serial.println("[OTA] Flash complete - setting rollback flag, rebooting");
    nvsSetPending(1);
    otaStatus.state = OTA_STATE_SUCCESS;
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    esp_restart();

cleanup:
    WiFi.softAPdisconnect(true);          // Explicitly tear down the AP first
    vTaskDelay(200 / portTICK_PERIOD_MS);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Block until WiFi stack fully unloads
                                           // before s_otaTaskHandle is cleared.
                                           // Without this, a rapid otaStart() re-entry
                                           // races esp_wifi_init() against the async
                                           // teardown and fails with ESP_ERR_WIFI_INIT_STATE.

    // Set CANCELLED unless we ended in a more specific terminal state
    // (FAILED or VERSION_CURRENT should be shown to the user as-is).
    // Note: cancelRequested being true does NOT prevent CANCELLED — it is
    // precisely why we arrive here when the user presses the cancel button.
    if (otaStatus.state != OTA_STATE_FAILED &&
        otaStatus.state != OTA_STATE_VERSION_CURRENT) {
        otaStatus.state = OTA_STATE_CANCELLED;
    }

    s_otaTaskHandle = nullptr;
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void otaInit(void)
{
    memset(&otaStatus, 0, sizeof(otaStatus));
    otaStatus.state = OTA_STATE_IDLE;
}

void otaStart(void)
{
    if (s_otaTaskHandle != nullptr) return;

    memset(&otaStatus, 0, sizeof(otaStatus));
    otaStatus.state = OTA_STATE_IDLE;

    xTaskCreatePinnedToCore(
        otaTask, "ota_task",
        8192, nullptr, 1,
        &s_otaTaskHandle, 1
    );
}

void otaCancel(void)
{
    otaStatus.cancelRequested = true;
}

void otaConfirmUpdate(void)
{
    otaStatus.updateConfirmed = true;
}

// ---------------------------------------------------------------------------
// Rollback
// ---------------------------------------------------------------------------

RollbackState otaCheckRollback(void)
{
    nvs_handle_t h;
    if (nvs_open(OTA_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return ROLLBACK_NONE;
    uint8_t pending = 0;
    nvs_get_u8(h, OTA_NVS_KEY, &pending);
    nvs_close(h);
    return (pending == 1) ? ROLLBACK_PENDING : ROLLBACK_NONE;
}

void otaConfirmValid(void)
{
    nvsSetPending(0);
    Serial.println("[OTA] New firmware confirmed valid");
}

void otaRollbackNow(void)
{
    nvsSetPending(0);
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *prev    = esp_ota_get_next_update_partition(running);
    if (prev) {
        esp_err_t err = esp_ota_set_boot_partition(prev);
        Serial.printf("[OTA] Rollback to '%s': %s\n",
                      prev->label, err == ESP_OK ? "OK" : "FAILED");
    } else {
        Serial.println("[OTA] Rollback: no previous partition found");
    }
    esp_restart();
}
