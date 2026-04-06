#include <Arduino.h>
#include <ArduinoOTA.h> //Over-the-air programming
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "ESPDashboardPlus.h"
#include "dashboard_html.h"

AsyncWebServer server(80);
ESPDashboardPlus dashboard("Lager Logger");

/* --- WiFi Credentials --- */
const char *ssid = "";
const char *password = "";

void setup()
{
    Serial.begin(115200);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
        delay(500);

    // Init OTA listener
    ArduinoOTA.begin();

    // Disable OTA tab, enable Console tab
    dashboard.begin(&server, DASHBOARD_HTML_DATA, DASHBOARD_HTML_SIZE, false, true);

    // Set firmware version info (displayed in OTA tab)
    // dashboard.setVersionInfo("1.0.2", "2026-04-03");

    server.begin();
}

void loop()
{   
    // OTA and UI heartbeats
    ArduinoOTA.handle();
    dashboard.loop();

    //
}