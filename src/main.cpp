#include <Arduino.h>
#include <ArduinoOTA.h> //Over-the-air programming
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>  // Allows for "beer.local" as the IP
#include <LittleFS.h> // Little File System for storing temp readings in FLASH memory
#include "ESPDashboardPlus.h"
#include "dashboard_html.h"

/* --- WiFi Credentials --- */
const char *ssid = "";
const char *password = "";

/* --- Server & Dashboard Setup --- */
AsyncWebServer server(80);
ESPDashboardPlus dashboard("Lager Logger");

/* --- Init minute history chart card --- */
ChartCard *minute_history = dashboard.addChartCard(
    "minute_history", // ID
    "Temp by Minute", // Title
    ChartType::AREA,  // Chart type
    30                // Keep 30 data points
);

StatCard *countdown = dashboard.addStatCard(
    "countdown",        // ID
    "Minute Countdown", // Title
    "59",               // Initial value
    "s"                 // Unit
);

StatCard *liveTemp = dashboard.addStatCard(
    "liveTemp",        // ID
    "Live Temp", // Title
    "0.0",               // Initial value
    "F"                 // Unit
);

/* --- Global Configuration --- */
const int PING_DELAY = 1000;                                                    // How quickly the webpage updates in ms
const int NUM_TEMP_SAMPLES = 600;                                               // "Resolution" of your average temperature in ms (how many samples to take across TOT_TEMP_SAMPLE_RANGE)
const unsigned int TOT_TEMP_SAMPLE_RANGE = 60000;                               // Each bar in the history temp chart will be the average temp across this many ms
const int SINGLE_TEMP_SAMPLE_DELAY = TOT_TEMP_SAMPLE_RANGE / NUM_TEMP_SAMPLES;  // ^...equally spaced out readings that is
const long MAX_POINTS = 180;                                                    // Max # of bars in barTemp chart (fine-grain) before scrolling visual begins

const long AVERAGES_PER_HOUR = 3600000/TOT_TEMP_SAMPLE_RANGE;                   // How many finished averages will there be in one hour (3,600,000 ms in an hour)
const unsigned int TIMESPAN = 24 * 42;                                          // How many hours of data to show in the ALL charts (24 hours times 42 days) Beginning to run into RAM limits
const bool WRITE_TO_CSV = true;                                                // Enable/disable .csv writes
const bool DELETE_CSV = true;                                                  // Enable/disable deletion of the .csv file listed in FILENAME var
const char *FILENAME = "/hourly.csv";

/* --- Storage of Average Temp Readings --- */
char avgLabels[MAX_POINTS][12];
const char *avgXAxis[MAX_POINTS];
float avgYAxis[MAX_POINTS];

char avgLabelsAll[TIMESPAN][12];
const char *avgXAxisAll[TIMESPAN];
float avgYAxisAll[TIMESPAN];

/* --- Storage of Swing Temp Readings --- */
char swingLabels[MAX_POINTS][12];
const char *swingXAxis[MAX_POINTS]; 
float swingYAxis[MAX_POINTS];

char swingLabelsAll[TIMESPAN][12];
const char *swingXAxisAll[TIMESPAN];
float swingYAxisAll[TIMESPAN];

/* --- Pin & Sensor Config --- */
const int ThermistorPin = 34;
const float R1 = 6900;
const float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
const float V_REF = 3.3;
const float R_BITS = 12.0;
const float ADC_STEPS = 4095.0; // (2^12 - 1)

/* --- Non-Blocking Timing Variables --- */
int pingctr = 0;                          // For the "seconds till next average" card value
unsigned long lastPingTime = 0;           // For refreshing the dashboard
unsigned long lastSampleTime = 0;         // For spacing out temp readings for the average
int sampleCtr = 0;                        // The number of temperature samples the next bar of barTemp chart has had done

/* --- Temperature Averaging Variables --- */
float liveT = 0.0;                        // Current temp reading
float runningT_sum = 0.0;                 // Sum of the ongoing average's temp readings so far
long updateCtr = 0;                       // # of average readings have been calculated (when to begin scrolling the barTemp chart instead of appending)
float currentT = 0.0;                     // Stores the latest calculated average
float minTempAll = 999.0;                 // Lowest average temp measured
float maxTempAll = -999.0;                // Highest average temp measured


/* --- Temperature Calculation --- */
float CalcTemp(int Vo_val)
{
  float R2 = R1 * (ADC_STEPS / (float)Vo_val - 1.0);
  float logR2 = log(R2);
  float T_kelvin = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  float T_far = ((T_kelvin - 273.15) * 9.0 / 5.0) + 32.0;

  // Custom curve calculated to match up to the Inkbird's output
  // T_far = (-0.0000223 * T_far * T_far * T_far) + (0.00564 * T_far * T_far) + (0.392 * T_far) + 17.08; // White Blue V2 (inverse equation)

  T_far = (0.0000140 * T_far * T_far * T_far) - (0.00178 * T_far * T_far) + (0.871 * T_far) + 7.87;// Red Blue V2 (inverse equation)

  // T_far = (-0.00005946 * T_far * T_far * T_far) + (0.01472 * T_far * T_far) - (0.3119 * T_far) + 32.0; // Brown white V2 (inverse equation)

  return T_far;
}

float FindMin(float temps[], const long size, long numReadings)
{
  long scanLimit = (numReadings > size) ? size : numReadings;
  float min = temps[0];

  for (int i = 0; i < scanLimit; i++)
  {
    if (temps[i] < min)
      min = temps[i];
  }

  return min;
}

float FindMax(float temps[], const long size, long numReadings)
{
  long scanLimit = (numReadings > size) ? size : numReadings;
  float max = temps[0];

  for (int i = 0; i < scanLimit; i++)
  {
    if (temps[i] > max)
      max = temps[i];
  }

  return max;
}

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

    server.begin();

    // Apply UI theme to all cards
    countdown->setVariant(CardVariant::PRIMARY);
    liveTemp->setVariant(CardVariant::PRIMARY);
    minute_history->setVariant(CardVariant::PRIMARY);
}

void loop()
{   
    // OTA and UI heartbeats
    ArduinoOTA.handle();
    dashboard.loop();

    // Main timing variable
    unsigned long currentMillis = millis();

    // --- STEP 0: Non-Blocking Temp Sampling ---
    if (currentMillis - lastSampleTime >= SINGLE_TEMP_SAMPLE_DELAY)
    {
        int Vo = analogRead(ThermistorPin);
        currentT = CalcTemp(Vo);
        runningT_sum += currentT;
        sampleCtr++;

        char liveBuf[12];                     // Temporary buffer for this block
        dtostrf(currentT, 0, 2, liveBuf);     // Convert float to string
        lastSampleTime = currentMillis;

        // --- STEP 1: Frequent Dashboard Update For Countdown---
        if (currentMillis - lastPingTime >= PING_DELAY)
        {
            lastPingTime = millis();
            if (PING_DELAY * pingctr >= TOT_TEMP_SAMPLE_RANGE){pingctr = 0;} // Reset countdown timer
    
            int secondsLeft = ((TOT_TEMP_SAMPLE_RANGE - (PING_DELAY * pingctr)) / 1000);
            char countBuf[12];               // Temporary buffer for this block
            itoa(secondsLeft, countBuf, 10); // Convert number to string
    
            // Update the cards
            dashboard.updateStatCard("liveTemp", liveBuf);
            dashboard.updateStatCard("countdown", countBuf);
    
            pingctr++;
        }
    }

}