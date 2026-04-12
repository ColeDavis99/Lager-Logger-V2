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

/* --- Global Configuration --- */
const int PING_DELAY = 1000;                                                   // How quickly the webpage UI updates in ms
const int NUM_TEMP_SAMPLES = 6000;                                             // "Resolution" of your average temperature in ms (how many samples to take across TOT_TEMP_SAMPLE_RANGE)
const unsigned int TOT_TEMP_SAMPLE_RANGE = 60000;                              // Each bar in the history temp chart will be the average temp across this many ms
const int SINGLE_TEMP_SAMPLE_DELAY = TOT_TEMP_SAMPLE_RANGE / NUM_TEMP_SAMPLES; // ^...equally spaced out readings that is
const long MAX_POINTS = 10080;                                                 // Max # of bars in "Resolution 2, fine-grain charts" before scrolling visual begins

const long AVERAGES_PER_HOUR = 3600000 / TOT_TEMP_SAMPLE_RANGE;                // How many finished averages will there be in one hour (3,600,000 ms in an hour)
const unsigned int TIMESPAN = 24 * 42;                                         // How many hours of data to show in the ALL charts (24 hours times 42 days) Beginning to run into RAM limits
const bool WRITE_TO_CSV = false;                                                // Enable/disable .csv writes
const bool DELETE_CSV = true;                                                  // Enable/disable deletion of the .csv file listed in FILENAME var
const char *FILENAME = "/hourly.csv";

/* --- Init dashboard cards --- */
StatCard *countdown = dashboard.addStatCard("countdown", "Minute Countdown", "59", "sec");
StatCard *liveTemp = dashboard.addStatCard("liveTemp", "Live Temp", "0.0", "F");
StatCard *lastAverage = dashboard.addStatCard("lastAverage", "Last Average", "0.0", "F");
ChartCard *minuteHistory = dashboard.addChartCard("minuteHistory", "7 Day Temp Log - By Minute", ChartType::BAR, MAX_POINTS);

/*
--- Storage of Average Temp Readings ---
char avgLabels[MAX_POINTS][12];
const char *avgXAxis[MAX_POINTS];
float avgYAxis[MAX_POINTS];

char avgLabelsAll[TIMESPAN][12];
const char *avgXAxisAll[TIMESPAN];
float avgYAxisAll[TIMESPAN];

--- Storage of Swing Temp Readings --- 
char swingLabels[MAX_POINTS][12];
const char *swingXAxis[MAX_POINTS]; 
float swingYAxis[MAX_POINTS];

char swingLabelsAll[TIMESPAN][12];
const char *swingXAxisAll[TIMESPAN];
float swingYAxisAll[TIMESPAN];
*/

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

    // Launch server
    server.begin();

    // Define UI look of the cards (grouping & whatnot)
    countdown->setVariant(CardVariant::PRIMARY);
    countdown->setSize(1,1);
    liveTemp->setVariant(CardVariant::PRIMARY);
    liveTemp->setSize(1,1);
    minuteHistory->setVariant(CardVariant::PRIMARY);
    // dashboard.addCardToGroup("TEST_GROUP", "minuteHistory");
    minuteHistory->setSize(1, 2);
    lastAverage->setVariant(CardVariant::PRIMARY);
    lastAverage->setSize(1,1);
}

void loop()
{   
    // OTA and UI heartbeats
    ArduinoOTA.handle();        // Listen for OTA program upload event
    dashboard.loop();           // Process WebSocket events

    // Main timing variable
    unsigned long currentMillis = millis();

    //================================================================================================================================================
    // --- STEP 0: Non-Blocking Temp Sampling ---       (Resolution 0, "fine-grain" timeframe. For high speed temp sampling.)
    //================================================================================================================================================
    if (currentMillis - lastSampleTime >= SINGLE_TEMP_SAMPLE_DELAY)
    {
        int Vo = analogRead(ThermistorPin);
        currentT = CalcTemp(Vo);
        runningT_sum += currentT;
        sampleCtr++;

        char liveBuf[12];                     // Temporary buffer for this block
        dtostrf(currentT, 0, 2, liveBuf);     // Convert float to string
        lastSampleTime = currentMillis;


        //================================================================================================================================================
        // --- STEP 1: Frequent Dashboard Update ---        Resolution 1, "UI update" timeframe. One second is typically what I have it set to.)
        //================================================================================================================================================
        if (currentMillis - lastPingTime >= PING_DELAY)
        {
            lastPingTime = millis();
            if (PING_DELAY * pingctr >= TOT_TEMP_SAMPLE_RANGE){pingctr = 0;} // Reset countdown timer
    
            int secondsLeft = ((TOT_TEMP_SAMPLE_RANGE - (PING_DELAY * pingctr)) / 1000);
            char countBuf[12];               // Temporary buffer for this block
            itoa(secondsLeft, countBuf, 10); // Convert number to string
    
            // Update the "one second" cards
            dashboard.updateStatCard("liveTemp", liveBuf);
            dashboard.updateStatCard("countdown", countBuf);
    
            pingctr++;


            //================================================================================================================================================
            // --- STEP 2: Process Average & Update Charts ---      (Resolution 2, "medium grain" timeframe. One minute is typically what I have it set to.)
            //================================================================================================================================================
            if (sampleCtr >= NUM_TEMP_SAMPLES){
                // Here's this run's average temp (fine-grain)
                currentT = runningT_sum / NUM_TEMP_SAMPLES;

                // Reset/Increment for next batch
                runningT_sum = 0;
                sampleCtr = 0;
                updateCtr++;

                
                // Update the "one minute" cards
                dashboard.updateChartCard("minuteHistory", currentT);
                
                char avgBuf[12];  // Temporary buffer for this block
                snprintf(avgBuf, sizeof(avgBuf), "%.2f", currentT);
                dashboard.updateStatCard("lastAverage", avgBuf);

                // Append to CSV (once per minute)
                // if (WRITE_TO_CSV)
                // {
                //     File file = LittleFS.open(FILENAME, "a");
                //     if (file)
                //     {
                //         file.print(updateCtr);
                //         file.print(",");
                //         file.println(currentT, 3); // 3 decimal places
                //         file.close();
                //         Serial.println("Logged minute temp to CSV");
                //     }
                //     else{Serial.println("Failed to open CSV for appending");}
                // }


/*
                // Update the "all time" cards
                if ((updateCtr % AVERAGES_PER_HOUR) == 0 && (updateCtr / AVERAGES_PER_HOUR) < TIMESPAN)
                {
                    // Avg Chart all-time
                    int hoursElapsed = updateCtr / AVERAGES_PER_HOUR;
                    int index = hoursElapsed - 1;
                    ltoa(hoursElapsed, avgLabelsAll[index], 10);
                    avgXAxisAll[index] = avgLabelsAll[index];
                    
                    // Calculate the hourly average
                    float hourTempSum = 0.0;
                    if(updateCtr > MAX_POINTS){
                    for(int i=MAX_POINTS-1; i>=MAX_POINTS-AVERAGES_PER_HOUR; i--){hourTempSum += avgYAxis[i];}}
                    else{
                    for (int i=((updateCtr-1) % MAX_POINTS); i>((updateCtr - 1) % MAX_POINTS)-AVERAGES_PER_HOUR; i--){hourTempSum += avgYAxis[i];}}
                    avgYAxisAll[index] = hourTempSum/AVERAGES_PER_HOUR;
                    barTempAll.setX(avgXAxisAll, hoursElapsed);
                    barTempAll.setY(avgYAxisAll, hoursElapsed);

                    // Swing Chart all-time
                    ltoa(hoursElapsed, swingLabelsAll[index], 10);
                    swingXAxisAll[index] = swingLabelsAll[index];
                    swingYAxisAll[index] = avgYAxisAll[index] - avgYAxisAll[index - (hoursElapsed > 1)]; // Can't have a swing temp with only 1 reading.
                    barSwingAll.setX(swingXAxisAll, hoursElapsed);
                    barSwingAll.setY(swingYAxisAll, hoursElapsed);
                }
*/

/*

                // Update Cards
                cardLiveTemperature.setValue(currentT);
                cardLastAverageTemp.setValue(currentT);
                cardMinTemp.setValue(FindMin(avgYAxis, MAX_POINTS, updateCtr));
                cardMaxTemp.setValue(FindMax(avgYAxis, MAX_POINTS, updateCtr));

                if(currentT > maxTempAll){maxTempAll = currentT;}
                if(currentT < minTempAll){minTempAll = currentT;}
                cardMinTempAll.setValue(minTempAll);
                cardMaxTempAll.setValue(maxTempAll);
*/

            }// End step 2
        }// End step 1
    }// End step 0
}// End loop()