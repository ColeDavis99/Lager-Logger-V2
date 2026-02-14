/*==========================================================================================
  * Using 30 pin ESP32-WROOM Devkit
  * Calculates ambient temperature using a cheap 10k ohm thermistor
  * Real-time rendering of historical and current temp readings to a local webpage
  * Pretty colors and animations
  * 
  * Once thermistor and 10k resistor is wired up, open a browser and type "beer.local"
  * Ensure you're on the same wifi network as the ESP32 (only supports 2.4GHz wifi band)
==========================================================================================*/

#include <Arduino.h>
#include <ArduinoOTA.h> //For Over-The-Air programming (OTA)
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <ESPmDNS.h>

/* --- WiFi Credentials --- */
const char *ssid = "";      // SSID
const char *password = ""; // Password
  
/* --- Server & Dashboard Setup --- */
AsyncWebServer server(80);
ESPDash dashboard(server);

dash::SeparatorCard sep1(dashboard, "");
dash::TemperatureCard cardLiveTemperature(dashboard, "Live");
dash::TemperatureCard cardLastAverageTemp(dashboard, "Last Average");
dash::GenericCard cardCounttown(dashboard, "Countdown");

dash::SeparatorCard sep2(dashboard, "");
dash::BarChart<const char *, float> barTemp(dashboard, "Temp Log - 24 hours");
dash::BarChart<const char *, float> barSwing(dashboard, "Swing Chart - 24 hours");

/* --- Global Configuration --- */
const int PING_DELAY = 1000;                                                    // How quickly the webpage updates in ms
const int NUM_TEMP_SAMPLES = 1200;                                              // "Resolution" of your average temperature in ms (how many samples to take across TOT_TEMP_SAMPLE_RANGE)
const unsigned int TOT_TEMP_SAMPLE_RANGE = 60000;                               // Each bar in the history temp chart will be the average temp across this many ms
const int SINGLE_TEMP_SAMPLE_DELAY = TOT_TEMP_SAMPLE_RANGE / NUM_TEMP_SAMPLES;  // ^...equally spaced out readings that is
const unsigned long MAX_POINTS = 1440;                                          // Max # of bars in barTemp chart before scrolling visual begins

/* --- Storage of Average Temp Readings --- */
char avgLabels[MAX_POINTS][12];
const char *avgXAxis[MAX_POINTS];
float avgYAxis[MAX_POINTS];

/* --- Storage of Swing Temp Readings --- */
char swingLabels[MAX_POINTS][12];
const char *swingXAxis[MAX_POINTS];
float swingYAxis[MAX_POINTS];

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
float runningT_sum = 0.0;                   // Sum of the ongoing average's temp readings so far
unsigned long updateCtr = 0;                // # of average readings have been calculated (when to begin scrolling the barTemp chart instead of appending)
float currentT = 0.0;                       // Stores the latest calculated average


/* --- Temperature Calculation --- */
float CalcTemp(int Vo_val)
{
  float R2 = R1 * (ADC_STEPS / (float)Vo_val - 1.0);
  float logR2 = log(R2);
  float T_kelvin = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  float T_far = ((T_kelvin - 273.15) * 9.0 / 5.0) + 32.0;

  // Custom linear curve calculated to match up to the Inkbird's output
  T_far = ((T_far + 5.2399) / 1.1622);

  return T_far;
}

void setup()
{
  Serial.begin(9600);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false); //High performance mode, fixes mDNS "beer.local" not connecting occasionally
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(2000);
    Serial.println(WiFi.status());
  }

  // Init debug prints
  Serial.println("\nIP Address: " + WiFi.localIP().toString());
  if (!MDNS.begin("beer")){Serial.println("Error starting mDNS");}
  else{Serial.println("mDNS started: http://beer.local");}

  server.begin();
}

void loop()
{
  unsigned long currentMillis = millis();

  // --- STEP 0: Frequent Dashboard Update For Countdown---
  if (currentMillis - lastPingTime >= PING_DELAY)
  {
    lastPingTime = millis();
    if (PING_DELAY * pingctr >= TOT_TEMP_SAMPLE_RANGE){pingctr = 0;} // Reset countdown timer

    int secondsLeft = ((TOT_TEMP_SAMPLE_RANGE - (PING_DELAY * pingctr)) / 1000);
    char countBuf[12];               // Temporary buffer for this block
    itoa(secondsLeft, countBuf, 10); // Convert number to string
    strcat(countBuf, "s");           // Add the 's' unit
    cardCounttown.setValue(countBuf);    // Send raw char array
    pingctr++;
    dashboard.sendUpdates();         //Render changes on webpage
  }

  // --- STEP 1: Non-Blocking Sampling ---
  if (currentMillis - lastSampleTime >= SINGLE_TEMP_SAMPLE_DELAY)
  {
    int Vo = analogRead(ThermistorPin);
    runningT_sum += CalcTemp(Vo);
    sampleCtr++;
    cardLiveTemperature.setValue(runningT_sum / sampleCtr);
    
    lastSampleTime = currentMillis;

    // --- STEP 2: Process Average & Update Charts ---
    if (sampleCtr >= NUM_TEMP_SAMPLES)
    {
      currentT = runningT_sum / NUM_TEMP_SAMPLES;

      // Reset for next batch
      runningT_sum = 0;
      sampleCtr = 0;

      // Update Cards
      cardLiveTemperature.setValue(currentT);
      cardLastAverageTemp.setValue(currentT);

      updateCtr++;
      // Update Chart (Rolling Buffer Logic)
      if (updateCtr <= MAX_POINTS)
      {
        // Avg Chart
        int index = updateCtr - 1;
        ltoa(updateCtr, avgLabels[index], 10);
        avgXAxis[index] = avgLabels[index];
        avgYAxis[index] = currentT;
        barTemp.setX(avgXAxis, updateCtr);
        barTemp.setY(avgYAxis, updateCtr);

        // Swing Chart
        ltoa(updateCtr, swingLabels[index], 10);
        swingXAxis[index] = swingLabels[index];
        swingYAxis[index] = currentT-avgYAxis[index - (updateCtr>1)]; //Can't have a swing temp with only 1 reading.
        barSwing.setX(swingXAxis, updateCtr);
        barSwing.setY(swingYAxis, updateCtr);
      }
      else
      {
        // Shift data left
        for (int i = 0; i < MAX_POINTS - 1; i++)
        {
          // Avg chart
          strcpy(avgLabels[i], avgLabels[i+1]);
          avgYAxis[i] = avgYAxis[i+1];
          avgXAxis[i] = avgLabels[i];

          // Swing chart
          strcpy(swingLabels[i], swingLabels[i+1]);
          swingYAxis[i] = swingYAxis[i+1];
          swingXAxis[i] = avgLabels[i];
        }
        // Add new data to end
        // Avg chart
        ltoa(updateCtr, avgLabels[MAX_POINTS-1], 10);
        avgYAxis[MAX_POINTS-1] = currentT;
        avgXAxis[MAX_POINTS-1] = avgLabels[MAX_POINTS-1];
        barTemp.setX(avgXAxis, MAX_POINTS);
        barTemp.setY(avgYAxis, MAX_POINTS);

        // Swing chart
        ltoa(updateCtr, swingLabels[MAX_POINTS-1], 10);
        swingYAxis[MAX_POINTS-1] = currentT-avgYAxis[MAX_POINTS-2];
        swingXAxis[MAX_POINTS-1] = swingLabels[MAX_POINTS-1];
        barSwing.setX(swingXAxis, MAX_POINTS);
        barSwing.setY(swingYAxis, MAX_POINTS);
      }

      Serial.print("Avg Temp: ");
      Serial.println(currentT);
    }
  }

  // No delay() here! The ESP32 is free to handle web requests instantly.

}
