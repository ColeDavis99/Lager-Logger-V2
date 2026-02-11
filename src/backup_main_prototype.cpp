// #include <Arduino.h>
// #include <WiFi.h>
// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
// #include <ESPDash.h>
// #include <ESPmDNS.h>

// /* --- WiFi Credentials --- */
// const char *ssid = "";     // SSID
// const char *password = ""; // Password

// AsyncWebServer server(80);
// ESPDash dashboard(server);

// /* --- Global Configuration --- */
// const unsigned long MAX_POINTS = 2880; // Max # of bars in chart before scrolling visual begins
// char labels[MAX_POINTS][12];
// const char *XAxis[MAX_POINTS];
// float YAxis[MAX_POINTS];

// /* --- Dashboard Cards --- */
// dash::TemperatureCard temperature(dashboard, "Live Temp");
// dash::GenericCard countdown(dashboard, "Seconds until next average");
// dash::TemperatureCard lastAverageTemp(dashboard, "Latest average temp");
// dash::BarChart<const char *, float> bar(dashboard, "Temp Log (48 hours)");

// /* --- Pin & Sensor Config --- */
// const int ThermistorPin = 34;
// float R1 = 6900;
// float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
// const float V_REF = 3.3;
// const float R_BITS = 12.0;
// const float ADC_STEPS = 4095.0; // (2^12 - 1)

// /* --- Non-Blocking Timing Variables --- */
// unsigned long lastSampleTime = 0; // For spacing out temp reading for the average
// unsigned long lastPingTime = 0;   // For refreshing the dashboard for countdown "pinging" I'm saying
// const int NUM_TEMP_SAMPLES = 1200;
// const unsigned int TOT_TEMP_SAMPLE_RANGE = 60000;
// const int SINGLE_TEMP_SAMPLE_DELAY = TOT_TEMP_SAMPLE_RANGE / NUM_TEMP_SAMPLES;
// const int PING_DELAY = 1000; // How often pinging happens (countdown card)
// int pingctr = 0;

// int sampleCounter = 0;
// float runningT_sum = 0.0;
// unsigned long updateCtr = 0;
// float currentT = 0.0; // Stores the latest calculated average

// /* --- Temperature Calculation --- */
// float CalcTemp(int Vo_val)
// {
//     float R2 = R1 * (ADC_STEPS / (float)Vo_val - 1.0);
//     float logR2 = log(R2);
//     float T_kelvin = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
//     float T_far = ((T_kelvin - 273.15) * 9.0 / 5.0) + 32.0;

//     // Custom linear curve calculated to match up to the Inkbird's output
//     T_far = ((T_far + 5.2399) / 1.1622);

//     return T_far;
// }

// void setup()
// {
//     Serial.begin(115200);

//     WiFi.mode(WIFI_STA);
//     WiFi.begin(ssid, password);
//     while (WiFi.status() != WL_CONNECTED)
//     {
//         delay(500);
//         Serial.print(".");
//     }

//     Serial.println("\nIP Address: " + WiFi.localIP().toString());

//     if (!MDNS.begin("beer"))
//     {
//         Serial.println("Error starting mDNS");
//     }
//     else
//     {
//         Serial.println("mDNS started: http://beer.local");
//     }

//     server.begin();
// }

// void loop()
// {
//     unsigned long currentMillis = millis();

//     // --- STEP 0: Frequent Dashboard Update For Countdown---
//     if (currentMillis - lastPingTime >= PING_DELAY)
//     {
//         lastPingTime = millis();
//         if (PING_DELAY * pingctr >= TOT_TEMP_SAMPLE_RANGE)
//         {
//             pingctr = 0;
//         } // Reset countdown timer

//         int secondsLeft = ((TOT_TEMP_SAMPLE_RANGE - (PING_DELAY * pingctr)) / 1000);
//         char countBuf[12];               // Temporary buffer for this block
//         itoa(secondsLeft, countBuf, 10); // Convert number to string
//         strcat(countBuf, "s");           // Add the 's' unit
//         countdown.setValue(countBuf);    // Send raw char array

//         dashboard.sendUpdates();
//         pingctr++;
//     }

//     // --- STEP 1: Non-Blocking Sampling ---
//     if (currentMillis - lastSampleTime >= SINGLE_TEMP_SAMPLE_DELAY)
//     {
//         lastSampleTime = currentMillis;

//         int Vo = analogRead(ThermistorPin);
//         runningT_sum += CalcTemp(Vo);
//         sampleCounter++;

//         // Render most recent non-averaged temp reading and countdown
//         temperature.setValue(runningT_sum / sampleCounter);

//         // dashboard.sendUpdates();

//         // --- STEP 2: Process Average & Update Dashboard ---
//         if (sampleCounter >= NUM_TEMP_SAMPLES)
//         {
//             currentT = runningT_sum / NUM_TEMP_SAMPLES;

//             // Reset for next batch
//             runningT_sum = 0;
//             sampleCounter = 0;

//             // Update Cards
//             temperature.setValue(currentT);
//             lastAverageTemp.setValue(currentT);

//             updateCtr++;
//             // Update Chart (Rolling Buffer Logic)
//             if (updateCtr <= MAX_POINTS)
//             {
//                 int index = updateCtr - 1;
//                 ltoa(updateCtr, labels[index], 10);
//                 XAxis[index] = labels[index];
//                 YAxis[index] = currentT;
//                 bar.setX(XAxis, updateCtr);
//                 bar.setY(YAxis, updateCtr);
//             }
//             else
//             {
//                 // Shift data left
//                 for (int i = 0; i < MAX_POINTS - 1; i++)
//                 {
//                     strcpy(labels[i], labels[i + 1]);
//                     YAxis[i] = YAxis[i + 1];
//                     XAxis[i] = labels[i];
//                 }
//                 // Add new data to end
//                 ltoa(updateCtr, labels[MAX_POINTS - 1], 10);
//                 YAxis[MAX_POINTS - 1] = currentT;
//                 XAxis[MAX_POINTS - 1] = labels[MAX_POINTS - 1];
//                 bar.setX(XAxis, MAX_POINTS);
//                 bar.setY(YAxis, MAX_POINTS);
//             }

//             // dashboard.sendUpdates();

//             Serial.print("Avg Temp: ");
//             Serial.println(currentT);
//         }
//     }

//     // No delay() here! The ESP32 is free to handle web requests instantly.
// }