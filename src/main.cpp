/*
 * sensor_monitor_v1.4
 *
 * MKR1000 - MKR WiFi 1010 - MKR VIDOR 4000 WiFi RTC
 * This sketch asks NTP for the Linux epoch and sets the 
 * internal Arduino MKR1000's RTC accordingly.
 * created 08 Jan 2016
 * by Arturo Guadalupi <a.guadalupi@arduino.cc>
 * modified 26 Sept 2018
 * http://arduino.cc/en/Tutorial/WiFiRTC
 * This code is in the public domain.
 * 
 * 1/10/2021
 * This sketch modifies the WiFi RTC tutorial found at
 * https://www.arduino.cc/en/Tutorial/WiFiRTC
 * to make it more modular and to correct the hours output
 * when using negative GMT timezones.
 * Functions are added to sample sensors.
 * Then it as modified to include the 
 * Arduino - ScheduledWiFiSSLWebClient tutoral found at 
 * https://www.arduino.cc/en/Tutorial/ScheduledWiFiSSLWebClient
 * By Jay Morreale
 *
 * 4/12/2021
 * The MKR WiFI 1010 sensors were replaced with new sensors and a relay. 
 * The sensors, ports, and IO type are shown below. 
 * 
 * A0, INPUT, Gas Sensor (MQ2) V1.5
 * D2, INPUT, PWD, Ultrasonic Distance Sensor v2.0
 * D1, INPUT, PIR Motion Sensor v1.2
 * A1, INPUT, Capacitive Moisture Sensor v1.0
 * D3, INPUT, Piezo Vibration Sensor v1.1
 * D0, OUTPUT, SPDT Relay (30A) v1.0b
 * I2C, OLED Display 96x96 v2.1
 * I2C, High Accuracy Temp & Humidity (SHT35) v1.0
 * I2C, High Precision Barometric Sensor (OPS310) v1.0
 * I2Cm SEK-SCD41 Sensirion C02 sensor
 * 
 * The I2C sensros are on a Grove I2C Hub (6 Port). 
 * A Li-poly 3.7 V 2000 mAh battery is to be attached to the board.
 * The code is to be added.
 * 
 * THe OLED display test code works but may causes the SJT35 to report errors.
 * 
 * 5/6/2021
 * The date and time functions have been rewritten to return a string.
 * 
 * 5/20/2021
 * Udpated all functions to return a value. Values are printed using 
 * sprintf now. Arduino sprintf does not support floating point numbers
 * so worked around them by conconvert floats to int and (int*100)%100. 
 * 
 * Added a variable that can be used to write sensor samples to a Google
 * Sheet using PushingBox and a Google Sript (Javascript) and later
 * optied not to use a third party service like PushingBox.
 * 
 * 5/22/21
 * The Send MKR1000 Data to Google Sheets tutorial seemed complicated and
 * requires a third party service to write data to a Google Sheet so
 * I opted to write data directly to the google sheet using the 
 * Super easy cloud data logging with google sheets example. 
 * https://www.reddit.com/r/arduino/comments/9zsmlv/super_easy_cloud_data_logging_with_google_sheets/
 *  
 * The sensor data string works when it is copied into you browser
 * when logged into Google. Google requires Authentication for writing
 * Google sheets now so Oauth2 needs to be added.
 * 
 * 
 * 6/8/2021
 * Added modified versions of the Sensirion SCD41X rounties.
 * 
 * 2/15/2022
 * Updated to fix changes in libraries
 * Settigns are used configure the SPI library now
 * The DPS310 uses the SPI in one of its classes
 * 
 * 3/21/2022
 * Tried using the DigitalPressureLibrary V1.06 and V1.07 instead of the
 * DPS310-Pressure-Sensor-dps310 library but neither works with the new
 * SPI Arduino Core library. The main error is as follows:
 * D:\jpm\arduino\libraries\DigitalPressureSensor\src\DpsClass.cpp:62:12:
 * error: 'arduino::SPIClass {aka class arduino::HardwareSPI}' 
 * has no member named 'setDataMode' m_spibus->setDataMode(SPI_MODE3);
 * The DpsClass.cpp library was modified to use the new SPI syntax. 
 * with m_spibus->begin(); being replaced with m_spibus->setDataMode(SPI_MODE3);
 * 
 * 7/19/2022
 * The program was converted to run on the VS Code PlatformIO IDE
 * 
 * 7/21/2022
 * A boolean flag was created to select when to write to the
 * Google Sheet
 * 
 * 7/24/2022
 * The example code by TaterTotsForLunch won't log into Google as written
 * as Google has changed the way it authenticates devices and API requests.
 * Google now uses Oauth2 to authenticate users and devices. The code here
 * is to be modified to use the Oauth2.
 * 
 * 7/18/2023
 * The code shown below is open source and is provided as is. The libraries that are
 * copyrighted are the copyright of the respective manufacture and must retain
 * the copyright notice, like the library from Sensirion, for example.  
 * 
 * 
 */
 
#include <string.h>
#include <stdio.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <RTCZero.h>

/*
 * The SPI library seems to have change and is initialized
 * using the SPI.beginTransaction
 * No sensor uses the SPI bus as it is not connected to the
 * carrier board. The DPS310 library was modified be with the new
 * configuration
 * SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0))
 */
#include <SPI.h>

#include "wifi_secrets.h"      // include the ssid and password

#include "Seeed_SHT35.h"       // High Accuracy Temp & Humidity (SHT35) v1.0 sensor library

#include <Dps310.h>           // High Precision Barometric Sensor (OPS310) v1.0 sensor library

#include "Ultrasonic.h"       // Ultrasonic Distance Sensor v2.0

#include "U8g2lib.h"          // OLED Display 96x96 v2.1
#include <Wire.h>

#include <Arduino.h>
#include <SensirionI2CScd4x.h>        // SEK-SCD41 CO2 sensor library

#include <ArduinoHttpClient.h>       // Web client library
/* 
 *  Digital IO definitions
 */
#define DISTANCE_SENSOR 2     // D2, INPUT, PWD, Ultrasonic Distance Sensor v2.0
#define PIR_MOTION_SENSOR 1   // D1, INPUT, PIR Motion Sensor v1.2
#define SPDT_RELAY 0          // D0, OUTPUT, SPDT Relay (30A) v1.0b
#define VIB_SENSOR 3          // D3, INPUT, Piezo Vibration Sensor v1.1
/*
 * Analog sensor pin definitions
 */
#define MQ2_GAS_SENSOR A0     // A0, INPUT, Gas Sensor (MQ2) V1.5
#define MOISTURE_SENSOR A1    // A1, INPUT, Capacitive Moisture Sensor v1.0
/*
 * Define WiFi variables
 */
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;             // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;  // define a variable for the wifi status

WiFiSSLClient wificlt;         // initialize the WiFi client library

/*
 * Define varialbles for calculationg local time for a given timezone.
 */
const int GMT = -4;           // change this to adapt it to your timezone
int diff = 0;                 // difference between gmt hours and GMT
int tz = 0;                   // hours correct for GMT time zone when GMT < 0
int tzhours = 0;              // used to hold the rtc.getHours value

/*
 * Define variables for for the sensors
 * A0, INPUT, Gas Sensor (MQ2) V1.5
 */
 // A0, INPUT, Gas Sensor (MQ2) V1.5
float R0;                     // MQ2 sensor ration RS_air/R0
float Ratio;                  // ratio of MQ2 sample to calibration

struct mq2struct {
  float sensorVoltage;
  float rsgas;
  float ratio;
};
typedef struct mq2struct mq2data;


// High Accuracy Temp & Humidity (SHT35) v1.0 sensor
/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SDAPIN  20
    #define SCLPIN  21
    #define RSTPIN  7
    #define SERIAL SerialUSB
#else
    #define SDAPIN  A4
    #define SCLPIN  A5
    #define RSTPIN  2
    #define SERIAL Serial
#endif

SHT35 sensor(SCLPIN);

// Ultrasonic Distance Sensor v2.0
Ultrasonic ultrasonic(DISTANCE_SENSOR);

// OLED Display 96x96 v2.1
U8G2_SH1107_SEEED_128X128_1_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

// define the High Precision Barometric Sensor (OPS310) v1.0 sensor
 Dps310 Dps310PressureSensor = Dps310();
 
/*
 * Define variables for the server
 */
char server[] = SECRET_API;
bool sendRequest = true;      // used to understand if the http request must be sent

RTCZero rtc;                  // create an RTC object

/*
 * Variables used to transfer sensor measurements to the Google Sheet
 */
static char sensestr[256];    // string for google sheet API string
static char displaystr[256];  // string fof long display string 
static char dl01[30];         // Display line 1: date and time 
static char dl02[30];         // Display line 2: temp1 & Humidity1 from the High Accuracy Temp (SHT35) v1.0
static char dl03[30];         // Display line 3: temp2 & pressure from the High Accuracy Humidity (SHT35) v1.0
static char dl04[30];         // Display line 4: ratio from the Gas Sensor (MQ2) V1.5
static char dl05[30];         // Display line 5: motor status, & motion status from the Piezo Vibration Sensor v1.1, and the PIR Motion Sensor v1.2
static char dl06[30];         // Display line 6: Moisture from the Capacitive Moisture Sensor v1.0
static char dl07[30];         // Display line 7: Distance from the Ultrasonic Distance Sensor v2.0
static char dl08[30];         // Display line 8: CO2 the SEK-SCD41 Sensirion C02 sensor
static char dl09[30];         // Display line 9: Temp3 and Humidity2 the SEK-SCD41 Sensirion C02 sensor

static char shdate[10];       // String for the data to be sent to the Google Sheet
static char shtime[11];       // Sting for the time to be sent to the Google Sheet
/*
 * variable definitions for the values to be written to the Googl Sheet.
 */
float temp1;
float humidity;
float temp2;
float pressure;
bool motorStatus;
bool motionStatus;
int waterLevel;
long depth;

/*
* Set true to send data to the Google datasheet
*/
bool sendtoGoolgle = false;

/*
 * Google Script API
 */
const String logURL = SECRET_API; // API URL
int statusCode;
String responsex;

char serverAddress[] = SECRET_API;  // authentication server URL
int port = 443;

/*
 * SDC4x C02 Sensor
 * Create a structure to return the CO2 level, temperature, and
 * humidity
 */
SensirionI2CScd4x scd4x;
struct co2struct {
  uint16_t co2;
  float temp3;
  float hum2;
};
typedef struct co2struct co2data;
bool ifLowPower = true;
float toffset;
uint16_t co2expected = 2000;

/*************************************Function Declariations***********************************
*
*/
void writeSheet(Client& aClient, const char* aServerName, uint16_t aServerPort, String aURL);
void checkWiFi();
void connectWiFi();
void getNTPtime();
int gmtHour(int hr, int gmt);
void printWiFiStatus();
const char* strDate();
const char* strTime();
float mq2cal();
mq2data mq2sample(float r0);
float accTempSense();
float accHumiditySense();
float baroTempSense();
float baroPressureSense();
int motorVibSense();
int motionSense();
int moistureSense();
long distanceSense();
void scd4xStop();
void scd4xGetSerNo();
void scd4xStartMeas();
co2data scd4xRead(float tos);
float scd41GetOffset();
uint16_t scd41GetAltitude();
uint16_t scd41GetAscEnabled();
void scd41AscDisabled();
void scd41Persist();
uint16_t scd41SelfTest();
void scd41LPstart(bool lp);
uint16_t scd41forceCorrection(uint16_t co2Target, float tos);
void scd4xStatus();
void printUint16Hex(uint16_t value);
void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2);
void helloWorld();
void writeDisplay(char *sp1, char *sp2, char *sp3, char *sp4, char *sp5, char *sp6, char *sp7, char *sp8, char *sp9);
void enableRTCAlarm();
void alarmMatch();
void httpRequest(const char* bServerName, uint16_t bServerPort);
void listenToClient();
void sensorPrint(float voltage);
void R0Print(float r);
void rsGasPrint(float rs);
void ratioPrint(float rt);
void temperaturePrint(float t);
void humidityPrint(float h);
void pressurePrint(float p);
void moisturePrint(int m);
void distancePrint(long d);

/**************************************Setup********************************************************
 * Setup
 */
void setup() {

  Serial.begin(115200);               // setup the serial monitor at the specified baud rate
    
  pinMode(DISTANCE_SENSOR, INPUT);
  pinMode(PIR_MOTION_SENSOR, INPUT);
  pinMode(SPDT_RELAY, OUTPUT);
  pinMode(VIB_SENSOR,INPUT);
  pinMode(MQ2_GAS_SENSOR, INPUT);
  pinMode(MOISTURE_SENSOR,INPUT);

  checkWiFi();                         // test to see that the WiFi module is working
  connectWiFi();                       // connect to the wifi network and wait until it connects

  printWiFiStatus();                   // print the status of the wifi network connection
  
  rtc.begin();                         // start the real time clock (RTC)

  getNTPtime();                        // setup the RTC by getting the epoch from the NTP server
  enableRTCAlarm();                    // stup the alarm to enable every minute

  u8g2.begin();                        // setup the OLED Display 96x96 v2.1
  u8g2.clearBuffer();                  // Clear the memory buffer
        
  if (sensor.init()) {                 // Initialize the High Accuracy Temp & Humidity (SHT35) v1.0 sensor
    SERIAL.println("SHT35 init failed!!!");
    } else {
    Serial.println("SHT35 Init Complete");
    }
    
  while (!Serial);                    // Initialize the High Precision Barometric Sensor (OPS310) v1.0
  Dps310PressureSensor.begin(Wire);
  Serial.println("Barmeter initialization complete");
  
  R0 = mq2cal();                      // calibrate the MQ2 sensor

  Wire.begin();                       // sutup the wire I2C interface
  scd4x.begin(Wire);                  // setup the SDC41
  scd4xStop();                        // stop potentially previously started measurement
  scd4xGetSerNo();                    // Get serial number
  toffset = scd41GetOffset();         // Get temperature offset
  scd41GetAltitude();                 // Get sensor altitude
  scd41SelfTest();                    // Get status
  scd41GetAscEnabled();               // Get ASC enabled
  scd41AscDisabled();                 // Disable ASC
  scd41Persist();                     // Execute perist settings
  scd41GetAscEnabled();               // Get ASC enabled
  scd41LPstart(ifLowPower);           // Start low power measurement
  
  //scd41forceCorrection(co2expected, toffset);  // for the SCD41 to an expected value
  // httpRequest();                   // request website data
  // listenToClient();                // read data from the client
}

/*****************************************Loop***********************************************
 * Loop
 */
void loop() {
  co2data co2sample;
  mq2data mq2;
  
  if (sendRequest) {
    sendRequest = false;

    strncpy(shdate, strDate(), 10 );      // get the date
    strncpy(shtime, strTime(), 11);       // get the time
    temp1 = accTempSense();               // get the temperature from the High Accuracy Temp (SHT35) v1.0
    humidity = accHumiditySense();        // get the temperature from the High Accuracy Humidity (SHT35) v1.0
    temp2 = baroTempSense();              // get the temperature from the High Precision Barometric Sensor (OPS310) v1.0
    pressure = baroPressureSense();       // get the pressure from the High Precision Barometric Sensor (OPS310) v1.0
    mq2 = mq2sample(R0);                  // get the ration from the Gas Sensor (MQ2) V1.5
    motorStatus = motorVibSense();        // get the vibration status from the Piezo Vibration Sensor v1.1
    motionStatus = motionSense();         // get the motion status from the PIR Motion Sensor v1.2
    waterLevel = moistureSense();         // get the water level form the Ultrasonic Distance Sensor v2.0
    depth = distanceSense();              // get the distance measured by the Ultrasonic Distance Sensor v2.0
    co2sample = scd4xRead(toffset);       // get the CO2, Temperature, and humidity from the SEK-SCD41 Sensirion C02 sensor
                 
    // create string to write to the Google Sheet
    sprintf(sensestr, "%s %s Temperature = %d.%02d C, Humidity = %d.%02d %c, Temperature = %d.%02d C, Pressure = %d.%02d InHg, ratio = %d.%02d, Motor: %s, Motion: %s, Moisture = %d, Distance = %d cm\n", shdate, shtime, (int)temp1, (int)(temp1*100)%100, (int)humidity, (int)(humidity*100)%100, 0x25, (int)temp2, (int)(temp2*100)%100, (int)pressure, (int)(pressure*100)%100, (int)mq2.ratio, (int)(mq2.ratio*100)%100, motorStatus?"On":"Off", motionStatus?"On":"Off", waterLevel, depth);
    // Serial.print(sensestr);                  
    /*
     * Create string to write to the Display
    */
    //sprintf(displaystr, "%s %s\nT1 %d.%02dC H %d.%02d%c T2 %d.%02dC\nP %d.%02d InHg\nratio %d.%02d Mr: %s Mn: %s\nMoist %d D %d cm\n", shdate, shtime, (int)temp1, (int)(temp1*100)%100, (int)humidity, (int)(humidity*100)%100, 0x25, (int)temp2, (int)(temp2*100)%100, (int)pressure, (int)(pressure*100)%100, (int)mq2.ratio, (int)(mq2.ratio*100)%100, motorStatus?"On":"Off", motionStatus?"On":"Off", waterLevel, depth);
    //Serial.print(displaystr);

    sprintf(dl01, "%s %s",shdate, shtime);
    sprintf(dl02, "T1 %d.%02dC H1 %d.%02d%c", (int)temp1, (int)(temp1*100)%100, (int)humidity, (int)(humidity*100)%100, 0x25);
    sprintf(dl03, "T2 %d.%02dC P1 %d.%02d InHg", (int)temp2, (int)(temp2*100)%100, (int)pressure, (int)(pressure*100)%100);
    sprintf(dl04, "r %d.%02d", (int)mq2.ratio, (int)(mq2.ratio*100)%100);
    sprintf(dl05, "Mr: %s Mn: %s", motorStatus?"On":"Off", motionStatus?"On":"Off");
    sprintf(dl06, "Moist %d", waterLevel);
    sprintf(dl07, "D %d cm", depth);
    sprintf(dl08, "CO2 %d", co2sample.co2);
    sprintf(dl09, "T3 %d.%02dC H2 %d.%02d%c", (int)co2sample.temp3, (int)(co2sample.temp3*100)%100, (int)co2sample.hum2, (int)(co2sample.hum2*100)%100, 0x25 );
    writeDisplay(dl01, dl02, dl03, dl04, dl05, dl06, dl07, dl08, dl09);
    /*
     * Call sensor funtions here
    */
    //helloWorld();          // Writing to the display results in error reading the I2C sensors
      

    String URL;
    URL += logURL;
    URL += "?Date=";           //"Key1" must match a column name in your spreadsheet
    URL += shdate;             //"value1" can be whatever data from your sketch you want to log
    URL += "&Time=";           //"Key2" must match a column name in your spreadsheet
    URL += shtime;             //"value2" can be whatever data from your sketch you want to log
    URL += "&Temperature1=";   //"Key3" must match a column name in your spreadsheet
    URL += temp1;              //"value3" can be whatever data from your sketch you want to log
    URL += "&Humidity=";       //"Key4" must match a column name in your spreadsheet
    URL += humidity;           //"value4" can be whatever data from your sketch you want to log
    URL += "&Temperature2=";   //"Key5" must match a column name in your spreadsheet
    URL += temp2;              //"value5" can be whatever data from your sketch you want to log
    URL += "&Pressure=";       //"Key6" must match a column name in your spreadsheet
    URL += pressure;           //"value6" can be whatever data from your sketch you want to log
    URL += "&Ratio=";          //"Key7" must match a column name in your spreadsheet
    URL += mq2.ratio;          //"value7" can be whatever data from your sketch you want to log
    URL += "&motorStatus=";    //"Key8" must match a column name in your spreadsheet
    URL += motorStatus;        //"value8" can be whatever data from your sketch you want to log
    URL += "&motionStatus=";   //"Key9" must match a column name in your spreadsheet
    URL += motionStatus;       //"value9" can be whatever data from your sketch you want to log
    URL += "&waterLevel=";     //"Key10" must match a column name in your spreadsheet
    URL += waterLevel;         //"value10" can be whatever data from your sketch you want to log
    URL += "&Depth=";          //"Key11" must match a column name in your spreadsheet
    URL += depth;              //"value11" can be whatever data from your sketch you want to log
    URL += "&CO2=";            //"Key12" must match a column name in your spreadsheet
    URL += co2sample.co2;      //"value12" can be whatever data from your sketch you want to log
    URL += "&Temperature3=";   //"Key13" must match a column name in your spreadsheet
    URL += co2sample.temp3;    //"value13" can be whatever data from your sketch you want to log
    URL += "&Humidity2=";      //"Key14" must match a column name in your spreadsheet
    URL += co2sample.hum2;     //"value14" can be whatever data from your sketch you want to log

                               //you can add as many key/value pairs as you need to...
/*
  * Write the parameters to the Google Sheet
 * Sheet URL: 
 * 
 * Authentication from Aduino forum https://forum.arduino.cc/t/rest-message-authentication/655945
 * This function no long works as writen because Google requires users to authenticate
 * before access to the Sheet is given.
 * 
 */
 
    // write data to the Google sheet is now a function
    if (sendtoGoolgle) {
      if (co2sample.co2 > 0) {
        writeSheet(wificlt, serverAddress, port, URL);
     }
    }
    /*
     * write the URL to the Serial Monitor
     */
    Serial.println(URL);
    // httpRequest(serverAddress,port);
    // listenToClient();
  }
    //delay(1);              // wait a 1000 ms before getting the next time
}
/*
************************ Write Google Sheet **************************************
 * writeSheet 
 * aClinet is a created by WiFiSSLClient
 * aServerName is the URL of the server of the sheet (Google sheet)
 * aServerPort is the port of the server
 * aURL is the URL a string containing the data to write to the Google Sheet
 */
void writeSheet(Client& aClient, const char* aServerName, uint16_t aServerPort, String aURL){
    HttpClient client = HttpClient(aClient, aServerName, aServerPort);
    //client.noCheckSSL();       //required because the web app URL is https://... but does not work with ArduinoHttpClient library
    //client.connectSSL(aServerName, aServerPort);
    client.beginRequest();
    client.sendHeader("Accept", "application/json");
    client.sendBasicAuth(SECRET_AUTHUSER, SECRET_AUTHPSW);
    client.endRequest();
    statusCode = client.responseStatusCode();
    responsex = client.responseBody();
    if (statusCode >= 0) {
      client.get(aURL);        // write sensor data to the Google Sheet
    } else {
      Serial.print("GET Status code: ");
      Serial.println(statusCode);
      Serial.print("GET Response: ");
      Serial.println(responsex);
    } 
}
/******************************************WiFi Routines*************************************************
 * Test the WiFi module works. Run once in setup()
 */
void checkWiFi() {
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }
}


/*
 *  Setup the wifi module by attempting to connect to WiFi network. Run once in setup()
 */
void connectWiFi() {

  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
}
/*******************************************Realtime Clock**************************************
 * 
 * Setup the RTC by getting the time form the NTP. Run once in setup()
 */
void getNTPtime() {
  unsigned long epoch;
  int numberOfTries = 0, maxTries = 6;

  do {
    epoch = WiFi.getTime();
    numberOfTries++;
  }
  while ((epoch == 0) && (numberOfTries < maxTries));
  if (numberOfTries == maxTries) {
    Serial.print("NTP unreachable!!");
    while (1);
  } else {
    Serial.print("Epoch received: ");
    Serial.println(epoch);
    rtc.setEpoch(epoch);
    Serial.println();
  }
}
/*
 * The original code to compute the time corrected for the users
 * timezone does not work for negatve GMT values. gmtHour() calculates
 * the correct hour assuming a 24 hour clock.
 */
int gmtHour(int hr, int gmt) {
  diff = (hr + gmt);
  if(diff < 0 ) {
    tz = (24 + diff);
  } else {
    tz = diff;
  }
  return tz;
}
/*********************************WiFi Status**********************************
 * Print the WiFi status
 */
void printWiFiStatus() {

  // print the SSID of the network you're attached to:

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:

  long rssi = WiFi.RSSI();

  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
/*
 * Convert the date to a string
 */
const char* strDate() {
  static char sdate[10];
  sprintf(sdate, "%02d/%02d/%02d", rtc.getMonth(), rtc.getDay(), rtc.getYear());
  return sdate;
}
/*
 * Get the time and return it as string
 */
const char* strTime() {
  static char stime[11];
  sprintf(stime, "%02d:%02d:%02d", gmtHour(rtc.getHours(), GMT), rtc.getMinutes(), rtc.getSeconds());
  return stime;
}
/*
  ******************************************Sensors***************************************************
 */
/*
 * MQ2 Gas Sensor Calibration
 * 
 * Run the calibration routine calculated to get the baseline baseline air 
 * quaility R0. R0 should be 9.8 for clean air. R0 is used to compare follow
 * on gas measurements to get a measure of how much cass is present.
 */
float mq2cal() {
  int x;
  float sensorValue;
  float sensorVoltage;
  float r0;
  float rs_air;
  
  for(x = 0 ; x< 100 ; x++) 
  {
    sensorValue = sensorValue + analogRead(MQ2_GAS_SENSOR); // get 100 measurements
  }
  sensorValue = sensorValue/100.0;                // take a average
  sensorVoltage = (sensorValue/1024)*5.0;         // convert the reading to a voltage
  rs_air = (5.0-sensorVoltage)/sensorVoltage;     // convert voltage to a proportion
  r0 = rs_air/9.8;                                // The ratio of the measurement to clean air
  sensorPrint(sensorVoltage);                     // print the sample voltage
  R0Print(r0);                                    // print R0
  Serial.println();
  return r0;
}
/*
 * mq2sensor reads the sensors and compares it to R0
 * mq2ample returns a structure containing sensorVoltage,
 *  rsgas, and ratio.
 */
mq2data mq2sample(float r0) {
  mq2data mq2s;
  float sensorValue;

  sensorValue = analogRead(MQ2_GAS_SENSOR);        // sample sensor
  mq2s.sensorVoltage = (sensorValue/1024)*5.0;     // convert sensor value to a voltage
  mq2s.rsgas = (5.0-mq2s.sensorVoltage)/mq2s.sensorVoltage;  // convert volatage to a proportion
  mq2s.ratio = mq2s.rsgas/r0;                       // ratio of sample to clean air calibration
  //sensorPrint(mq2.sensorVoltage);                // print the sensor voltage
  //rsGasPrint(mq2s.rsgas);                        // print the proportion
  //ratioPrint(mq2s.ratio);                        // print the ratio
  return mq2s;
}
/*
 * High Accuracy Temp & Humidity (SHT35) v1.0
 */
 float accTempSense() {
    u16 value = 0;
    u8 data[6] = {0};
    float temp, hum;
    if (NO_ERROR != sensor.read_meas_data_single_shot(HIGH_REP_WITH_STRCH, &temp, &hum)) {
        return -1e-6;
    } else {
      return temp;
    }
 }
 /*
 * High Accuracy Temp & Humidity (SHT35) v1.0
 */
 float accHumiditySense() {
    u16 value = 0;
    u8 data[6] = {0};
    float temp, hum;
    if (NO_ERROR != sensor.read_meas_data_single_shot(HIGH_REP_WITH_STRCH, &temp, &hum)) {
        return -1e-6;
    } else {
      return hum;
    }
 }
/*
 * High Precision Barometric Sensor (OPS310) v1.0
 * Temperature measutement. Returns 1e-6 on error
 */
float baroTempSense() {
  float temperature;
  uint8_t oversampling = 7;
  int16_t ret;
  
  ret = Dps310PressureSensor.measureTempOnce(temperature, oversampling);
  if (ret != 0)
  {
    //Something went wrong.
    //Look at the library code for more information about return codes
    Serial.print("Error: ret = ");
    Serial.println(ret);
    return 1e-6;
  }
  else
  {
    return temperature;
  }
}
/*
 * High Precision Barometric Sensor (OPS310) v1.0
 * Pressure measutement in Pascals. 1 Pascal = 0.0002954 inHg Returns 1e-6 on error
 */
float baroPressureSense() {
  float pressure;
  uint8_t oversampling = 7;
  int16_t ret;
  float const PinHg = 0.0002953;
  
  ret = Dps310PressureSensor.measurePressureOnce(pressure, oversampling);
  if (ret != 0)
  {
    //Something went wrong.
    //Look at the library code for more information about return codes
    Serial.print("Error: ret = ");
    Serial.println(ret);
    return -1e-6;
  }
  else
  {
    return pressure * PinHg;
  }
}
/*
 * Piezo Vibration Sensor v1.1
 */
 int motorVibSense() {
  int vib;
  vib = digitalRead(VIB_SENSOR);
  return vib;
 }
 /*
  *  PIR Motion Sensor v1.2
  */
int motionSense() {
  int motion;
  motion = digitalRead(PIR_MOTION_SENSOR);
  return motion;
 }
 /*
  *  Cpacitive Moisture Sensor v1.0
  */
int moistureSense() {
  int moisture;
  moisture = analogRead(MOISTURE_SENSOR);
  return moisture;
 }
 /*
  * 
  * Ultrasonic Distance Sensor v2.0
  */
long distanceSense() {
  long r;
  r = ultrasonic.MeasureInCentimeters();
  return r;
 }
/*
 * SCD41x CO2 sensor functions -----------------------------------------------------
 * 
 * The SCD4X routines are based on based on the examples from Sensirion AG that are copyrighted
 * The following functions are a modification of the Sensirion AG Copyrighted functions
 *   scd4xStop();              // stop potentially previously started measurement
 *   scd4xGetSerNo();          // Get serial number
 *   scd41GetOffset();         // Get temperature offset
 *   scd41GetAltitude();       // Get sensor altitude
 *   scd41GetAscEnabled();     // Get ASC enabled
 *   scd41SelfTest();          // Get status
 *   scd41LPstart(ifLowPower); // Start low power measurement
 *   scd41AscDisabled();       // Disable ASC
 *   scd41Persist();           // Execute perist settings
 *   scd41forceCorrection;     // Recalbirate the sensor to a target value
 */
 /*
  * SDC4X Stop Measurement
  */
 void scd4xStop() {
   uint16_t error;
   char errorMessage[256];
   // stop potentially previously started measurement
   error = scd4x.stopPeriodicMeasurement();
   if (error) {
      Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
   }
   delay(500);
 }
 /*
  * SDC4X Get Serial number
  */
void scd4xGetSerNo() {
  uint16_t error;
  char errorMessage[256];
  uint16_t serial0;
  uint16_t serial1;
  uint16_t serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error) {
      Serial.print("Error trying to execute getSerialNumber(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  } else {
      printSerialNumber(serial0, serial1, serial2);
  }
}
/*
 * SCD4x Start measurement
 */
void scd4xStartMeas() {
   uint16_t error;
   char errorMessage[256];
   error = scd4x.startPeriodicMeasurement();
   if (error) {
       Serial.print("Error trying to execute startPeriodicMeasurement(): ");
       errorToString(error, errorMessage, 256);
       Serial.println(errorMessage);
    }
    Serial.println("Waiting for first measurement... (5 sec)");
    delay(5000);
}
/*
 * SCD4X Read Measurement
 * Returns a string with the CO2 level, temperature, and humidity.
 * tos is the calibration temperature offset
 */
co2data scd4xRead(float tos) {
  uint16_t error, dataReady;
  char errorMessage[256];
  co2data scd41;
  uint16_t c;
  float t;
  float h;

  scd4x.getDataReadyStatus(dataReady);
  dataReady &= 0x07FF; //Serial.print("Data ready =");Serial.println(dataReady, HEX);
  if (dataReady) {
    error = scd4x.readMeasurement(c, t, h);
    if (error) {
       Serial.print("Error trying to execute readMeasurement(): ");
       errorToString(error, errorMessage, 256);
       Serial.println(errorMessage);
    } else if (c == 0) {
      Serial.println("Invalid sample detected, skipping.");
       scd41.co2 = 0;
       scd41.temp3 = 0;
       scd41.hum2 = 0;
      } else {
       scd41.co2 = c;
       scd41.temp3 = t + tos;
       scd41.hum2 = h;
      }
  }
  return scd41;
}
/*
 * Get SCD41 temperature offset
 */
float scd41GetOffset() {
    float tempOffset;
    uint16_t errorNum;
    char errorMessage[256];
    errorNum = scd4x.getTemperatureOffset(tempOffset);
    if (errorNum) {
        Serial.print("Error trying to execute getTemperatureOffset(): ");
        errorToString(errorNum, errorMessage, 256);
        Serial.println(errorMessage);
    }
    Serial.print("Temperature offset (C) = ");
    Serial.println(tempOffset);
    return tempOffset;
 }
 /*
  * Get SCD41 Altitude
  */
uint16_t scd41GetAltitude() {
    uint16_t sensorAltitude;
    uint16_t errorNum;
    char errorMessage[256];
    errorNum = scd4x.getSensorAltitude(sensorAltitude);
    if (errorNum) {
      Serial.print("Error trying to execute getSensorAltitude(): ");
      errorToString(errorNum, errorMessage, 256);
      Serial.println(errorMessage);
    } 
    Serial.print("Sensor altitude (m) = ");
    Serial.println(sensorAltitude);
    return sensorAltitude;
}
/*
 * Get ASC Enabled
 */
uint16_t scd41GetAscEnabled() {
   uint16_t ascEnabled;
   uint16_t errorNum;
   char errorMessage[256];
   errorNum = scd4x.getAutomaticSelfCalibration(ascEnabled);
    if (errorNum) {
      Serial.print("Error trying to execute getAutomaticSelfCalibration(): ");
      errorToString(errorNum, errorMessage, 256);
      Serial.println(errorMessage);
    }
    Serial.print("ASC = ");
    Serial.println(ascEnabled);
    return ascEnabled;
}
/*
 * Set ASC Disabled
 */
void scd41AscDisabled() {
   uint16_t errorNum;
   char errorMessage[256];
   errorNum = scd4x.setAutomaticSelfCalibration(0x0000U);
    if (errorNum) {
      Serial.print("Error trying to execute setAutomaticSelfCalibration(): ");
      errorToString(errorNum, errorMessage, 256);
      Serial.println(errorMessage);
    } else {
    }
}
/*
 * Execute persist settings
 */
void scd41Persist() {
   uint16_t errorNum;
   char errorMessage[256];
    errorNum = scd4x.persistSettings();
    if (errorNum) {
      Serial.print("Error trying to execute persistSettings(): ");
      errorToString(errorNum, errorMessage, 256);
      Serial.println(errorMessage);
    } else {
    }
    delay(800);
}
/*
 * Get Self Test
 */
 uint16_t scd41SelfTest() {
   uint16_t sensorStatus;
   uint16_t errorNum;
   char errorMessage[256];
   errorNum = scd4x.performSelfTest(sensorStatus);
    if (errorNum) {
      Serial.print("Error trying to execute performSelfTest(): ");
      errorToString(errorNum, errorMessage, 256);
      Serial.println(errorMessage);
    } 
    Serial.print("Self-test status = ");
    Serial.println(sensorStatus, HEX); 
    return sensorStatus;
    delay(10000);
 }
 /*
  * Low power Statt Measuement
  */
void scd41LPstart(bool lp) {
    uint16_t errorNum;
    char errorMessage[256];
    if (lp) {
        errorNum = scd4x.startLowPowerPeriodicMeasurement();
        if (errorNum) {
            Serial.print("Error trying to execute startPeriodicMeasurement(): ");
            errorToString(errorNum, errorMessage, 256);
            Serial.println(errorMessage);         
        }
        Serial.println("Started Low Power Periodic Measurement.");
    } else {
        errorNum = scd4x.startPeriodicMeasurement();
        if (errorNum) {
            Serial.print("Error trying to execute startPeriodicMeasurement(): ");
            errorToString(errorNum, errorMessage, 256);
            Serial.println(errorMessage);      
        }
        Serial.println("Started High Power Periodic Measurement.");
    }
}
 /*
  * Force recaliblration
  * co2target is the value to recalibrate to
  * the functio returns the calibration value
  */
uint16_t scd41forceCorrection(uint16_t co2Target, float tos) {
    uint16_t errorNum;
    char errorMessage[256];
    uint16_t frcCorrection;
    bool ifLowPower = true;
    co2data scd41;
    Serial.println("Recalibration started");
    //scd41SelfTest();              // Get Self test
    //scd4xStop();                  // Stop measurement
    //scd4xStartMeas();             // Start peridic measurements
    //Read for 3 minutes
    uint32_t startTime = millis();
    while(millis() - startTime < (3 * 1000 * 60)) {
      delay(5000);
      scd41 = scd4xRead(tos);
      Serial.println(scd41.co2);
    }
    scd4xStop();                  // Stop measurement
    Serial.println("Stopped Periodic Measurement.");
    // force the correction
    errorNum = scd4x.performForcedRecalibration(co2Target, frcCorrection);
    if (errorNum) {
        Serial.print("Error trying to execute performForcedRecalibration(): ");
        errorToString(errorNum, errorMessage, 256);
        Serial.println(errorMessage);      
    }
    Serial.print("Target Value = ");
    Serial.println(co2Target);
    Serial.println("Force recalibration done.");
    Serial.print("Correction returned is ");
    Serial.println(frcCorrection, DEC);
    scd41LPstart(ifLowPower);         // Start the measurement up again
}
/*
 * Sensirion SCD41 Sensor status
 */
void scd4xStatus() {
    bool ifLowPower = true;
    
    scd4xStop();              // stop potentially previously started measurement
    scd4xGetSerNo();          // Get serial number
    scd41GetOffset();         //Get temperature offset
    scd41GetAltitude();       // Get sensor altitude
    scd41GetAscEnabled();     //Get ASC enabled
    scd41SelfTest();          // Get status
    scd41LPstart(ifLowPower); // Start low power measurement
}
/*
 * SDC4X Print Funtions 
 */
void printUint16Hex(uint16_t value) {
    Serial.print(value < 4096 ? "0" : "");
    Serial.print(value < 256 ? "0" : "");
    Serial.print(value < 16 ? "0" : "");
    Serial.print(value, HEX);
}
void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
    Serial.print("Serial: 0x");
    printUint16Hex(serial0);
    printUint16Hex(serial1);
    printUint16Hex(serial2);
    Serial.println();
}
/* 
 ************************************************** Display ***********************
 * OLED Display 96x96 v2.1
 */
void helloWorld() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(0,20,"Hello World!");
    } while ( u8g2.nextPage() );
}
/*
 * Write sensor data to the display
 * using a pointer to a string
 * ln is the line number 
 * *sp is a pointer to the string to be displayed on line ln
 * lw is the line width
 * ls is the spacing between lines
 * loc is the location of the line
 */
void writeDisplay(char *sp1, char *sp2, char *sp3, char *sp4, char *sp5, char *sp6, char *sp7, char *sp8, char *sp9) {
  int l1 = 12;
  int l2 = 22;
  int l3 = 32;
  int l4 = 42;
  int l5 = 52;
  int l6 = 62;
  int l7 = 72;
  int l8 = 82;
  int l9 = 92;
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, l1, sp1);
    u8g2.drawStr(0, l2, sp2);
    u8g2.drawStr(0, l3, sp3);
    u8g2.drawStr(0, l4, sp4);
    u8g2.drawStr(0, l5, sp5);
    u8g2.drawStr(0, l6, sp6);
    u8g2.drawStr(0, l7, sp7);
    u8g2.drawStr(0, l8, sp8);
    u8g2.drawStr(0, l9, sp9);
    } while ( u8g2.nextPage() );
}
/*
 ************************************************* RTC *******************************
 * Set the alarm to enable every minute
 */
void enableRTCAlarm() {
  rtc.setAlarmTime(0, 0, 0);    //in this way the request is sent every minute at 0 seconds
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(alarmMatch);
 }
/*
 * Enable the alarm
 */
 void alarmMatch() {
  sendRequest = true;
}

/*
 * ********************************************** httpReqest ***********************
 * this method makes a HTTP connection to the server:
 * I don't think this function ever worked properly
 */
void httpRequest(const char* bServerName, uint16_t bServerPort) {
  sendRequest = false;

  Serial.println();
  Serial.print("Request made @ ");
  Serial.print(strTime());
  int clientStatus = wificlt.connectSSL(bServerName, bServerPort);
  Serial.print(" status: ");
  Serial.print(clientStatus);
  Serial.println();
 
  if (wificlt.connectSSL(bServerName, bServerPort)) {
    // Make a HTTP request:
    //client.println("GET /asciilogo.txt HTTP/1.1");
    wificlt.println("GET /search?q=arduino HTTP/1.1");
    wificlt.println("Host: www.arduino.cc");
    wificlt.println("Connection: close");
    wificlt.println();
  } else {
    Serial.println("connection failed");
  }
}

void listenToClient(){
  unsigned long startTime = millis();
  bool received = false;
  while ((millis() - startTime < 5000) && !received) { //try to listen for 5 seconds
    while (wificlt.available()) {
      received = true;
      char c = wificlt.read();
      Serial.write(c);
    }
  }
  wificlt.stop();
  Serial.println();
}

/*
 * ************************************************ Print functions ***********************
 * Print functions
 * These have been replaced with sprintf() function
 * and may be delated when they are no longer needed
 */
 void sensorPrint(float voltage) {
  Serial.print("Sensor Voltage = ");
  Serial.print(voltage);
  Serial.print(" V, ");  
}
void R0Print(float r) {
  Serial.print("R0 = ");
  Serial.print(r);
  Serial.print(", ");  
}
void rsGasPrint(float rs) {
  Serial.print("RS_gas = ");
  Serial.print(rs);
  Serial.print(", ");
}
void ratioPrint(float rt) {
  Serial.print("ratio = ");
  Serial.print(rt);
  Serial.print(", ");  
}
void temperaturePrint(float t) {
  Serial.print("Temperature = ");
  Serial.print(t);
  Serial.print(" C, ");  
}
void humidityPrint(float h) {
  Serial.print("Humidity = ");
  Serial.print(h);
  Serial.print(" %, ");  
}
void pressurePrint(float p) {
  Serial.print("Pressure = ");
  Serial.print(p);
  Serial.print(" InHg, ");  
}
void moisturePrint(int m) {
  Serial.print("Moisture = ");
  Serial.print(m);
  Serial.print(", ");  
}
void distancePrint(long d) {
  Serial.print("Distance = ");
  Serial.print(d);
  Serial.print(" cm, ");  
}