#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Adxl355.h>
#include <TinyGPS++.h>
#include <MQTT.h>
#include "config.h"

//Declare Functions for PlatformIO
void IRAM_ATTR isr_gps();
void IRAM_ATTR isr_adxl();
void WiFiEvent(WiFiEvent_t event);
int numScannedNetworks();
int numNetworksStored();
void readNetworkStored(int netId);
void storeNetwork(String ssid, String pswd);
bool readAndConnect();
bool startSmartConfig();
//void generateMessage(String &out, long fifoData[32][3], int entryCount, char *timeObtained);
void generateMessage(String &out, long fifoData[32][3], int entryCount);
void appendArray(String &out, long fifoData[32][3], int entryCount, int index);

//Objects
Preferences prefs;
TinyGPSPlus gps;
WiFiClient net;
MQTTClient client = MQTTClient(4096);

//WiFi variables
String _ssid;
String _pswd;
int networksStored;
bool connected = false;
unsigned long reconnectionTimer;

//GPS
struct Pulse
{
  const uint8_t PIN;
  bool pressed;
};

double _latitude;
double _longitude;
double _hdop;
double _altitude;
unsigned long lastTime = 0;

Pulse pps = {4, false}; //GPS PPS attached in pin 4
bool gpsData = false;
char location_str[100];
char timestamp[32];
HardwareSerial gps_serial(2);
String globalTimeTrace;
char data[100];

//GPS pulse interrupt set flags
void IRAM_ATTR isr_gps()
{
  pps.pressed = true;
  //lastTime = micros(); //If inestability, change to loop
  //timeDiff = micros() - lastTime;
}

//ADXL Accelerometer
int8_t CHIP_SELECT_PIN_ADXL = 15;
int8_t INT_PIN = 2;
Adxl355::RANGE_VALUES range = Adxl355::RANGE_VALUES::RANGE_2G;
Adxl355::ODR_LPF odr_lpf;
Adxl355::STATUS_VALUES status;
bool deviceRecognized = false;

long fifoOut[32][3];
String traceMessage;
int numEntriesFifo = 0;
long runningAverage[3] = {0, 0, 0};
long sumFifo[3];
long fifoDelta[32][3];
bool fifoFull = false;
int fifoCount = 0;
int numValsForAvg = 0;

unsigned long rightNow;
unsigned long printTime;

String fifoMessage = "{\"traces\":[";

Adxl355 adxl355(CHIP_SELECT_PIN_ADXL);
SPIClass *spi1 = NULL;

void IRAM_ATTR isr_adxl()
{
  fifoFull = true;
  rightNow = micros();
  //fifoCount++;
}

double sr = 0;

void setup()
{
#if debug
  Serial.begin(115200);
#endif

#if SAMPLE_RATE_125
  odr_lpf = Adxl355::ODR_LPF::ODR_125_AND_31_25;
  sr = 125.0;
#endif

#if SAMPLE_RATE_31_25
  odr_lpf = Adxl355::ODR_LPF::ODR_31_25_AND_7_813;
  sr = 31.25;
#endif

  pinMode(pps.PIN, INPUT_PULLUP);
  attachInterrupt(pps.PIN, isr_gps, RISING);

  pinMode(INT_PIN, INPUT);
  pinMode(CHIP_SELECT_PIN_ADXL, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), isr_adxl, FALLING);

  //gps_serial.begin(9600, SERIAL_8N1, 16, 17);

  spi1 = new SPIClass(HSPI);
  adxl355.initSPI(*spi1);

  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);

  delay(500);

  adxl355.start();
  delay(1000);

  if (adxl355.isDeviceRecognized())
  {
    deviceRecognized = true;
    DEBUG("Initializing sensor");
    adxl355.initializeSensor(range, odr_lpf, debug);
    DEBUG("Calibrating sensor");
    adxl355.calibrateSensor(5, debug);
    DEBUG("Acc. activated");
  }
  else
  {
    DEBUG("Unable to get accelerometer");
  }

  DEBUG("Finished accelerometer configuration");

  /*
  while (!gpsData)
  {
    while (gps_serial.available() > 0)
    {
      gps.encode(gps_serial.read());
    }
    if (gps.location.isValid() && gps.altitude.isValid() && gps.hdop.isValid())
    {
      _latitude = gps.location.lat();
      _longitude = gps.location.lng();
      _hdop = (int)gps.hdop.hdop();
      _altitude = gps.altitude.meters();

      sprintf(location_str, "{\"id\":\"%s\",\"sr\":%.2f,\"lat\": %.6f, \"lon\": %.6f, \"hdop\":%.2f, \"alt\":%.2f}", deviceId, sr, _latitude, _longitude, _hdop, _altitude);
      DEBUG("Got location string:");
      DEBUG(location_str);

      gpsData = true;
    }
  }
  DEBUG("Finished GPS configuration");

  */

  if (!(connected = readAndConnect()))
  {
    startSmartConfig();
  }
  DEBUG("Finished connection");

  if (connected)
  {
    client.begin(MQTTBrokerIp, MQTTBrokerPort, net);

    while (!client.connect(deviceId))
    {
      DEBUG_IL(".");
      delay(1000);
    }

    if (client.connected())
      DEBUG("Connected to MQTT");

    //sendPacket(location_str);//Change to MQTT
  }
  DEBUG("Sent location string");
}

static int id = 0;

void loop()
{
  client.loop();
  delay(10);

  //========================= WiFi Reconnection ==================
  if (WiFi.status() != WL_CONNECTED && (millis() - reconnectionTimer) > RECONNECTION_TO)
  {
    DEBUG("Retrying connection");
    reconnectionTimer = millis();
    WiFi.mode(WIFI_STA);
    connected = readAndConnect();
  }
  //=========================== GPS Section ========================

  /*
  while (gps_serial.available() > 0)
  {
    gps.encode(gps_serial.read());
  }
  
  if (pps.pressed)
  {
    lastTime = micros();
    if (gps.date.isValid())
    {

      time_t rawtime;

      struct tm *time_of_day;
      time_of_day = localtime(&rawtime);

      time_of_day->tm_year = gps.date.year() - 1900;
      time_of_day->tm_mon = gps.date.month() - 1;
      time_of_day->tm_mday = gps.date.day();
      time_of_day->tm_hour = gps.time.hour();
      time_of_day->tm_min = gps.time.minute();
      time_of_day->tm_sec = gps.time.second();

      sprintf(timestamp, "%i", mktime(time_of_day));
      //sendPacket(timestamp);
      //DEBUG_IL("Got time from GPS: ");
      //DEBUG(timestamp);
      pps.pressed = false;
    }
  }
  
  if (gpsData && micros() - lastTime > 10000000)
  {
    gpsData = false;
    char gps_msg[100];
    sprintf(gps_msg, "{\"msg \":\"GPS_LOST\"}");
    //    sendPacket(gps_msg);
    DEBUG("GPS not available");
  }

  */

  //====================== ADXL Accelerometer =====================
  //if (fifoFull && gpsData)
  if (fifoFull)
  {
    fifoFull = false;
    status = adxl355.getStatus();

    if (status & Adxl355::STATUS_VALUES::FIFO_FULL)
    {
      //DEBUG("FIFO is full");

      /*

      if (lastTime > rightNow)
      { //If new PPS comes between ADXL interrupt and print time creation, there would be negative time
        rightNow = micros();
      }

      printTime = rightNow - lastTime;
      char aux[100];
      sprintf(aux, "%s.%06d", timestamp, printTime);

      //DEBUG(aux);

      */

      if (-1 != (numEntriesFifo = adxl355.readFifoEntries((long *)fifoOut)))
      {
        sumFifo[0] = 0;
        sumFifo[1] = 0;
        sumFifo[2] = 0;

        for (int i = 0; i < 32; i++)
        {
          for (int j = 0; j < 3; j++)
          {
            fifoDelta[i][j] = fifoOut[i][j] - runningAverage[j];
            sumFifo[j] += fifoOut[i][j];
          }
        }

        for (int j = 0; j < 3; j++)
        {
          runningAverage[j] = (numValsForAvg * runningAverage[j] + sumFifo[j]) / (numValsForAvg + 32);
        }

        numValsForAvg = min(numValsForAvg + 32, 2000);

        //generateMessage(traceMessage, fifoDelta, numEntriesFifo, aux);
        generateMessage(traceMessage, fifoDelta, numEntriesFifo);

        fifoMessage += traceMessage.c_str();
        fifoCount++;

        if (fifoCount < MAX_FIFO_COUNT)
        {
          fifoMessage += ", ";
        }
        else
        {
          // if (gpsData && client.connected())
          if (client.connected())
          {
            //  fifoMessage += "]},\n";
            fifoMessage += "]}\n";

            //DEBUG(fifoMessage);
            if (!(client.publish("/trace", fifoMessage)))
            {
              DEBUG_IL("Could not send packet with id: ");
              DEBUG((String)id);
            }
            else
            {
              DEBUG(fifoMessage);
              //                DEBUG_IL("Sent trace with id: ");
              //                DEBUG((String)id);
            }
            fifoCount = 0;
            id++;
            fifoMessage = "{\n\"device\":\"" + (String)deviceId + "\", \n\"id\":" + (String)id + ", \n\"traces\":[";
          }
        }
      }
    }
  }
}

void WiFiEvent(WiFiEvent_t event)
{
  char event_str[30];
  sprintf(event_str, "[WiFi-event] event: %d\n", event);
  DEBUG_L2(event_str);

  switch (event)
  {
  case SYSTEM_EVENT_WIFI_READY:
    DEBUG_L2("WiFi interface ready");
    break;
  case SYSTEM_EVENT_SCAN_DONE:
    DEBUG_L2("Completed scan for access points");
    break;
  case SYSTEM_EVENT_STA_START:
    DEBUG_L2("WiFi client started");
    break;
  case SYSTEM_EVENT_STA_STOP:
    DEBUG_L2("WiFi clients stopped");
    break;
  case SYSTEM_EVENT_STA_CONNECTED:
    DEBUG_L2("Connected to access point");
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    DEBUG_L2("Disconnected from WiFi access point");
    connected = false;
    reconnectionTimer = millis();
    break;
  case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
    DEBUG_L2("Authentication mode of access point has changed");
    break;
  case SYSTEM_EVENT_STA_GOT_IP:
    DEBUG_IL_L2("Obtained IP address: ");
    DEBUG_L2(WiFi.localIP());
    break;
  case SYSTEM_EVENT_AP_START:
    DEBUG_L2("WiFi access point started");
    break;
  case SYSTEM_EVENT_AP_STOP:
    DEBUG_L2("WiFi access point  stopped");
    break;
  case SYSTEM_EVENT_AP_STACONNECTED:
    DEBUG_L2("Client connected");
    break;
  case SYSTEM_EVENT_AP_STADISCONNECTED:
    DEBUG_L2("Client disconnected");
    break;
  case SYSTEM_EVENT_AP_STAIPASSIGNED:
    DEBUG_L2("Assigned IP address to client");
    break;
  case SYSTEM_EVENT_AP_PROBEREQRECVED:
    DEBUG_L2("Received probe request");
    break;
  default:
    break;
  }
}

//================================= WiFi Handling ================================
//Scan networks in range and return how many are they.
int numScannedNetworks()
{
  int n = WiFi.scanNetworks();
  DEBUG_L2("scan done");
  if (n == 0)
  {
    DEBUG_L2("no networks found");
  }
  else
  {
    DEBUG_IL_L2(n);
    DEBUG_L2(" networks found");
#if LOG_L2
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      DEBUG_IL_L2(i + 1);
      DEBUG_IL_L2(": ");
      DEBUG_IL_L2(WiFi.SSID(i));
      DEBUG_IL_L2(" (");
      DEBUG_IL_L2(WiFi.RSSI(i));
      DEBUG_IL_L2(")");
      DEBUG_L2((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
      delay(10);
    }
#endif
  }
  return n;
}
//Return how many networks are stored in the NVM
int numNetworksStored()
{
  prefs.begin("networks", true);
  networksStored = prefs.getInt("num_nets");
  DEBUG_IL("Found this nets: ");
  DEBUG(networksStored);
  prefs.end();

  return networksStored;
}

//Each network as an id so reading the network stored with said ID.
void readNetworkStored(int netId)
{
  DEBUG("Reading stored networks");

  prefs.begin("networks", true);
  String idx;
  idx = "SSID" + (String)netId;
  _ssid = prefs.getString(idx.c_str(), "");
  idx = "key" + (String)netId;
  _pswd = prefs.getString(idx.c_str(), "");
  prefs.end();

  DEBUG_IL("Found network ");
  DEBUG_IL(_ssid);
  DEBUG_IL(" , ");
  DEBUG(_pswd);
}

//Save a pair of SSID and PSWD to NVM
void storeNetwork(String ssid, String pswd)
{
  DEBUG_IL("Writing network: ");
  DEBUG_IL(ssid);
  DEBUG_IL(",");
  DEBUG(pswd);

  prefs.begin("networks", false);
  int aux_num_nets = prefs.getInt("num_nets");
  DEBUG_IL("num nets returned ");
  DEBUG(aux_num_nets);
  aux_num_nets++;
  String idx;
  idx = "SSID" + (String)aux_num_nets;
  prefs.putString(idx.c_str(), ssid);
  idx = "key" + (String)aux_num_nets;
  prefs.putString(idx.c_str(), pswd);
  prefs.putInt("num_nets", aux_num_nets);
  prefs.end();
  DEBUG_IL("Device has ");
  DEBUG_IL(aux_num_nets);
  DEBUG(" networks stored");
}

//Joins the previous functions, gets the stored networks and compares to the available, if there is a match and connects, return true
//if no match or unable to connect, return false.
bool readAndConnect()
{
  int num_nets = numNetworksStored();
  int num_scan = numScannedNetworks();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

  for (int i = 1; i < (num_nets + 1); i++)
  {
    readNetworkStored(i);

    for (int j = 0; j < num_scan; j++)
    {
      if (_ssid == WiFi.SSID(j))
      {
        //Serial.print("Status from connection attempt");
        WiFi.begin(_ssid.c_str(), _pswd.c_str());
        WiFi.setSleep(false);
        unsigned long t0 = millis();

        while (WiFi.status() != WL_CONNECTED && (millis() - t0) < CONNECTION_TO)
        {
          delay(1000);
        }
        if (WiFi.status() == WL_CONNECTED)
        {
          DEBUG("WiFi was successfully connected");
          return true;
        }
        else
        {
          DEBUG("There was a problem connecting to WiFi");
        }
      }
      else
      {
        DEBUG("Got no match for network");
      }
    }
  }
  DEBUG("Found no matches for saved networks");
  return false;
}

//Executes the Smart config routine and if connected, will save the network for future use

bool startSmartConfig()
{
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP_STA);
  WiFi.beginSmartConfig();

  //Wait for SmartConfig packet from mobile
  DEBUG("Waiting for SmartConfig.");
  while (!WiFi.smartConfigDone())
  {
    delay(500);
    DEBUG_IL(".");
  }

  DEBUG("");
  DEBUG("SmartConfig received.");

  //Wait for WiFi to connect to AP
  DEBUG("Waiting for WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < CONNECTION_TO)
  {
    delay(500);
    DEBUG_IL(".");
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    _ssid = WiFi.SSID();
    _pswd = WiFi.psk();
    DEBUG_IL("Smart config done, connected to: ")
    DEBUG_IL(_ssid);
    DEBUG_IL(" with psswd: ");
    DEBUG(_pswd);
    storeNetwork(_ssid, _pswd);
    return true;
  }
  else
  {
    DEBUG("Something went wrong with SmartConfig");
    return false;
  }
}
//void generateMessage(String &out, long fifoData[32][3], int entryCount, char *timeObtained)
void generateMessage(String &out, long fifoData[32][3], int entryCount)
{
  //Serial.println((String)timeObtained);
  
  char buffer[20];
  //out = "\n{ \n\"t\" : " + (String)timeObtained;
  //out += ",\n\"sr\": " + (String)sr;
  //out += ",\n\"x\" : ";
  out = "{\"x\" : ";
  appendArray(out, fifoData, entryCount, 0);
  out += ",\n\"y\" : ";
  appendArray(out, fifoData, entryCount, 1);
  out += ",\n\"z\" : ";
  appendArray(out, fifoData, entryCount, 2);

  out += "\n}";
  
}
void appendArray(String &out, long fifoData[32][3], int entryCount, int index)
{
  // stringstream ss;
  out += "[";
  double gal;
  int32_t galInt;
  int32_t galDec;
  char buffer[20];

  for (int i = 0; i < entryCount; i++)
  {
    gal = adxl355.valueToGals(fifoData[i][index]);
    sprintf(buffer, "%.3f", gal);
    out += (String)buffer;

    if (entryCount - i != 1)
      out += ", ";
  }

  out += "]";

}
