#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Adxl355.h>
#include <math.h>
#include "config.h"

// --------------------------------------------------------------------------------------------
//        UPDATE CONFIGURATION TO MATCH YOUR ENVIRONMENT
// --------------------------------------------------------------------------------------------

// Watson IoT connection details
static char MQTT_HOST[48];  // ORGID.messaging.internetofthings.ibmcloud.com
//#define MQTT_PORT 1883
#define MQTT_PORT 8883      // Secure MQTT
char MQTT_DEVICEID[30];     // Allocate a buffer large enough for "d:orgid:devicetype:deviceid"
char deviceId[13];
#define MQTT_ORGID       "abcdef"     // Watson IoT 6 character orgid
#define MQTT_TOKEN       "secret-password-here"   // Watson IoT DeviceId authentication token  
#define MQTT_DEVICETYPE  "OpenEEW"    // Watson IoT DeviceType
#define MQTT_USER        "use-token-auth"
#define MQTT_TOPIC       "iot-2/evt/status/fmt/json"
#define MQTT_TOPIC_ALARM "iot-2/cmd/earthquake/fmt/json"
#define MQTT_TOPIC_SAMPLERATE "iot-2/cmd/samplerate/fmt/json"
#define MQTT_TOPIC_DEVICES "iot-2/evt/status/fmt/json"

// Timezone info
#define TZ_OFFSET -5  // (EST) Hours timezone offset to GMT (without daylight saving time)
#define TZ_DST    60  // Minutes timezone offset for Daylight saving

// Add WiFi connection information in config.h
//char ssid[] = "<SSID>";  // your network SSID (name)
//char pass[] = "<PASSWORD>";  // your network password

// MQTT objects
void callback(char* topic, byte* payload, unsigned int length);
WiFiClientSecure wifiClient;
PubSubClient mqtt(MQTT_HOST, MQTT_PORT, callback, wifiClient);

// variables to hold accelerometer data
StaticJsonDocument<100> jsonReceiveDoc;
DynamicJsonDocument jsonDoc(4000);
DynamicJsonDocument jsonTraces(4000);
JsonArray traces = jsonTraces.to<JsonArray>();
static char msg[2000];

// ADXL Accelerometer
void IRAM_ATTR isr_adxl();
void generateMessage(String &out, long fifoData[32][3], int entryCount);
void appendArray(String &out, long fifoData[32][3], int entryCount, int index);

int32_t Adxl355SampleRate = 31;  // Reporting Sample Rate [31,125]

int8_t CHIP_SELECT_PIN_ADXL = 15;
int8_t INT_PIN = 2;
Adxl355::RANGE_VALUES range = Adxl355::RANGE_VALUES::RANGE_2G;
Adxl355::ODR_LPF odr_lpf;
Adxl355::STATUS_VALUES adxstatus;

bool deviceRecognized = false;
long fifoOut[32][3];
int numEntriesFifo = 0;
long runningAverage[3] = {0, 0, 0};
long sumFifo[3];
long fifoDelta[32][3];
bool fifoFull = false;
int fifoCount = 0;
int numValsForAvg = 0;
static int id = 0;

Adxl355 adxl355(CHIP_SELECT_PIN_ADXL);
SPIClass *spi1 = NULL;
double sr = 0;

// --------------------------------------------------------------------------------------------

void IRAM_ATTR isr_adxl()
{
  fifoFull = true;
  //fifoCount++;
}


// Handle subscribed MQTT topics - Alerts and Sample Rate changes
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] : ");
  
  payload[length] = 0; // ensure valid content is zero terminated so can treat as c-string
  Serial.println((char *)payload);
  DeserializationError err = deserializeJson(jsonReceiveDoc, (char *)payload);
  if (err) {
    Serial.print(F("deserializeJson() failed with code ")); 
    Serial.println(err.c_str());
  } else {
    JsonObject cmdData = jsonReceiveDoc.as<JsonObject>();
    if (0 == strcmp(topic, MQTT_TOPIC_ALARM)) {
      // Sound the Buzzer & Blink the LED
      Serial.println("Earthquake Alarm!");

    } else if (0 == strcmp(topic, MQTT_TOPIC_SAMPLERATE)) {
      // Set the ADXL355 Sample Rate
      int32_t NewSampleRate = 0;
      bool  SampleRateChanged = false ;
      
      NewSampleRate = cmdData["SampleRate"].as<int32_t>(); // this form allows you specify the type of the data you want from the JSON object
      if( NewSampleRate == 31 ) {
        // valid
        Adxl355SampleRate = 31;
        SampleRateChanged = true;
        odr_lpf = Adxl355::ODR_LPF::ODR_31_25_AND_7_813;
        sr = 31.25;
      } else if ( NewSampleRate == 125 ) {
        // valid
        Adxl355SampleRate = 125;
        SampleRateChanged = true;
        odr_lpf = Adxl355::ODR_LPF::ODR_125_AND_31_25;
        sr = 125.0;
      } else {
        // invalid - leave the Sample Rate unchanged
      }

      Serial.print("ADXL355 Sample Rate has been changed:");
      Serial.println(Adxl355SampleRate);
      //SampleRateChanged = false;
      DEBUG(SampleRateChanged) ;
      if( SampleRateChanged ) {
        DEBUG("Changing the ADXL355 Sample Rate");
        adxl355.stop();
        delay(1000);
        DEBUG("Restarting");
        adxl355.start();
        delay(1000);

        if (adxl355.isDeviceRecognized()) {
          deviceRecognized = true;
          DEBUG("Initializing sensor");
          adxl355.initializeSensor(range, odr_lpf, debug);
          DEBUG("Calibrating sensor");
          adxl355.calibrateSensor(5, debug);
          DEBUG("ADXL355 Accelerometer activated");
        }
        else {
          DEBUG("Unable to get accelerometer");
        }
        DEBUG("Finished accelerometer configuration");
      }
      jsonReceiveDoc.clear();
    } else {
      Serial.println("Unknown command received");
    }
  }
}


void setup() {
  // Start serial console
  Serial.begin(115200);
  Serial.setTimeout(2000);
  while (!Serial) { }
  Serial.println();
  Serial.println("OpenEEW Sensor Application");

#if OPENEEW_SAMPLE_RATE_125
  odr_lpf = Adxl355::ODR_LPF::ODR_125_AND_31_25;
  sr = 125.0;
#endif

#if OPENEEW_SAMPLE_RATE_31_25
  odr_lpf = Adxl355::ODR_LPF::ODR_31_25_AND_7_813;
  sr = 31.25;
#endif

  pinMode(INT_PIN, INPUT);
  pinMode(CHIP_SELECT_PIN_ADXL, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), isr_adxl, FALLING);

  spi1 = new SPIClass(HSPI);
  adxl355.initSPI(*spi1);

  // Start WiFi connection
  WiFi.mode(WIFI_STA);
  byte mac[6];                     // the MAC address of your Wifi shield
  WiFi.macAddress(mac);
  
  // Output this ESP32 Unique Mac Address
  Serial.print("MAC: "); 
  char macAddress[18];
  sprintf(macAddress,"%02X:%02X:%02X:%02X:%02X:%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Serial.println(macAddress);
  // Use the reverse octet Mac Address as the MQTT deviceId
  sprintf(deviceId,"%02X%02X%02X%02X%02X%02X",mac[5],mac[4],mac[3],mac[2],mac[1],mac[0]);
  Serial.println(deviceId);
  
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected");

  // Set time from NTP servers
  configTime(TZ_OFFSET * 3600, TZ_DST * 60, "pool.ntp.org", "0.pool.ntp.org");
  Serial.println("\nWaiting for time");
  while(time(nullptr) <= 100000) {
    Serial.print(".");
    delay(100);
  }
  unsigned timeout = 5000;
  unsigned start = millis();
  while (millis() - start < timeout) {
      time_t now = time(nullptr);
      if (now > (2018 - 1970) * 365 * 24 * 3600) {
          break;
      }
      delay(100);
  }
  delay(1000); // Wait for time to fully sync
  Serial.println("Time sync'd");
  time_t now = time(nullptr);
  Serial.println(ctime(&now));

  // Dynamically build the MQTT Device ID from the Mac Address of this ESP32
  sprintf(MQTT_DEVICEID,"d:%s:%s:%02X%02X%02X%02X%02X%02X",MQTT_ORGID,MQTT_DEVICETYPE,mac[5],mac[4],mac[3],mac[2],mac[1],mac[0]);
  Serial.println(MQTT_DEVICEID);
  
  sprintf(MQTT_HOST,"%s.messaging.internetofthings.ibmcloud.com",MQTT_ORGID);

  char mqttparams[100]; // Allocate a buffer large enough for this string ~95 chars
  sprintf(mqttparams, "MQTT_USER:%s  MQTT_TOKEN:%s  MQTT_DEVICEID:%s", MQTT_USER, MQTT_TOKEN, MQTT_DEVICEID);
  Serial.println(mqttparams);
  
  // Connect to MQTT - IBM Watson IoT Platform
  while(! mqtt.connected()){
    if (mqtt.connect(MQTT_DEVICEID, MQTT_USER, MQTT_TOKEN)) { // Token Authentication
    // if (mqtt.connect(MQTT_DEVICEID)) { // No Token Authentication
      Serial.println("MQTT Connected");
      mqtt.subscribe(MQTT_TOPIC_ALARM);
      mqtt.subscribe(MQTT_TOPIC_SAMPLERATE);
      mqtt.setBufferSize(2000);
    } else {
      Serial.println("MQTT Failed to connect! ... retrying");
      delay(500);
    }
  }

  // Announce this device is connected
  // Not a requirement for Watson IoT, tell OpenEEW the location of this device
  JsonObject payload = jsonDoc.to<JsonObject>();
  payload["device_id"] = deviceId;
  payload["time"] = ctime(&now);
  // payload["location"] = // {gps_lat,gps_lon}
  serializeJson(jsonDoc, msg, 2000);
  Serial.println(msg);

  if (!mqtt.publish(MQTT_TOPIC_DEVICES, msg)) {
    Serial.println("MQTT Publish failed");
  }
  jsonDoc.clear();

  adxl355.start();
  delay(1000);

  if (adxl355.isDeviceRecognized()) {
    deviceRecognized = true;
    DEBUG("Initializing sensor");
    adxl355.initializeSensor(range, odr_lpf, debug);
    DEBUG("Calibrating sensor");
    adxl355.calibrateSensor(5, debug);
    DEBUG("ADXL355 Accelerometer activated");
  }
  else {
    DEBUG("Unable to get accelerometer");
  }
  DEBUG("Finished accelerometer configuration");
}


void loop() {
  mqtt.loop();
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to re-connect to IBM Watson IoT Platform
    if( mqtt.connect(MQTT_DEVICEID, MQTT_USER, MQTT_TOKEN) ) {
  //if( mqtt.connect(MQTT_DEVICEID) ) { // No Token Authentication      
      Serial.println("MQTT Connected");
      mqtt.subscribe(MQTT_TOPIC_ALARM);
      mqtt.subscribe(MQTT_TOPIC_SAMPLERATE);
      mqtt.setBufferSize(2000);
      mqtt.loop();
    } else {
      Serial.println("MQTT Failed to connect!");
      delay(5000);
    }
  }
  
  //====================== ADXL Accelerometer =====================
  if (fifoFull)  {
    fifoFull = false;
    adxstatus = adxl355.getStatus();
    
    if (adxstatus & Adxl355::STATUS_VALUES::FIFO_FULL) {
      if (-1 != (numEntriesFifo = adxl355.readFifoEntries((long *)fifoOut))) {
        sumFifo[0] = 0;
        sumFifo[1] = 0;
        sumFifo[2] = 0;

        for (int i = 0; i < 32; i++) {
          for (int j = 0; j < 3; j++) {
            fifoDelta[i][j] = fifoOut[i][j] - runningAverage[j];
            sumFifo[j] += fifoOut[i][j];
          }
        }

        for (int j = 0; j < 3; j++) {
          runningAverage[j] = (numValsForAvg * runningAverage[j] + sumFifo[j]) / (numValsForAvg + 32);
        }

        numValsForAvg = min(numValsForAvg + 32, 2000);

        // Generate an array of json objects that contain x,y,z arrays of 32 floats.
        // [{"x":[],"y":[],"z":[]},{"x":[],"y":[],"z":[]}]
        JsonObject acceleration = traces.createNestedObject();
        
        // [{"x":[9.479,0],"y":[0.128,-1.113],"z":[-0.185,123.321]},{"x":[9.479,0],"y":[0.128,-1.113],"z":[-0.185,123.321]}]
        double gal;
        for (int i = 0; i < numEntriesFifo; i++) {
          gal = adxl355.valueToGals(fifoDelta[i][0]);
          acceleration["x"].add(round(gal*1000)/1000);
          
          gal = adxl355.valueToGals(fifoDelta[i][1]);
          acceleration["y"].add(round(gal*1000)/1000);

          gal = adxl355.valueToGals(fifoDelta[i][2]);
          acceleration["z"].add(round(gal*1000)/1000);
        }
        //serializeJson(traces, Serial);
        //DEBUG("");
        fifoCount++;

        if (fifoCount < MAX_FIFO_COUNT) {
          // Collect two samples
        } else {
          // variables to hold accelerometer data
          // DynamicJsonDocument is stored on the heap
          JsonObject payload = jsonDoc.to<JsonObject>();
          JsonObject status = payload.createNestedObject("d");

          // Load the key/value pairs into the serialized ArduinoJSON format
          status["device_id"] = deviceId;
          status["msgId"] = id;
          status["traces"] = traces;

          // Serialize the entire string to be transmitted
          serializeJson(jsonDoc, msg, 2000);
          Serial.println(msg);

          // Publish the message to MQTT Broker
          if (!mqtt.publish(MQTT_TOPIC, msg)) {
            Serial.println("MQTT Publish failed");
          }
          
          // Reset fifoCount and fifoMessage
          fifoCount = 0;
          id++;
          jsonDoc.clear();
          jsonTraces.clear();
          traces = jsonTraces.to<JsonArray>();
        }
      }
    }
  }
}
