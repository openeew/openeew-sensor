#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ETH.h>
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
char deviceID[13];
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

// ETH_CLOCK_GPIO17_OUT - 50MHz clock from internal APLL inverted output on GPIO17 - tested with LAN8720
#ifdef ETH_CLK_MODE
#undef ETH_CLK_MODE
#endif
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
// Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_POWER_PIN   -1
// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE        ETH_PHY_LAN8720
// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR        0
// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN     23
// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN    18

static bool eth_connected = false;

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
int  numEntriesFifo = 0;
long runningAverage[3] = {0, 0, 0};
long sumFifo[3];
long fifoDelta[32][3];
bool fifoFull = false;
int  fifoCount = 0;
int  numValsForAvg = 0;
static int id = 0;

Adxl355 adxl355(CHIP_SELECT_PIN_ADXL);
SPIClass *spi1 = NULL;
double sr = 0;

// --------------------------------------------------------------------------------------------
void IRAM_ATTR isr_adxl() {
  fifoFull = true;
  //fifoCount++;
}

void StartADXL355() {
  // odr_lpf is a global
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


// Handle subscribed MQTT topics - Alerts and Sample Rate changes
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] : ");

  payload[length] = 0; // ensure valid content is zero terminated so can treat as c-string
  Serial.println((char *)payload);
  DeserializationError err = deserializeJson(jsonReceiveDoc, (char *)payload);
  if (err) {
    Serial.print(F("deserializeJson() failed with code : "));
    Serial.println(err.c_str());
  } else {
    JsonObject cmdData = jsonReceiveDoc.as<JsonObject>();
    if ( strcmp(topic, MQTT_TOPIC_ALARM) == 0 ) {
      // Sound the Buzzer & Blink the LED
      Serial.println("Earthquake Alarm!");

    } else if ( strcmp(topic, MQTT_TOPIC_SAMPLERATE) == 0 ) {
      // Set the ADXL355 Sample Rate
      int32_t NewSampleRate = 0;
      bool    SampleRateChanged = false ;

      NewSampleRate = cmdData["SampleRate"].as<int32_t>(); // this form allows you specify the type of the data you want from the JSON object
      if( NewSampleRate == 31 ) {
        // Requested sample rate of 31 is valid
        Adxl355SampleRate = 31;
        SampleRateChanged = true;
        odr_lpf = Adxl355::ODR_LPF::ODR_31_25_AND_7_813;
        sr = 31.25;
      } else if ( NewSampleRate == 125 ) {
        // Requested sample rate of 125 is valid
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
        StartADXL355();
      }
      jsonReceiveDoc.clear();
    } else {
      Serial.println("Unknown command received");
    }
  }
}

void Connect2MQTTbroker() {
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect / re-connect to IBM Watson IoT Platform
    // These are globals set in setup()
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
}

void NetworkEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_WIFI_READY:    // 0
      Serial.println("ESP32 WiFi ready");
      break;
    case SYSTEM_EVENT_STA_START:     // 2
      Serial.println("ESP32 station start");
      break;
    case SYSTEM_EVENT_STA_CONNECTED: // 4
      Serial.println("ESP32 station connected to AP");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:    // 7
      Serial.println("ESP32 station got IP from connected AP");
      if( !eth_connected ) {
        Serial.println("Should I connect to the MQTT broker here?");
      }
      break;
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth / wifi hostname here
      ETH.setHostname( "openeew-sensor-eth" );
      WiFi.setHostname("openeew-sensor-wifi");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      Serial.print("ETH MAC: ");
      Serial.println(ETH.macAddress());
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;

      // Switch the MQTT connection to Ethernet from WiFi (or initially)
      // Preference the Ethernet wired interence if its available
      // Disconnect the MQTT session
      if( mqtt.connected() ){
        mqtt.disconnect();
        // Handled at a lower level?
        // mqtt.setClient(ETH); // Fails. wifiClient might still be valid
        Connect2MQTTbroker();
      }
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      // Disconnect the MQTT client
      if( mqtt.connected() ){
        mqtt.disconnect();
      }
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      Serial.print("Unhandled Network Interface event : ");
      Serial.println(event);
      break;
  }
}


// MQTT SSL requires a relatively accurate time between broker and client
void SetTimeESP32() {
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
}


void setup() {
  // Start serial console
  Serial.begin(115200);
  Serial.setTimeout(2000);
  while (!Serial) { }
  Serial.println();
  Serial.println("OpenEEW Sensor Application");

  // Start WiFi connection
  WiFi.onEvent(NetworkEvent);
  WiFi.mode(WIFI_STA);
  byte mac[6];                     // the MAC address of your Wifi shield
  WiFi.macAddress(mac);

  // Output this ESP32 Unique WiFi MAC Address
  Serial.print("WiFi MAC: ");
  Serial.println(WiFi.macAddress());

  // Start the ETH interface
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
  Serial.print("ETH  MAC: ");
  Serial.println(ETH.macAddress());

  // Use the reverse octet Mac Address as the MQTT deviceID
  sprintf(deviceID,"%02X%02X%02X%02X%02X%02X",mac[5],mac[4],mac[3],mac[2],mac[1],mac[0]);
  Serial.println(deviceID);

  // Dynamically build the MQTT Device ID from the Mac Address of this ESP32
  sprintf(MQTT_DEVICEID,"d:%s:%s:%02X%02X%02X%02X%02X%02X",MQTT_ORGID,MQTT_DEVICETYPE,mac[5],mac[4],mac[3],mac[2],mac[1],mac[0]);
  Serial.println(MQTT_DEVICEID);

  sprintf(MQTT_HOST,"%s.messaging.internetofthings.ibmcloud.com",MQTT_ORGID);

  char mqttparams[100]; // Allocate a buffer large enough for this string ~95 chars
  sprintf(mqttparams, "MQTT_USER:%s  MQTT_TOKEN:%s  MQTT_DEVICEID:%s", MQTT_USER, MQTT_TOKEN, MQTT_DEVICEID);
  Serial.println(mqttparams);

  WiFi.begin(ssid, pass);
  while( (WiFi.status() != WL_CONNECTED) ) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected");

  // Set the time on the ESP32
  SetTimeESP32();

  // Connect to MQTT - IBM Watson IoT Platform
  Connect2MQTTbroker();

/*
  // Announce this device is connected
  // Not a requirement for Watson IoT, tell OpenEEW the location of this device
  JsonObject payload = jsonDoc.to<JsonObject>();
  payload["device_id"] = deviceID;
  payload["time"] = ctime(&now);
  // payload["location"] = // {gps_lat,gps_lon}
  serializeJson(jsonDoc, msg, 2000);
  Serial.println(msg);

  if (!mqtt.publish(MQTT_TOPIC_DEVICES, msg)) {
    Serial.println("MQTT Publish failed");
  }
  jsonDoc.clear();
*/

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
  StartADXL355();
}


void loop() {
  mqtt.loop();
  // Confirm Connection to MQTT - IBM Watson IoT Platform
  Connect2MQTTbroker();

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
          status["device_id"] = deviceID;
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
