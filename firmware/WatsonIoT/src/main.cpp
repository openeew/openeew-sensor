#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ETH.h>
#include <time.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Adxl355.h>
#include <math.h>
#include <esp_https_ota.h>
#include "config.h"
#include "semver.h"  // from https://github.com/h2non/semver.c

// --------------------------------------------------------------------------------------------
//        UPDATE CONFIGURATION TO MATCH YOUR ENVIRONMENT
// --------------------------------------------------------------------------------------------
#define OPENEEW_ACTIVATION_ENDPOINT "https://openeew-earthquakes.mybluemix.net/activation?ver=1"
#define OPENEEW_FIRMWARE_VERSION    "1.0.0"

// Watson IoT connection details
static char MQTT_HOST[48];            // ORGID.messaging.internetofthings.ibmcloud.com
static char MQTT_DEVICEID[30];        // Allocate a buffer large enough for "d:orgid:devicetype:deviceid"
static char MQTT_ORGID[7];            // Watson IoT 6 character orgid
#define MQTT_PORT        8883         // Secure MQTT 8883 / Insecure MQTT 1833
#define MQTT_TOKEN       "OpenEEW-sens0r"   // Watson IoT DeviceId authentication token
#define MQTT_DEVICETYPE  "OpenEEW"    // Watson IoT DeviceType
#define MQTT_USER        "use-token-auth"
#define MQTT_TOPIC       "iot-2/evt/status/fmt/json"
#define MQTT_TOPIC_ALARM "iot-2/cmd/earthquake/fmt/json"
#define MQTT_TOPIC_SAMPLERATE "iot-2/cmd/samplerate/fmt/json"
char deviceID[13];

#define MYBLUEMIX_PEM                                      \
"-----BEGIN CERTIFICATE----- \r\n"  \
"MIIHOTCCBiGgAwIBAgIQAf75+k140Eka/Iwmx5OFeTANBgkqhkiG9w0BAQsFADBN\r\n"  \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMScwJQYDVQQDEx5E\r\n"  \
"aWdpQ2VydCBTSEEyIFNlY3VyZSBTZXJ2ZXIgQ0EwHhcNMjAwNDExMDAwMDAwWhcN\r\n"  \
"MjIwNzE0MTIwMDAwWjCBgTELMAkGA1UEBhMCVVMxETAPBgNVBAgTCE5ldyBZb3Jr\r\n"  \
"MQ8wDQYDVQQHEwZBcm1vbmsxNDAyBgNVBAoTK0ludGVybmF0aW9uYWwgQnVzaW5l\r\n"  \
"c3MgTWFjaGluZXMgQ29ycG9yYXRpb24xGDAWBgNVBAMMDyoubXlibHVlbWl4Lm5l\r\n"  \
"dDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMmMQwfczgNeG1NeF++p\r\n"  \
"9LhODnAawubHb9Zh3H7fBklbImq5s0K1WMQy2ZemETP83aii3dEDAUtfHQ8uP2If\r\n"  \
"zh2SFg7AEvkXQjg9XERONbC9muLCJ3XIF6vIWLpgI+Ojq+cIRAOgaLR5xyc8V1DR\r\n"  \
"gFFJ9n/zMTqUkKna7R0IeQqJOth1f5nJom2+m0FVXeKd2nwAVC9c9DlQ4BZMveyM\r\n"  \
"Je/dnCqrJNx4xHgU9k7z/fZ0KOj1aJOnzYWpTDtncyAdr3Q/PiWItTsiy9IgLYt7\r\n"  \
"/wCWWUMSVnuPMyAHR42iAHjMOQLQxQnNcnp+GOUcH4KiR8YHo5FSJtv+LxCxMEme\r\n"  \
"uokCAwEAAaOCA94wggPaMB8GA1UdIwQYMBaAFA+AYRyCMWHVLyjnjUY4tCzhxtni\r\n"  \
"MB0GA1UdDgQWBBTvEX9mc1YBJTOgsByB7AiD2HYyTjApBgNVHREEIjAggg8qLm15\r\n"  \
"Ymx1ZW1peC5uZXSCDW15Ymx1ZW1peC5uZXQwDgYDVR0PAQH/BAQDAgWgMB0GA1Ud\r\n"  \
"JQQWMBQGCCsGAQUFBwMBBggrBgEFBQcDAjBrBgNVHR8EZDBiMC+gLaArhilodHRw\r\n"  \
"Oi8vY3JsMy5kaWdpY2VydC5jb20vc3NjYS1zaGEyLWc2LmNybDAvoC2gK4YpaHR0\r\n"  \
"cDovL2NybDQuZGlnaWNlcnQuY29tL3NzY2Etc2hhMi1nNi5jcmwwTAYDVR0gBEUw\r\n"  \
"QzA3BglghkgBhv1sAQEwKjAoBggrBgEFBQcCARYcaHR0cHM6Ly93d3cuZGlnaWNl\r\n"  \
"cnQuY29tL0NQUzAIBgZngQwBAgIwfAYIKwYBBQUHAQEEcDBuMCQGCCsGAQUFBzAB\r\n"  \
"hhhodHRwOi8vb2NzcC5kaWdpY2VydC5jb20wRgYIKwYBBQUHMAKGOmh0dHA6Ly9j\r\n"  \
"YWNlcnRzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydFNIQTJTZWN1cmVTZXJ2ZXJDQS5j\r\n"  \
"cnQwDAYDVR0TAQH/BAIwADCCAfUGCisGAQQB1nkCBAIEggHlBIIB4QHfAHYApLkJ\r\n"  \
"kLQYWBSHuxOizGdwCjw1mAT5G9+443fNDsgN3BAAAAFxZpGDjwAABAMARzBFAiEA\r\n"  \
"1thufVlU62LWP3zPJjLOa+Er6SKGUlztCGEkNFmDR0gCIAPZWiu4e7kmfoFHexrU\r\n"  \
"Yy1p2MG5ew/Y82kQXpuhcM1qAHYAIkVFB1lVJFaWP6Ev8fdthuAjJmOtwEt/XcaD\r\n"  \
"XG7iDwIAAAFxZpGDywAABAMARzBFAiBqVTHJp4rl6vLheLDdn93Hs1RESGFXrP6I\r\n"  \
"x0tcb9aaqAIhAL6KsaeStsUkgdfWk7yLZ1wDS2aIspDUYdeyZg6WNMWgAHUAu9nf\r\n"  \
"vB+KcbWTlCOXqpJ7RzhXlQqrUugakJZkNo4e0YUAAAFxZpGDnQAABAMARjBEAiBR\r\n"  \
"hnp6teLVf4t7dG9bLKVht8xxKbobF0cmKxN0BCW9SwIgPlCu4GqM6C+sRBsHjayO\r\n"  \
"VrSVNcAzopUqf9Vb1VZTwysAdgBByMqx3yJGShDGoToJQodeTjGLGwPr60vHaPCQ\r\n"  \
"YpYG9gAAAXFmkYORAAAEAwBHMEUCIQDmsOIqY05G23bz4l+Fj5tFM8JzH2bV1biF\r\n"  \
"TrCRQKj8VgIgN+jr9st1aLmtI9sp1US7cHTgzGz242n8Oq1e2fc1lz4wDQYJKoZI\r\n"  \
"hvcNAQELBQADggEBAAvZTk4F39nw30fFjzn4CPU2hZOR6JMitqDavHDGPmiCAyr+\r\n"  \
"/gs+QQhOhkW8IdFuZ8uvdrQuX/uqWEDgmFQCqglI1ojm0j/LqOpAVmugo7QpUwi9\r\n"  \
"pZ0U9/8atrvRxjaoykQXEKyrYkYuAKXdYgo2zpCXhTtVcqfDW27/3FO2Ydd6oXwi\r\n"  \
"d4xZbBID86GXSf0ri4wOTT+7U1kCzeFSoViVgZiRgBkJRqcZtxxj4C1G/aDfcm09\r\n"  \
"tZxKOotOoR7JvYRw50t/yW+Gs9ailgquXULv2R6mwV7f/WixJIxJt0yv85Dn7mgI\r\n"  \
"0lebhyIGOv+jUOaKn5s/ltGYLrAE/QM3GkGO704=\r\n"  \
"-----END CERTIFICATE----- \r\n"                \
"-----BEGIN CERTIFICATE----- \r\n" \
"MIIElDCCA3ygAwIBAgIQAf2j627KdciIQ4tyS8+8kTANBgkqhkiG9w0BAQsFADBhMQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBDQTAeFw0xMzAzMDgxMjAwMDBaFw0yMzAzMDgxMjAwMDBaME0xCzAJBgNVBAYTAlVTMRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxJzAlBgNVBAMTHkRpZ2lDZXJ0IFNIQTIgU2VjdXJlIFNlcnZlciBDQTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBANyuWJBNwcQwFZA1W248ghX1LFy949v/cUP6ZCWA1O4Yok3wZtAKc24RmDYXZK83nf36QYSvx6+M/hpzTc8zl5CilodTgyu5pnVILR1WN3vaMTIa16yrBvSqXUu3R0bdKpPDkC55gIDvEwRqFDu1m5K+wgdlTvza/P96rtxcflUxDOg5B6TXvi/TC2rSsd9f/ld0Uzs1gN2ujkSYs58O09rg1/RrKatEp0tYhG2SS4HD2nOLEpdIkARFdRrdNzGXkujNVA075ME/OV4uuPNcfhCOhkEAjUVmR7ChZc6gqikJTvOX6+guqw9ypzAO+sf0/RR3w6RbKFfCs/mC/bdFWJsCAwEAAaOCAVowggFWMBIGA1UdEwEB/wQIMAYBAf8CAQAwDgYDVR0PAQH/BAQDAgGGMDQGCCsGAQUFBwEBBCgwJjAkBggrBgEFBQcwAYYYaHR0cDovL29jc3AuZGlnaWNlcnQuY29tMHsGA1UdHwR0MHIwN6A1oDOGMWh0dHA6Ly9jcmwzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RDQS5jcmwwN6A1oDOGMWh0dHA6Ly9jcmw0LmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RDQS5jcmwwPQYDVR0gBDYwNDAyBgRVHSAAMCowKAYIKwYBBQUHAgEWHGh0dHBzOi8vd3d3LmRpZ2ljZXJ0LmNvbS9DUFMwHQYDVR0OBBYEFA+AYRyCMWHVLyjnjUY4tCzhxtniMB8GA1UdIwQYMBaAFAPeUDVW0Uy7ZvCj4hsbw5eyPdFVMA0GCSqGSIb3DQEBCwUAA4IBAQAjPt9L0jFCpbZ+QlwaRMxp0Wi0XUvgBCFsS+JtzLHgl4+mUwnNqipl5TlPHoOlblyYoiQm5vuh7ZPHLgLGTUq/sELfeNqzqPlt/yGFUzZgTHbO7Djc1lGA8MXW5dRNJ2Srm8c+cftIl7gzbckTB+6WohsYFfZcTEDts8Ls/3HB40f/1LkAtDdC2iDJ6m6K7hQGrn2iWZiIqBtvLfTyyRRfJs8sjX7tN8Cp1Tm5gr8ZDOo0rwAhaPitc+LJMto4JQtV05od8GiG7S5BNO98pVAdvzr508EIDObtHopYJeS4d60tbvVS3bR0j6tJLp07kzQoH3jOlOrHvdPJbRzeXDLz\r\n" \
"-----END CERTIFICATE----- \r\n" \
"-----BEGIN CERTIFICATE----- \r\n" \
"MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBhMQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBDQTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVTMRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5jb20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsBCSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7PT19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbRTLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUwDQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/EsrhMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJFPnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0lsYSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQkCAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\r\n" \
"-----END CERTIFICATE----- \r\n"

const char MyBlueMixNetPem[] = MYBLUEMIX_PEM;

// Timezone info
#define TZ_OFFSET -5  // (EST) Hours timezone offset to GMT (without daylight saving time)
#define TZ_DST    60  // Minutes timezone offset for Daylight saving

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
#ifdef  PRODUCTION_BOARD
#define ETH_POWER_PIN    2  // Ethernet on production board
#else
#define ETH_POWER_PIN   -1  // Ethernet on prototype board
#endif
// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE        ETH_PHY_LAN8720
// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR        0
// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN     23
// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN    18

// Network variables
Preferences prefs;
String _ssid;    // your network SSID (name) - loaded from NVM
String _pswd;    // your network password    - loaded from NVM
int networksStored;
static bool eth_connected = false;
static bool wificonnected = false;

// variables to hold accelerometer data
StaticJsonDocument<100> jsonReceiveDoc;
DynamicJsonDocument jsonDoc(4000);
DynamicJsonDocument jsonTraces(4000);
JsonArray traces = jsonTraces.to<JsonArray>();
static char msg[2000];

// ADXL Accelerometer
void IRAM_ATTR isr_adxl();

int32_t Adxl355SampleRate = 31;  // Reporting Sample Rate [31,125]

int8_t CHIP_SELECT_PIN_ADXL = 15;
#ifdef  PRODUCTION_BOARD
int8_t ADXL_INT_PIN = 35; // ADXL is on interrupt 35 on production board
#else
int8_t ADXL_INT_PIN = 2;  // ADXL is on interrupt 2 on prototype board
#endif
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
// static int id = 0;

Adxl355 adxl355(CHIP_SELECT_PIN_ADXL);
SPIClass *spi1 = NULL;
double sr = 0;

// --------------------------------------------------------------------------------------------
// SmartConfig
int  numScannedNetworks();
int  numNetworksStored();
void readNetworkStored(int netId);
void storeNetwork(String ssid, String pswd);
bool WiFiScanAndConnect();
bool startSmartConfig();

// --------------------------------------------------------------------------------------------
// NeoPixel LEDs
#include <Adafruit_NeoPixel.h>
#define LED_PIN 16
#define LED_COUNT 3
//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
void NeoPixelStatus( int );

// Map the OpenEEW LED status colors to the Particle Photon status colors
#define LED_OFF           0
#define LED_CONNECTED     1 // Cyan breath
#define LED_FIRMWARE_OTA  2 // Magenta
#define LED_CONNECT_WIFI  3 // Green
#define LED_CONNECT_CLOUD 4 // Cyan fast
#define LED_LISTEN_WIFI   5 // Blue
#define LED_WIFI_OFF      6 // White
#define LED_SAFE_MODE     7 // Magenta breath
#define LED_FIRMWARE_DFU  8 // Yellow
#define LED_ERROR         9 // Red

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
    Serial.println("Initializing sensor");
    adxl355.initializeSensor(range, odr_lpf, debug);
    Serial.println("Calibrating sensor");
    adxl355.calibrateSensor(5, debug);
    Serial.println("ADXL355 Accelerometer activated");
  }
  else {
    Serial.println("Unable to get accelerometer");
  }
  Serial.println("Finished accelerometer configuration");
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
      Serial.println( Adxl355SampleRate );
      //SampleRateChanged = false;
      Serial.println( SampleRateChanged ) ;
      if( SampleRateChanged ) {
        Serial.println("Changing the ADXL355 Sample Rate");
        adxl355.stop();
        delay(1000);
        Serial.println("Restarting");
        StartADXL355();
      }
      jsonReceiveDoc.clear();
    } else {
      Serial.println("Unknown command received");
    }
  }
}


bool FirmwareVersionCheck( char *, String );
bool FirmwareVersionCheck( char *firmware_latest, String firmware_ota_url ) {
  semver_t current_version = {};
  semver_t latest_version = {};
  char VersionCheck[48];
  bool bFirmwareUpdateRequiredOTA = false;

  if (semver_parse(OPENEEW_FIRMWARE_VERSION, &current_version)
    || semver_parse(firmware_latest, &latest_version)) {
    Serial.println("Invalid semver string");
    return false;
  }

  int resolution = semver_compare(latest_version, current_version);

  if (resolution == 0) {
    sprintf(VersionCheck,"Version %s is equal to: %s", firmware_latest, OPENEEW_FIRMWARE_VERSION);
  }
  else if (resolution == -1) {
    sprintf(VersionCheck,"Version %s is lower than: %s", firmware_latest, OPENEEW_FIRMWARE_VERSION);
  }
  else {
    sprintf(VersionCheck,"Version %s is higher than: %s", firmware_latest, OPENEEW_FIRMWARE_VERSION);
    bFirmwareUpdateRequiredOTA = true;
  }
  Serial.println(VersionCheck);

  if( bFirmwareUpdateRequiredOTA ) {
    // OTA upgrade is required
    Serial.println("An OTA upgrade is required. Download the new OpenEEW firmware :");
    Serial.println(firmware_ota_url);
    // Launch an OTA upgrade
    NeoPixelStatus( LED_FIRMWARE_OTA ); // blink magenta

    // Serial.println( MyBlueMixNetPem );

    esp_http_client_config_t config = {0};
    config.url = firmware_ota_url.c_str() ;
    config.cert_pem = MyBlueMixNetPem ;
    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        Serial.println("OTA upgrade downloaded. Restarting...");
        esp_restart();
    } else {
        Serial.println("The OpenEEW OTA firmware upgrade failed : ESP_FAIL");
    }
  }

  // Free allocated memory when we're done
  semver_free(&current_version);
  semver_free(&latest_version);
  return true;
}


void GetGeoCoordinates( float *, float *);
void GetGeoCoordinates( float *latitude, float *longitude) {
  HTTPClient http;
  #define GEOCOORD_APIKEY "9f0acd1eb4c51704c2f4429be20ba4c6"
  http.begin( "http://api.ipstack.com/check?access_key=9f0acd1eb4c51704c2f4429be20ba4c6" );
  int httpResponseCode = http.GET();
  Serial.print("GetGeoCoordinates() ipstack HTTP Response code: ");
  Serial.println(httpResponseCode);
  String payload = http.getString();
  http.end();  // free resources
  Serial.print("ipstack HTTP get response payload: ");
  Serial.println( payload );

  if( httpResponseCode == 200 ) {  // Success
    DynamicJsonDocument ReceiveDoc(900);
    DeserializationError err = deserializeJson(ReceiveDoc, payload);
    if (err) {
      Serial.print(F("deserializeJson() failed with code : "));
      Serial.println(err.c_str());
    } else {
      JsonObject GeoCoordData =  ReceiveDoc.as<JsonObject>();
      *latitude  = GeoCoordData["latitude"];
      *longitude = GeoCoordData["longitude"];
    }
  }
}


// Call the OpenEEW Device Activation endpoint to retrieve MQTT OrgID
bool OpenEEWDeviceActivation();
bool OpenEEWDeviceActivation() {
  // OPENEEW_ACTIVATION_ENDPOINT "https://openeew-earthquakes.mybluemix.net/activation?ver=1"
  // $ curl -i  -X POST -d '{"macaddress":"112233445566","lat":40,"lng":-74,"firmware_device":"1.0.0"}'
  //    -H "Content-type: application/JSON" https://openeew-earthquakes.mybluemix.net/activation?ver=1
  Serial.println("Contacting the OpenEEW Device Activation Endpoint :");
  Serial.println(OPENEEW_ACTIVATION_ENDPOINT);

  float lat, lng ;
  GetGeoCoordinates( &lat, &lng );
  Serial.print("GetGeoCoordinates() reported latitude,longitude : ");
  Serial.print(lat,5);
  Serial.print(",");
  Serial.println(lng,5);

  HTTPClient http;
  // Domain name with URL path or IP address with path
  http.begin( OPENEEW_ACTIVATION_ENDPOINT );

   // HTTP request with a content type: application/json
  http.addHeader("Content-Type", "application/json");

  // Construct the serialized http request body
  // '{"macaddress":"112233445566","lat":40.00000,"lng":-74.00000,"firmware_device":"1.0.0"}'
  DynamicJsonDocument httpSendDoc(120);
  String httpRequestData;
  httpSendDoc["macaddress"] = deviceID;
  httpSendDoc["lat"] = lat;
  httpSendDoc["lng"] = lng;
  httpSendDoc["firmware_device"] = OPENEEW_FIRMWARE_VERSION;
  // Serialize the entire string to be transmitted
  serializeJson(httpSendDoc, httpRequestData);
  Serial.print("Sending Device Activation : ");
  Serial.println(httpRequestData);

  int httpResponseCode = http.POST(httpRequestData);

  Serial.print("HTTP Response code: ");
  Serial.println(httpResponseCode);
  if( httpResponseCode == 200 ) {  // Success
    // Get the response payload
    // Ex {"org":"5yrusp","firmware_latest":"1.1.0","firmware_ota_url":"https://download.firmware.com/openeew.bin"}
    String payload = http.getString();
    Serial.print("HTTP post response payload: ");
    Serial.println( payload );
    http.end();  // free resources

    DynamicJsonDocument ReceiveDoc(200);
    DeserializationError err = deserializeJson(ReceiveDoc, payload );
    if (err) {
      Serial.print(F("deserializeJson() failed with code : "));
      Serial.println(err.c_str());
      return false;
    } else {
      JsonObject ActivationData = ReceiveDoc.as<JsonObject>();
      char firmware_latest[10];
      String firmware_ota_url;

      strncpy(MQTT_ORGID, ActivationData["org"], sizeof(MQTT_ORGID) );
      Serial.print("OpenEEW Device Activation directs MQTT data from this sensor to :");
      Serial.println(MQTT_ORGID);

      strncpy(firmware_latest, ActivationData["firmware_latest"], sizeof(firmware_latest) );
      firmware_ota_url = ActivationData["firmware_ota_url"].as<String>();
      FirmwareVersionCheck(firmware_latest, firmware_ota_url);
    }
    return true ;
  } else {        // Failed to successfully contact endpoint
    http.end();   // free resources
    Serial.println("Device Activation failed. Waiting...");
    return false;
  }
}


void Connect2MQTTbroker() {
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    NeoPixelStatus( LED_CONNECT_CLOUD ); // blink cyan
    // Attempt to connect / re-connect to IBM Watson IoT Platform
    // These params are globals assigned in setup()
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
      Serial.println("ESP32 WiFi interface ready");
      break;
    case SYSTEM_EVENT_STA_START:     // 2
      Serial.println("ESP32 WiFi started");
      break;
    case SYSTEM_EVENT_SCAN_DONE:
      Serial.println("Completed scan for access points");
      break;
    case SYSTEM_EVENT_STA_CONNECTED: // 4
      Serial.println("ESP32 WiFi connected to AP");
      WiFi.setHostname("openeew-sensor-wifi");
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi access point");
      wificonnected = false;
      break;
    case SYSTEM_EVENT_STA_GOT_IP:    // 7
      Serial.println("ESP32 station got IP from connected AP");
      Serial.print("Obtained IP address: ");
      Serial.println( WiFi.localIP() );
      if( eth_connected ) {
        Serial.println("Ethernet is already connected");
      }
      break;
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth / wifi hostname here
      ETH.setHostname( "openeew-sensor-eth" );
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
    case SYSTEM_EVENT_STA_STOP:
      Serial.println("WiFi Stopped");
      NeoPixelStatus( LED_WIFI_OFF ); // White
      break;
    case SYSTEM_EVENT_AP_STOP:
      Serial.println("ESP32 soft-AP stop");
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      Serial.println("a station connected to ESP32 soft-AP");
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
    NeoPixelStatus( LED_LISTEN_WIFI ); // blink blue
    Serial.print(".");
    delay(100);
  }
  unsigned timeout = 5000;
  unsigned start = millis();
  while (millis() - start < timeout) {
      time_t now = time(nullptr);
      if (now > (2019 - 1970) * 365 * 24 * 3600) {
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

  strip.setBrightness(130);  // Dim the LED to 50% - 0 off, 255 full bright

  // Start WiFi connection
  WiFi.onEvent(NetworkEvent);
  WiFi.mode(WIFI_STA);

  wificonnected = WiFiScanAndConnect();
  if( !wificonnected )  {
    startSmartConfig();
  }
  Serial.println("WiFi Connected");

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
  //sprintf(deviceID,"%02X%02X%02X%02X%02X%02X",mac[5],mac[4],mac[3],mac[2],mac[1],mac[0]);
  sprintf(deviceID,"%02X%02X%02X%02X%02X%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Serial.println(deviceID);

  // Set the time on the ESP32
  SetTimeESP32();

  // Call the Activation endpoint to retrieve this OpenEEW Sensor details
  while( ! OpenEEWDeviceActivation() ) {
    // Loop forever, waiting for activation success
    NeoPixelStatus( LED_CONNECT_CLOUD ); // blink Cyan
    delay(10000);
  }

  // Dynamically build the MQTT Device ID from the Mac Address of this ESP32
  // MQTT_ORGID was retreived by the OpenEEWDeviceActivation() function
  //sprintf(MQTT_DEVICEID,"d:%s:%s:%02X%02X%02X%02X%02X%02X",MQTT_ORGID,MQTT_DEVICETYPE,mac[5],mac[4],mac[3],mac[2],mac[1],mac[0]);
  sprintf(MQTT_DEVICEID,"d:%s:%s:%02X%02X%02X%02X%02X%02X",MQTT_ORGID,MQTT_DEVICETYPE,mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Serial.println(MQTT_DEVICEID);

  sprintf(MQTT_HOST,"%s.messaging.internetofthings.ibmcloud.com",MQTT_ORGID);

  char mqttparams[100]; // Allocate a buffer large enough for this string ~95 chars
  sprintf(mqttparams, "MQTT_USER:%s  MQTT_TOKEN:%s  MQTT_DEVICEID:%s", MQTT_USER, MQTT_TOKEN, MQTT_DEVICEID);
  Serial.println(mqttparams);

  // Connect to MQTT - IBM Watson IoT Platform
  Connect2MQTTbroker();

#if OPENEEW_SAMPLE_RATE_125
  odr_lpf = Adxl355::ODR_LPF::ODR_125_AND_31_25;
  sr = 125.0;
#endif

#if OPENEEW_SAMPLE_RATE_31_25
  odr_lpf = Adxl355::ODR_LPF::ODR_31_25_AND_7_813;
  sr = 31.25;
#endif

  pinMode(ADXL_INT_PIN, INPUT);
  pinMode(CHIP_SELECT_PIN_ADXL, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ADXL_INT_PIN), isr_adxl, FALLING);

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
        //Serial.println("");
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
          // status["msgId"] = id;
          status["traces"] = traces;

          // Serialize the entire string to be transmitted
          serializeJson(jsonDoc, msg, 2000);
          Serial.println(msg);

          // Publish the message to MQTT Broker
          if (!mqtt.publish(MQTT_TOPIC, msg)) {
            Serial.println("MQTT Publish failed");
          } else {
            NeoPixelStatus( LED_CONNECTED ); // Success - blink cyan
          }

          // Reset fifoCount and fifoMessage
          fifoCount = 0;
          // id++;
          jsonDoc.clear();
          jsonTraces.clear();
          traces = jsonTraces.to<JsonArray>();
        }
      }
    }
  }
}


//================================= WiFi Handling ================================
//Scan networks in range and return how many are they.
int numScannedNetworks() {
  int n = WiFi.scanNetworks();
  Serial.println("WiFi Network scan done");
  if (n == 0)  {
    Serial.println("No networks found");
  }
  else {
    Serial.print(n);
    Serial.println(" network(s) found");
//#if LOG_L2
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
      delay(10);
    }
//#endif
  }
  return n;
}

//Return how many networks are stored in the NVM
int numNetworksStored() {
  prefs.begin("networks", true);
  networksStored = prefs.getInt("num_nets");
  Serial.print("Stored networks : ");
  Serial.println(networksStored);
  prefs.end();

  return networksStored;
}

//Each network as an id so reading the network stored with said ID.
void readNetworkStored(int netId)
{
  Serial.println("Reading stored networks from NVM");

  prefs.begin("networks", true);
  String idx;
  idx = "SSID" + (String)netId;
  _ssid = prefs.getString(idx.c_str(), "");
  idx = "key" + (String)netId;
  _pswd = prefs.getString(idx.c_str(), "");
  prefs.end();

  Serial.print("Found network ");
  Serial.print(_ssid);
  Serial.print(" , ");
  DEBUG_L2(_pswd);  // off by default
  Serial.println("xxxxxx");
}

//Save a pair of SSID and PSWD to NVM
void storeNetwork(String ssid, String pswd)
{
  Serial.print("Writing network to NVM: ");
  Serial.print(ssid);
  Serial.print(",");
  Serial.println(pswd);

  prefs.begin("networks", false);
  int aux_num_nets = prefs.getInt("num_nets");
  Serial.print("Stored networks in NVM: ");
  Serial.println(aux_num_nets);
  aux_num_nets++;
  String idx;
  idx = "SSID" + (String)aux_num_nets;
  prefs.putString(idx.c_str(), ssid);
  idx = "key" + (String)aux_num_nets;
  prefs.putString(idx.c_str(), pswd);
  prefs.putInt("num_nets", aux_num_nets);
  prefs.end();
  Serial.print("Device has ");
  Serial.print(aux_num_nets);
  Serial.println(" networks stored in NVM");
}

//Joins the previous functions, gets the stored networks and compares to the available, if there is a match and connects, return true
//if no match or unable to connect, return false.
bool WiFiScanAndConnect()
{
  int num_nets = numNetworksStored();
  int num_scan = numScannedNetworks();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

  for (int i = 1; i < (num_nets + 1); i++) {
    readNetworkStored(i);

    for (int j = 0; j < num_scan; j++) {
      if (_ssid == WiFi.SSID(j)) {
        //Serial.print("Status from connection attempt");
        WiFi.begin(_ssid.c_str(), _pswd.c_str());
        WiFi.setSleep(false);
        unsigned long t0 = millis();

        while (WiFi.status() != WL_CONNECTED && (millis() - t0) < CONNECTION_TO) {
          NeoPixelStatus( LED_LISTEN_WIFI ); // blink blue
          delay(1000);
        }
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("WiFi was successfully connected");
          return true;
        }
        else {
          Serial.println("There was a problem connecting to WiFi");
        }
      }
      else {
        Serial.println("Got no match for network");
      }
    }
  }
  Serial.println("Found no matches for saved networks");
  return false;
}

// Executes the Smart config routine and if connected, will save the network for future use
bool startSmartConfig()
{
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP_STA);
  WiFi.beginSmartConfig();

  // Wait for SmartConfig packet from mobile
  Serial.println("Waiting for SmartConfig.");
  while( !WiFi.smartConfigDone() || eth_connected ) {
    delay(500);
    Serial.print(".");
    NeoPixelStatus( LED_LISTEN_WIFI );  // blink blue
  }

  for( int i=0;i<4;i++){
    delay(500);
    NeoPixelStatus( LED_CONNECT_WIFI ); // Success - blink green
  }

  if( eth_connected ) {
    // Ethernet cable was connected during or before SmartConfig
    // Skip SmartConfig
    return true;
  }
  Serial.println("SmartConfig received.");

  // Wait for WiFi to connect to AP
  Serial.println("Waiting for WiFi");
  unsigned long t0 = millis();
  while( WiFi.status() != WL_CONNECTED && (millis() - t0) < CONNECTION_TO)  {
    delay(500);
    Serial.print(".");
    NeoPixelStatus( LED_LISTEN_WIFI );  // blink blue
  }
  if (WiFi.status() == WL_CONNECTED) {
    _ssid = WiFi.SSID();
    _pswd = WiFi.psk();
    Serial.print("Smart Config done, connected to: ");
    Serial.print(_ssid);
    Serial.print(" with psswd: ");
    Serial.println("xxxxxx");
    DEBUG_L2(_pswd)  // off by default
    storeNetwork(_ssid, _pswd);
    NeoPixelStatus( LED_CONNECT_WIFI ); // Success - blink green
    return true;
  }
  else {
    Serial.println("Something went wrong with SmartConfig");
    return false;
  }
}


void NeoPixelStatus( int status ) {
  // Turn leds off to cause a blink effect
  strip.clear();  // Off
  strip.show(); // This sends the updated pixel color to the hardware.
  delay(400);   // Delay for a period of time (in milliseconds).

  switch( status ) {
    case LED_OFF :
      strip.clear();  // Off
      break;
    case LED_CONNECTED :
      strip.fill( strip.Color(0,255,255), 0, 3);  // Cyan breath
      Serial.println("LED_CONNECTED - Cyan");
      break;
    case LED_FIRMWARE_OTA :
      strip.fill( strip.Color(255,0,255), 0, 3);  // Magenta
      Serial.println("LED_FIRMWARE_OTA - Magenta");
      break;
    case LED_CONNECT_WIFI :
      strip.fill( strip.Color(0,255,0), 0, 3);  // Green
      Serial.println("LED_CONNECT_WIFI - Green");
      break;
    case LED_CONNECT_CLOUD :
      strip.fill( strip.Color(0,255,255), 0, 3);  // Cyan fast
      Serial.println("LED_CONNECT_CLOUD - Cyan");
      break;
    case LED_LISTEN_WIFI :
      strip.fill( strip.Color(0,0,255), 0, 3);  // Blue
      Serial.println("LED_LISTEN_WIFI - Blue");
      break;
    case LED_WIFI_OFF :
      strip.fill( strip.Color(255,255,255), 0, 3);  // White
      Serial.println("LED_WIFI_OFF - White");
      break;
    case LED_SAFE_MODE :
      strip.fill( strip.Color(255,0,255), 0, 3);  // Magenta breath
      Serial.println("LED_SAFE_MODE - Magenta");
      break;
    case LED_FIRMWARE_DFU :
      strip.fill( strip.Color(255,255,0), 0, 3);  // Yellow
      Serial.println("LED_FIRMWARE_DFU - Yellow");
      break;
    case LED_ERROR :
      strip.fill( strip.Color(0,255,0), 0, 3);  // Red
      Serial.println("LED_ERROR - Red");
      break;
    default :
      strip.clear();  // Off
      break;
  }
  strip.show(); // Send the updated pixel color to the hardware
}