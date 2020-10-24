const char *deviceId = "sensor_001";
const char *MQTTBrokerIp = "192.168.0.11";
const int MQTTBrokerPort = 1833;

#define SAMPLE_RATE_125 true
#define SAMPLE_RATE_31_25 false

#define debug true

#define LOG_L2 false //Needs to be true to have logging about deep wifi info

#define DEBUG_IL(x) \
    if (debug)      \
        Serial.print(x);
#define DEBUG(x) \
    if (debug)   \
        Serial.println(x);

#define DEBUG_IL_L2(x) \
    if (LOG_L2)        \
        Serial.print(x);
#define DEBUG_L2(x) \
    if (LOG_L2)     \
        Serial.println(x);

#define SAMPLE_RATE_125 true
#define SAMPLE_RATE_31_25 false

#define CONNECTION_TO 6000    //ms
#define RECONNECTION_TO 10000 //ms

#define MAX_FIFO_COUNT 2

// const char * udpDestination = "192.168.1.80";
// const int udpPort = 5001;
