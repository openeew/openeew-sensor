#define OPENEEW_SAMPLE_RATE_125 false
#define OPENEEW_SAMPLE_RATE_31_25 true

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

#define CONNECTION_TO 6000    //ms
#define RECONNECTION_TO 10000 //ms

#define PRODUCTION_BOARD 1
