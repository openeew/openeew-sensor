# OpenEEW firmware


## Operation
In the program setup we check that the accelerometer is present, calibrate it and set the ODR, LPF and RANGE. After that, we wait for the GPS to have reception. Once location is acquired it is put into a location message. Finally the device reads the flash memory for saved networks and scans to see if they match any available. If there's a match, connect, if not, start smart config. If smart config is available and was successful, save the network.

For the loop, the device checks if still connected to WiFi, if not, retry connection. When a PPS signal is present, it interrupts the system, gets the timestamp and starts a micros timer.When the interrupt coming from the ADXL is present, meaning that FIFO is full, the system takes the timestamp, and attaches the micro seconds that passed since the PPS started, giving time accuracy. Then the device reads the FIFO values, puts them into a JSON message and sends them to the udpDestination and udpPort specified in the config file. Multiple FIFOS can be concatenated in a message, number of fifos in a message can be specified in the config file.

For tracking purposes the traces have a consecutive id, this is not intended for a production firmware, their purpose is to count how many traces are sent and received.




## Flashing

### Config.h
In the config.h file two levels of debugging can be set, first "debug" variable needs to be set true to allow serial communication and only basic status lines are part of the output. Second level is set by making LOG_L2 true, this would give specific output on the WiFi events.

The Sample rate needs to be defined by making true either of the 125Hz or 31.25Hz options. Device Id can be edited also, along with the udp port and destination IP.
