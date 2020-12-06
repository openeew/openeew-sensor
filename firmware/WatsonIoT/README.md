# OpenEEW firmware


## Operation
In the program setup we check that the accelerometer is present, calibrate it and set the ODR, LPF and RANGE. The device reads the flash memory for saved networks and scans to see if they match any available. If there's a match, connect.

For the loop, the device checks if still connected to WiFi, if not, retry connection. When a PPS signal is present, it interrupts the system, gets the timestamp and starts a micros timer.When the interrupt coming from the ADXL is present, meaning that FIFO is full, the system takes the timestamp, and attaches the micro seconds that passed since the PPS started, giving time accuracy. Then the device reads the FIFO values, puts them into a JSON message and sends them to the udpDestination and udpPort specified in the config file. Multiple FIFOS can be concatenated in a message, number of fifos in a message can be specified in the config file.

For tracking purposes the traces have a consecutive id, this is not intended for a production firmware, their purpose is to count how many traces are sent and received.

## Watson IoT

### Instructions

This example can run using an IBM Cloud Lite account.
- Create an [IBM Cloud](http://cloud.ibm.com/registration)
- Login into IBM Cloud
- Create a [Watson IoT Platform](https://cloud.ibm.com/catalog/services/internet-of-things-platform) service instance
- Log into the [Watson IoT Platform](https://internetofthings.ibmcloud.com/)
  - Create a Device Type named **OpenEEW**
  - Create a Device with a Device ID unique to the OpenEEW sensor MAC Address you plan to install
- Add the Watson OrgID and secret authentication token to ```src/main.cpp```
- Add the WiFi ssid and passwd to ```src/config.h```

## Flash a new device

### Setup
You will need an FTDI device to program the device. Something like [this](https://www.aliexpress.com/item/32826575637.html?spm=a2g0o.productlist.0.0.20ef16282CTwNw&algo_pvid=97db3d99-6904-47b4-b90d-4787bd56682d&algo_expid=97db3d99-6904-47b4-b90d-4787bd56682d-5&btsid=0ab6fb8815972923937521550efbde&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_) should work (please note I have not tested this product).

### Install PlatformIO

Follow this guide to [install PlatformIO](https://docs.platformio.org/en/latest/integration/ide/vscode.html#installation) on your machine. PlaformIO offers several benefits to the Arduino IDE, particularly the ability to contain dependencies within a simple folder structure.

### Open project
Inside VSCode go to PlaformIO home, which is available on the bottom toolbar, and select `Projects`, then `Open Project`. Navigate to the root folder where you cloned this repository and open.

### Upload to an OpenEEW sensor
Build the project using the check mark on the bottom toolbar, then upload using the arrow button adjacent to it. The IDE should automatically detect the board of your connnected OpenEEW sensor and start to write the new firmware.

To add the certificates and other contents of the `data` folder to the SPIFFS memory, you need to
open tasks in the PlaformIO menubar on the left, and select `Upload FileSystem Image`:
![](/images/platformio-spiffs.png)

Alternatively, run this command from the directory which contains min_spiffs.csv
```
platformio run --target uploadfs
```

### Config.h
In the config.h file two levels of debugging can be set, first "debug" variable needs to be set true to allow serial communication and only basic status lines are part of the output. Second level is set by making LOG_L2 true, this would give specific output on the WiFi events.

The Sample rate needs to be defined by making true either of the 125Hz or 31.25Hz options. Device Id can be edited also, along with the udp port and destination IP.
