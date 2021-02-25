# OpenEEW Firmware and MQTT

This README describes many of the features of the OpenEEW firmware and how it transmits data and receives commands via the MQTT protocol.

## MQTT

[MQTT](https://mqtt.org) is an OASIS standard messaging protocol for the Internet of Things (IoT). It is designed as an extremely lightweight publish / subscribe messaging transport that is ideal for connecting remote devices with a small code footprint and minimal network bandwidth.

### Understanding MQTT Topics

- An MQTT topic is used to filter messages for each connected client. The topic consists of one or more topic levels. Each topic level is separated by a forward slash (topic level separator).
- Read about the Watson IoT Platform opinionated topic hierachy and how to [subscribe / publish commands and events](https://www.ibm.com/support/knowledgecenter/SSQP8H/iot/platform/devices/mqtt.html) to Watson IoT Platform.

## Accelerometer Data

There is only one type of message that the firmware publishes to the MQTT broker. This ESP32 based board reads Accelerometer data from the Analog Devices ADXL355 chip.  The ADXL355 fills its buffer depending on the sample rate.  The firmware loops until the buffer is filled then it reads the data. The data is then stored in an array of json objects that contain x,y,z arrays of 32 floats.

```json
[ {"x":[],"y":[],"z":[]} , {"x":[],"y":[],"z":[]} ]
```

### Format of an example record of accelerometer data

Included in the Json data transmitted to the MQTT broker is the Device ID of the OpenEEW sensor.
Each sensor uses the unique 12 digit hexadecimal MAC address identifier of the WiFi chip to identify itself to the OpenEEW network.

The JSON object of one second of accelerometer data might look like this:

```json
{"d":{"device_id":"A8032A4DD5F0","traces":[{"x":[-0.03,-0.011,-0.019,0,-0.026,-0.023,0.091,-0.049,0.004,0.03,-0.023,-0.004,0.057,0.049,0.023,-0.008,-0.113,0.008,-0.026,-0.023,0.015,-0.026,-0.004,0,0.011,-0.045,0.004,-0.008,-0.124,-0.026,0.041,0],"y":[-0.207,-0.083,-0.147,-0.298,-0.253,-0.256,-0.158,-0.17,-0.147,-0.275,-0.287,-0.207,-0.26,-0.162,-0.241,-0.215,-0.162,-0.223,-0.207,-0.207,-0.219,-0.264,-0.26,-0.226,-0.215,-0.162,-0.181,-0.181,-0.207,-0.256,-0.162,-0.155],"z":[-0.17,-0.204,-0.2,0.011,-0.249,-0.17,-0.057,-0.204,-0.106,0.038,-0.17,-0.196,-0.185,-0.256,-0.079,-0.068,-0.072,-0.128,-0.136,-0.102,-0.091,-0.049,-0.109,-0.143,-0.038,-0.279,-0.147,-0.109,-0.004,-0.151,-0.26,-0.162]}]}}
```

### Publishing an MQTT event to the MQTT Broker

Watson IoT Platform has several types of messages; events, commands, monitoring.
Sending data **from** the sensor **to** the remote MQTT broker is an *event*.
Denoted as a `/evt/`

```c
#define MQTT_TOPIC            "iot-2/evt/status/fmt/json"
```

### Subscribing to the Accelerometer data

Seismologists and data scientists want to study the accelerometer data for potential earthquake indications, often in near real time.

#### MQTT Broker on a local subnet

Listening to the accelerometer data from a MQTT broker on a local subnet is easy.  The mosquitto MQTT Broker can even be running on a Raspberry Pi.

```sh
mosquitto_sub -t "iot-2/evt/+/fmt/json" -h 192.168.1.101 -p 1883 -i "a:listen:data"
```

The client identifier `-i "a:string:string"` is optional when communicating with the mosquitto broker. It can be any string but might be helpful if you are running your mosquitto server in `-v` verbose mode and you're wondering where messages are coming from.

#### Watson IoT Platform MQTT Broker

IoT security is critical when running large distributed networks of IoT sensors.  Sending  unencrypted data across the Internet should be discouraged.  The following command will allow you subscribe. In this case, the `-u`, `-P`, and `-i` are significant for protecting your data.

- `-u` : The API Key can be generated on the Watson IoT web console. By convention, this API key will be `a-<orgid>-10digits`.  For example `a-k55sah-iwh1ollkfr`
- `-P` : The Password token can be generated on the Watson IoT web console. Special characters in the password need to be escaped with a slash on the mosquitto_sub CLI.  If the generated password is `PZD*V!GFA5cEDHQrc)`, you want to specify `PZD*V\!GFA5cEDHQrc\)`
- `-i` : must adhere to a specific format. It needs to be in the format of `a:<orgid>:anystring`. "a:" declares this as an application, followed by the 6 character Watson IoT OrgID, followed by a unique string of your choosing. For example `a:xyz123:walicki`
- If you want to use mosquitto_sub to watch the data, the WIoTP connection security needs to be TLS Optional. The port needs to be 1883. The device can be sending its data via TLS with Token Auth 8883 to the cloud.
- Better would be to turn off TLS Optional, use port 8883 and pass the Watson IoT messaging root certificate pem file.

```sh
mosquitto_sub -t "iot-2/type/OpenEEW/id/+/evt/+/fmt/+" -h k55sah.messaging.internetofthings.ibmcloud.com -p 1883  -u "a-k55sah-i0a6h9p8ea" -P PZD*V\!GFA5cEDHQrc\) -i "a:k55sah:mosquitto"
```

```sh
mosquitto_sub -t "iot-2/type/OpenEEW/id/+/evt/+/fmt/+" -h k55sah.messaging.internetofthings.ibmcloud.com -p 8883 --cafile messaging.pem  -u "a-k55sah-i0a6h9p8ea" -P PZD*V\!GFA5cEDHQrc\) -i "a:k55sah:mosquitto"
```

Now that we've demonstrated the `mosquitto_sub` syntax, I'll move the API Key and Token into environment variables so as to not confuse subsequent examples. That will make readability and your copy/paste easier.  Just set the two env vars and specify your Org ID.

```sh
export WIOTP_APIKEY=a-OrgID-10digits
export WIOTP_TOKEN=<token>
mosquitto_sub -t iot-2/type/+/id/+/evt/+/fmt/+ -h OrgId.messaging.internetofthings.ibmcloud.com -p 8883 --cafile messaging.pem -u $WIOTP_APIKEY -P $WIOTP_TOKEN -i a:OrgId:mosquitto
```

#### Subscribe to all OpenEEW commands sent to your OpenEEW device

Sometimes it is useful to subscribe to and watch all the commands crossing across your MQTT topic space.

##### Local

```sh
mosquitto_sub -t "iot-2/cmd/+/fmt/json" -h localhost -p 1883 -i "a:listen:commands"
```

##### Watson IoT Platform

```sh
mosquitto_sub -t iot-2/type/+/id/+/cmd/+/fmt/json -h OrgId.messaging.internetofthings.ibmcloud.com -p 8883 --cafile messaging.pem -u $WIOTP_APIKEY -P $WIOTP_TOKEN -i a:OrgId:mosquitto
```

##### Notes

Sending Json data via MQTT requires serialization and deserialization of the data.  The firmware uses the ArduinoJson library to marshal the arrays of json objects.

## Controlling the behavior of the device via remote messages

The remote OpenEEW Device Management Dashboards can control the behavior of the devices in the OpenEEW network via MQTT topics.  The firmware running on each device subscribes to these commands. It only receives published MQTT topics that are specificially for it. This is similar to listening for your mother calling just your name on the crowded playground. MQTT handles this with topic namespaces.

## MQTT Topics defined by the OpenEEW firmware

This section describes the various commands that can be sent to the OpenEEW firmware. Some of the commands include parameters which control the firmware behavior.  The section also provides `mosquitto_pub` examples on how to send the commands to your device.

### Snippet from OpenEEW firmware `main.cpp`

```c
#define MQTT_TOPIC_ALARM      "iot-2/cmd/earthquake/fmt/json"
#define MQTT_TOPIC_SAMPLERATE "iot-2/cmd/samplerate/fmt/json"
#define MQTT_TOPIC_FWCHECK    "iot-2/cmd/firmwarecheck/fmt/json"
#define MQTT_TOPIC_SEND10SEC  "iot-2/cmd/10secondhistory/fmt/json"
#define MQTT_TOPIC_SENDACCEL  "iot-2/cmd/sendacceldata/fmt/json"
```

### ALARM

Tell the firmware to make the device blink the LEDs the color red and bleep a warning of an impending earthquake.

#### Local MQTT Broker

```sh
mosquitto_pub -h 192.168.1.101 -t iot-2/cmd/earthquake/fmt/json -i cmd:earthquake -m {Alarm:true}
```

#### Watson IoT Platform

```sh
mosquitto_pub -h OrgID.messaging.internetofthings.ibmcloud.com -p 8883 --cafile messaging.pem -u $WIOTP_APIKEY -P $WIOTP_TOKEN -i "a:OrgID:mosquitto" -t iot-2/type/OpenEEW/id/A8032A4DD5F0/cmd/earthquake/fmt/json  -m {Alarm:true}
```

### SENDACCEL

Tell the firmware to send live accelerometer data to the cloud for some duration of seconds.

In the two subsections below, there are examples which send various amounts of accelerometer data to the cloud:

- 1 second
- 30 seconds
- "Continous mode" - Send the maximum uint32 of 4294967295
- To reset it to silence mode, send 0 seconds

#### Local MQTT Broker

```sh
mosquitto_pub -h 192.168.1.101 -t iot-2/cmd/sendacceldata/fmt/json -i cmd:sendacceldata  -m {LiveDataDuration:1}

mosquitto_pub -h 192.168.1.101 -t iot-2/cmd/sendacceldata/fmt/json -i cmd:sendacceldata  -m {LiveDataDuration:30}

mosquitto_pub -h 192.168.1.101 -t iot-2/cmd/sendacceldata/fmt/json -i cmd:sendacceldata  -m {LiveDataDuration:4294967295}

mosquitto_pub -h 192.168.1.101 -t iot-2/cmd/sendacceldata/fmt/json -i cmd:sendacceldata  -m {LiveDataDuration:0}
```

#### Watson IoT Platform

```sh
mosquitto_pub -h OrgID.messaging.internetofthings.ibmcloud.com -p 8883 --cafile messaging.pem -u $WIOTP_APIKEY -P $WIOTP_TOKEN -i "a:OrgID:mosquitto" -t iot-2/type/OpenEEW/id/A8032A4DD5F0/cmd/sendacceldata/fmt/json  -m {LiveDataDuration:1}

mosquitto_pub -h OrgID.messaging.internetofthings.ibmcloud.com -p 8883 --cafile messaging.pem -u $WIOTP_APIKEY -P $WIOTP_TOKEN -i "a:OrgID:mosquitto" -t iot-2/type/OpenEEW/id/A8032A4DD5F0/cmd/sendacceldata/fmt/json  -m {LiveDataDuration:30}

mosquitto_pub -h OrgID.messaging.internetofthings.ibmcloud.com -p 8883 --cafile messaging.pem -u $WIOTP_APIKEY -P $WIOTP_TOKEN -i "a:OrgID:mosquitto" -t iot-2/type/OpenEEW/id/A8032A4DD5F0/cmd/sendacceldata/fmt/json  -m {LiveDataDuration:4294967295}

mosquitto_pub -h OrgID.messaging.internetofthings.ibmcloud.com -p 8883 --cafile messaging.pem -u $WIOTP_APIKEY -P $WIOTP_TOKEN -i "a:OrgID:mosquitto" -t iot-2/type/OpenEEW/id/A8032A4DD5F0/cmd/sendacceldata/fmt/json  -m {LiveDataDuration:0}
```

### SAMPLERATE

Use this MQTT topic to change the ADXL355 sampling rate.

- Turn off the Accelerometer:

 ```sh
 mosquitto_pub -h 192.168.1.101 -t iot-2/cmd/samplerate/fmt/json -m {SampleRate:0} -i cmd:samplerate

 mosquitto_pub -h OrgID.messaging.internetofthings.ibmcloud.com -p 8883 --cafile messaging.pem -u $WIOTP_APIKEY -P $WIOTP_TOKEN -i "a:OrgID:mosquitto" -t iot-2/type/OpenEEW/id/A8032A4DD5F0/cmd/samplerate/fmt/json  -m {SampleRate:0}
```

- Standard 31 samples per second

```sh
 mosquitto_pub -h 192.168.1.101 -t iot-2/cmd/samplerate/fmt/json -m {SampleRate:31} -i cmd:samplerate

 mosquitto_pub -h OrgID.messaging.internetofthings.ibmcloud.com -p 8883 --cafile messaging.pem -u $WIOTP_APIKEY -P $WIOTP_TOKEN -i "a:OrgID:mosquitto" -t iot-2/type/OpenEEW/id/A8032A4DD5F0/cmd/samplerate/fmt/json  -m {SampleRate:31}
 ```

- 125 samples per second (a firehose that eats bandwidth)

 ```sh
 mosquitto_pub -h 192.168.1.101 -t iot-2/cmd/samplerate/fmt/json -m {SampleRate:125} -i cmd:samplerate

 mosquitto_pub -h OrgID.messaging.internetofthings.ibmcloud.com -p 8883 --cafile messaging.pem -u $WIOTP_APIKEY -P $WIOTP_TOKEN -i "a:OrgID:mosquitto" -t iot-2/type/OpenEEW/id/A8032A4DD5F0/cmd/samplerate/fmt/json  -m {SampleRate:125}
 ```

### FWCHECK

Use this MQTT topic to force a check for new firmware. If a device is running for many months it might fall behind on the version of the firmware it has installed. As part of the ESP32 power up / activation process, the board does a firmware version check. If there is a newer firmware version, it initiates an OTA firmware update. That only happens on startup.

A board that has been running for a long time might be stranded on an old version.
The administrator / device owner can command the running firmware to check for a firmware update.

This allows us to dynamically update sensors if there is a security / software flaw / feature improvement.

The board will blink magenta while it checks if newer firmware is available. If the current firmware is downlevel, the new firmware will be downloaded, verified, installed and the device restarted.

```sh
mosquitto_pub -h 192.168.1.101 -t iot-2/cmd/firmwarecheck/fmt/json -m {} -i cmd:firmware

mosquitto_pub -h OrgID.messaging.internetofthings.ibmcloud.com -p 8883 --cafile messaging.pem -u $WIOTP_APIKEY -P $WIOTP_TOKEN -i "a:OrgID:mosquitto" -t iot-2/type/OpenEEW/id/A8032A4DD5F0/cmd/firmwarecheck/fmt/json  -m {}
```

### SEND10SEC

Use this MQTT topic to send 10 seconds of accelerometer history to the cloud.  In normal silence mode, the sensor will queue 10 seconds of data and run an algorithm that detects anomolous shaking. This command will send the entire 10 second buffer to the cloud.

```sh
mosquitto_pub -h 192.168.1.101 -t iot-2/cmd/10secondhistory/fmt/json -m {} -i cmd:send10sec

mosquitto_pub -h OrgID.messaging.internetofthings.ibmcloud.com -p 8883 --cafile messaging.pem -u $WIOTP_APIKEY -P $WIOTP_TOKEN -i "a:OrgID:mosquitto" -t iot-2/type/OpenEEW/id/A8032A4DD5F0/cmd/10secondhistory/fmt/json  -m {}
```

### Python Examples

This repository also contains Python examples that can be modified to do the above. `tbd`
Learn more about the [IBM Watson IoT Platform Python SDK](https://ibm-watson-iot.github.io/iot-python/)

### Node Examples

This repository also contains Node examples that can be modified to do the above. `tbd`

### Author

- John Walicki
