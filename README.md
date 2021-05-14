# OpenEEW Sensor
The OpenEEW sensor has already shown itself to be [as good as seismometers](https://openeew.com/blog/sensor-benchmark) for the purpose of earthquake early-warnings (EEW). EEWs are concerned only with strong shaking and so expensive broadband seismometers are not necessary for detection.

This hardware design has been created to drastically reduce the cost of a seismometer through the usage of off-the-shelf parts. The key component is the ADXL355 MEMS accelerometer, which has far lower noise than other accelerometers on the market. This low noise allows it to detect earthquakes at further distances.

The sensor can be bought from [PCBWay](https://www.pcbway.com/project/gifts_detail/OpenEEW_Node.html), [Grillo](https://grillo.io/product/openeew-node/), or made following these instructions.

## Hardware
The PCB features the following:
- A high performance MEMS accelerometer
- ESP-32-WROOM-32 which features a dual-core processor, 240MHz frequency, 4MB Flahs, and 8MB PSRAM
- Ethernet connector and controller to provide more stable internet connections that need to last for years in some installations
- 3 Neopixel RBG LEDs and a buzzer to provide alarm functions when an alert message is received on the device
- A USB-C interface for powering and flashing the device

![PCB](https://user-images.githubusercontent.com/6279965/118044476-4dd2c380-b33c-11eb-8baa-c089b383fa31.PNG)
[Schematic here](/pcb/openeew-schematic.pdf)

Please note the following pins:
- ADXL355 > SPI (HSPI) > CS GPIO 15
- Neopixel data pin> GPIO 16
- Buzzer > GPIO 32

The board operates at 3.3V with a minimum current of 0.5A. The accelerometer is accessed via the ESP32's HSPI interface.

The ethernet uses the LAN8720A transceiver. We have not included PoE in this variant to reduce complexity and cost.

GPS can optionally be added via the UART header, and I2C devices can be added via the I2C header. However we have opted to use NTP as a default for timekeeping, and we use the OpenEEW app to record latitude and longitude when provisioning the device.

### Information
You can find the schematics, PCB, and BOM files in [here](/pcb). The board was generated using [Kicad](https://kicad-pcb.org/).

<img src="/images/openeew-node-withlid.jpg" width="300">
<img src="/images/openeew-node-blue.jpg" width="300">
<img src="/images/animated-box.gif" width="300">

___

Enjoy! Give us [feedback](https://github.com/openeew/openeew-sensor/issues) if you have suggestions on how to improve this information.

## Contributing and Developer information

The community welcomes your involvement and contributions to this project. Please read the OpenEEW [contributing](https://github.com/openeew/openeew/blob/master/CONTRIBUTING.md) document for details on our code of conduct, and the process for submitting pull requests to the community.

## License

The OpenEEW sensor is licensed under the Apache Software License, Version 2. Separate third party code objects invoked within this code pattern are licensed by their respective providers pursuant to their own separate licenses. Contributions are subject to the [Developer Certificate of Origin, Version 1.1 (DCO)](https://developercertificate.org/) and the [Apache Software License, Version 2](http://www.apache.org/licenses/LICENSE-2.0.txt).
