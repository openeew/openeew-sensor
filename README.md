# OpenEEW Sensor
The OpenEEW sensor features a high performance MEMS accelerometer and Ethernet or WiFi connectivity. It includes also a loud buzzer and 3 bright Neopixel LEDS for alarm functions. By including alarm functions, the owners of the locations where they are installed are more likely to value and look after the device.

The OpenEEW sensor has already shown itself to be [as good as seismometers that cost 60x more](https://openeew.com/blog/sensor-benchmark) for the purpose of earthquake early-warnings.

The sensor can be readily [bought from Grillo](https://grillo.io/product/openeew-node/), or made following these instructions.

## Hardware

Components are mounted in a PCB with the corresponding circuitry. The board operates at 3.3V with a maximum current of 1A. The accelerometer is accessed via SPI interface, specifically ESP32's HSPI. For this device we have selected the ADXL355 accelerometer for its low noise and relatively low cost.

The ethernet uses the LAN8720A transceiver. We have not included PoE in this varient to reduce complexity and cost, but this may be a good option for some.

GPS can optionally be added with a UART interface.

You can find the schematics, PCB, and BOM files in [here](/pcb). The board was generated using [Kicad](https://kicad-pcb.org/).

The assembled sensor, enclosure, and power supply, can also be bought directly [here](https://grillo.io/product/openeew-node/).

![PCB](images/openeew-node-board.jpg)

Please note the following pins:
- ADXL355 > SPI (HSPI) > CS GPIO 15
- Neopixel data pin> GPIO 16
- Buzzer > GPIO 32

## Enclosure

You can [3d print yourself a sturdy wall mounted enclosure](https://www.thingiverse.com/thing:4854991) for your PCB using the 3d files. This design features snap lip joints to make it easy to assemble, and flanged base for a sturdy connection to the wall surface. 

<img src="/images/openeew-node-withlid.jpg" width="300">
<img src="/images/openeew-node-blue.jpg" width="300">
<img src="/images/animated-box.gif" width="300">


## Firmware
The firmware now has its [own repo](https://github.com/openeew/openeew-firmware).

<a href="https://github.com/openeew/openeew-sensor/graphs/contributors">
  <img src="https://contributors-img.web.app/image?repo=openeew/openeew-sensor" />
</a>

___

Enjoy! Give us [feedback](https://github.com/openeew/openeew-sensor/issues) if you have suggestions on how to improve this information.

## Contributing and Developer information

The community welcomes your involvement and contributions to this project. Please read the OpenEEW [contributing](https://github.com/openeew/openeew/blob/master/CONTRIBUTING.md) document for details on our code of conduct, and the process for submitting pull requests to the community.

## License

The OpenEEW sensor is licensed under the Apache Software License, Version 2. Separate third party code objects invoked within this code pattern are licensed by their respective providers pursuant to their own separate licenses. Contributions are subject to the [Developer Certificate of Origin, Version 1.1 (DCO)](https://developercertificate.org/) and the [Apache Software License, Version 2](http://www.apache.org/licenses/LICENSE-2.0.txt).
