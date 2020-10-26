# ADXL355 library arduino

This library allows to have interaction with the ADXL355 accelerometer sensor using SPI interface. All is written in C++.
All registers addresses are declared in the header file, as well as the enumeration of POWER_CTL_VLAUES, STATUS_VALUES, RANGE_VALUES and ODR_LPF.

ADXL355 object is instantiated with the Chip select (CS) as parameter. Includes functions to check the ID of device, revision, etc.
Allows to check the STATUS_VALUES, the status of the FIFO, get and set properties as High Pass Corner frequency, ODR, Low Pass frequency, trim, range, interrupts, etc.

It also allows user to control the start and stop the accelerometer and to transform values to gals.

### V1.2
Standarized functions like initializeSensor to have the RANGE_VALUES and ODR_LPF values as parameters instead of a fixed declaration. Also changed the init(SPIClass &spi) function to initSPI(SPIClass &spi) so that it is more intuitive that this function can be used if a specific SPI interface wants to be used, for using the default one, no init function needs to be used other than the default SPI initialization on the main code.

### V1.1
Added initialize and calibrate functions that were originally declared in the main file. Also added the function init which allows to assign a specific SPI interface to the ADXL and starts it.

### V1.0
Initial version

___

### Authors

- [Mark Radbourne](https://github.com/markrad)
- [Grillo](https://grillo.io)

Enjoy!  Give us [feedback](https://github.com/grillo/arduino-adxl355-spi/issues) if you have suggestions on how to improve this library.

## License

This library is licensed under the Apache Software License, Version 2. Contributions are subject to the [Developer Certificate of Origin, Version 1.1 (DCO)](https://developercertificate.org/) and the [Apache Software License, Version 2](http://www.apache.org/licenses/LICENSE-2.0.txt).
