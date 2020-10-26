//=============================================================================
// Copyright Grillo Holdings Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// =============================================================================

#ifndef _ADXL355_H
#define _ADXL355_H
#include <Wire.h>
#include "math.h"
#include <SPI.h>
#include "Arduino.h"

// #define Grillo_Class false
//
// #if Grillo_Class
//   #include <Grillo.h>
// #else
#define SAMPLE_RATE_125 true
#define SAMPLE_RATE_62_5 false
#define SAMPLE_RATE_31_25 false
// #endif

class Adxl355
{
private:
  static const uint8_t DEVID_AD = 0x00;
  static const uint8_t DEVID_MST = 0x01;
  static const uint8_t PARTID = 0x02;
  static const uint8_t REVID = 0x03;
  static const uint8_t STATUS = 0x04;
  static const uint8_t FIFO_ENTRIES = 0x05;
  static const uint8_t INT_MAP = 0x2a;
  static const uint8_t I2CSPEED_INTPOLARITY_RANGE = 0x2c;
  static const uint8_t POWER_CTL = 0x2d;
  static const uint8_t RESETTT = 0x2f;

  static const uint8_t RESET_VALUE = 0x52;

  static const uint8_t TEMP2 = 0x06;
  static const uint8_t TEMP1 = 0x07;

  static const uint8_t XDATA3 = 0x08;
  static const uint8_t XDATA2 = 0x09;
  static const uint8_t XDATA1 = 0x0a;
  static const uint8_t YDATA3 = 0x0b;
  static const uint8_t YDATA2 = 0x0c;
  static const uint8_t YDATA1 = 0x0d;
  static const uint8_t ZDATA3 = 0x0e;
  static const uint8_t ZDATA2 = 0x0f;
  static const uint8_t ZDATA1 = 0x10;

  static const uint8_t FIFO_DATA = 0x11;

  static const uint8_t OFFSET_X_H = 0x1e;
  static const uint8_t OFFSET_X_L = 0x1f;
  static const uint8_t OFFSET_Y_H = 0x20;
  static const uint8_t OFFSET_Y_L = 0x21;
  static const uint8_t OFFSET_Z_H = 0x22;
  static const uint8_t OFFSET_Z_L = 0x23;

  static const uint8_t FILTER = 0x28;

  static const int READ_BYTE = 0x01;
  static const int WRITE_BYTE = 0x00;

  // Pins used for the connection with the sensor
  int _chipSelectPin;

  enum POWER_CTL_VALUES
  {
    POWER_CTL_OFF = 0x01,
    POWER_CTL_ON = ~POWER_CTL_OFF,
    POWER_CTL_TEMP_OFF = 0x02,
    POWER_CTL_TEMP_ON = ~POWER_CTL_TEMP_OFF
  };

  enum I2C_SPEED_VALUES
  {
    I2C_SPEED_FAST = 0x80,
    I2C_SPEED_SLOW = 0x00
  };

  uint8_t _deviceId;

  //SPI
  SPIClass *spi_obj = NULL;

public:
  enum STATUS_VALUES
  {
    NVM_BUSY = 0x10,
    ACTIVITY = 0x08,
    FIFO_OVERRUN = 0x04,
    FIFO_FULL = 0x02,
    DATA_READY = 0x01
  };

  enum RANGE_VALUES
  {
    RANGE_2G = 0x01,
    RANGE_4G = 0x02,
    RANGE_8G = 0x03,
    RANGE_MASK = 0x03
  };

  enum HPF_CORNER
  {
    NOT_ENABLED = 0b000,
    ODR_X_2_47 = 0b001,
    ODR_X_62_084 = 0b010,
    ODR_X_15_545 = 0b011,
    ODR_X_3_862 = 0b100,
    ODR_X_0_954 = 0b101,
    ODR_X_0_238 = 0b110,
    HPF_CORNER_MASK = 0b01110000
  };

  enum ODR_LPF
  {
    ODR_4000_AND_1000 = 0b0000,
    ODR_2000_AND_500 = 0b0001,
    ODR_1000_AND_250 = 0b0010,
    ODR_500_AND_125 = 0b0011,
    ODR_250_AND_62_5 = 0b0100,
    ODR_125_AND_31_25 = 0b0101,
    ODR_62_5_AND_15_625 = 0b0110,
    ODR_31_25_AND_7_813 = 0b0111,
    ODR_15_625_AND_3_906 = 0b1000,
    ODR_7_813_AND_1_953 = 0b1001,
    ODR_3_906_AND_0_977 = 0b1010,
    ODR_LPF_MASK = 0b1111
  };

  Adxl355(int chipSelectPin, uint8_t deviceId = 0x1d, bool wireBegin = false);
  ~Adxl355();
  void initSPI(SPIClass &spi);
  //void init(int8_t sck, int8_t miso, int8_t mosi, int8_t ss);
  uint8_t getAnalogDevicesID();
  uint8_t getAnalogDevicesMEMSID();
  uint8_t getDeviceId();
  uint8_t getRevision();
  STATUS_VALUES getStatus();
  bool isFifoFull();
  bool isFifoOverrun();
  bool isDataReady();
  HPF_CORNER getHpfCorner();
  void setHpfCorner(HPF_CORNER value);
  ODR_LPF getOdrLpf();
  void setOdrLpf(ODR_LPF value);
  int getTrim(int32_t *x, int32_t *y, int32_t *z);
  void setTrim(int32_t x, int32_t y, int32_t z);
  int getFifoCount();

  uint8_t getIntMap();
  void setIntMap(uint8_t value);
  void calibrateSensor(int fifoReadCount, bool verbose);
  void initializeSensor(RANGE_VALUES range, ODR_LPF odr_lpf, bool verbose);

  int start();
  int stop();
  bool isDeviceRecognized();
  void Reset();
  bool isRunning();
  bool isTempSensorOn();
  void startTempSensor();
  void stopTempSensor();
  double getTemperatureC();
  double getTemperatureF();
  int getRawAxes(long *x, long *y, long *z);

  // Output must be 96 entries
  int readFifoEntries(long *output);

  RANGE_VALUES getRange();
  void setRange(RANGE_VALUES value);

  static double valueToGals(long rawValue, int decimals = 3);
  static int64_t valueToGalsInt(int32_t rawValue, int decimals = 3);

private:
  bool errorIfRunning();
  static long twosComplement(unsigned long value);
  uint8_t read8(uint8_t reg);
  uint16_t read16(uint8_t reg);
  uint8_t readBlock(uint8_t reg, uint8_t length, uint8_t *output);
  void write8(uint8_t reg, uint8_t data);
  int Write16(uint8_t reg, uint16_t value);
};

#endif