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

#include "Adxl355.h"

Adxl355::Adxl355(int chipSelectPin, uint8_t deviceId, bool spiBegin) : _deviceId(deviceId), _chipSelectPin(chipSelectPin){

                                                                                            };

Adxl355::~Adxl355()
{
  if (isRunning())
    stop();
}
void Adxl355::initSPI(SPIClass &spi)
{
  spi_obj = &spi;
  spi_obj->begin();
}
// void Adxl355::init(int8_t sck, int8_t miso, int8_t mosi, int8_t ss){
//   spi_obj = new SPIClass();
//   spi_obj -> setDataMode(SPI_MODE0);
//   spi_obj -> begin(sck,miso,mosi,ss);
// }

bool Adxl355::isDeviceRecognized()
{
  // Read two registers to make sure it is working
  uint16_t test = read16(0x01);

  // These registers should always have 0x1ded in them
  // if (test != 0x1ded)
  // {
  //   Serial.printf("Connected device does not appear to be an Adxl355: %02x\n\n", test);
  // }
  // else{
  //   Serial.printf("Conected to ADXL: %02x\n",test);
  // }

  return (test == 0x1ded);
}

int Adxl355::start()
{
  int result = 0;

  if (!isDeviceRecognized())
  {
    result = -1;
  }
  else
  {
    uint8_t power = read8(POWER_CTL);

    if (power & POWER_CTL_VALUES::POWER_CTL_OFF)
    {
      power = power & (int)POWER_CTL_VALUES::POWER_CTL_ON;
      write8(POWER_CTL, power);
    }
  }

  return result;
}

int Adxl355::stop()
{
  int power = read8(POWER_CTL);

  if (!(power & POWER_CTL_VALUES::POWER_CTL_OFF))
  {
    power = power | (int)POWER_CTL_VALUES::POWER_CTL_OFF;
    write8(POWER_CTL, power);
  }

  return power;
}

uint8_t Adxl355::getAnalogDevicesID()
{
  uint8_t work = read8(DEVID_AD);

  return work;
}

uint8_t Adxl355::getAnalogDevicesMEMSID()
{
  uint8_t work = read8(DEVID_MST);

  return work;
}

uint8_t Adxl355::getDeviceId()
{
  uint8_t work = read8(PARTID);

  return work;
}

uint8_t Adxl355::getRevision()
{
  uint8_t work = read8(REVID);

  return work;
}

bool Adxl355::isRunning()
{
  bool result = false;
  int work = read8(POWER_CTL);

  result = (work & POWER_CTL_VALUES::POWER_CTL_OFF)
               ? false
               : true;

  return result;
}

bool Adxl355::isTempSensorOn()
{
  bool result = false;

  uint8_t work = read8(POWER_CTL);

  result = ((work & POWER_CTL_VALUES::POWER_CTL_OFF) || (work & POWER_CTL_VALUES::POWER_CTL_TEMP_OFF))
               ? false
               : true;

  return result;
}

void Adxl355::startTempSensor()
{
  uint8_t work = read8(POWER_CTL);

  if (work & POWER_CTL_VALUES::POWER_CTL_TEMP_OFF)
  {
    work = work & (int)POWER_CTL_VALUES::POWER_CTL_TEMP_ON;
    write8(POWER_CTL, work);
  }
}

void Adxl355::stopTempSensor()
{
  uint8_t work = read8(POWER_CTL);

  if (!(work & POWER_CTL_VALUES::POWER_CTL_TEMP_OFF))
  {
    work = work | (int)POWER_CTL_VALUES::POWER_CTL_TEMP_OFF;
    write8(POWER_CTL, work);
  }
}

double Adxl355::getTemperatureC()
{
  uint16_t itemp = read16(TEMP2);
  double dtemp = ((double)(1852 - itemp)) / 9.05 + 19.21;

  return dtemp;
}

double Adxl355::getTemperatureF()
{
  double result = getTemperatureC();

  return result * 9 / 5 + 32;
}

Adxl355::HPF_CORNER Adxl355::getHpfCorner()
{
  uint8_t work = read8(FILTER);

  return (HPF_CORNER)((work & HPF_CORNER::HPF_CORNER_MASK) >> 4);
}

int Adxl355::getRawAxes(long *x, long *y, long *z)
{
  uint8_t output[9];
  memset(output, 0, 9);

  int result = readBlock(XDATA3, 9, (uint8_t *)output);

  unsigned long workx = 0;
  unsigned long worky = 0;
  unsigned long workz = 0;

  if (result == 9)
  {
    workx = (output[0] << 12) | (output[1] << 4) | (output[2] >> 4);
    worky = (output[3] << 12) | (output[4] << 4) | (output[5] >> 4);
    workz = (output[6] << 12) | (output[7] << 4) | (output[8] >> 4);
  }

  *x = twosComplement(workx);
  *y = twosComplement(worky);
  *z = twosComplement(workz);

  return result;
}

void Adxl355::setHpfCorner(HPF_CORNER value)
{
  if (errorIfRunning())
    return;

  int work = read8(FILTER);

  work = (work & ~(HPF_CORNER::HPF_CORNER_MASK << 4)) | ((int)value) << 4;

  write8(FILTER, work);
}

Adxl355::ODR_LPF Adxl355::getOdrLpf()
{
  uint8_t work = read8(FILTER);

  return (ODR_LPF)(work & ODR_LPF::ODR_LPF_MASK);
}

void Adxl355::setOdrLpf(ODR_LPF value)
{
  if (errorIfRunning())
    return;

  uint8_t work = read8(FILTER);

  work = (work & ~(ODR_LPF::ODR_LPF_MASK)) | ((int)value);

  write8(FILTER, work);
}

Adxl355::RANGE_VALUES Adxl355::getRange()
{
  int range = read8(I2CSPEED_INTPOLARITY_RANGE);

  return (RANGE_VALUES)(range & RANGE_VALUES::RANGE_MASK);
}

void Adxl355::setRange(RANGE_VALUES value)
{
  if (errorIfRunning())
    return;

  uint8_t range = read8(I2CSPEED_INTPOLARITY_RANGE);

  range &= ~(RANGE_VALUES::RANGE_MASK);
  range |= (int)value;

  write8(I2CSPEED_INTPOLARITY_RANGE, range);
}

int Adxl355::getTrim(int32_t *x, int32_t *y, int32_t *z)
{
  uint8_t output[6];

  memset(output, 0xff, sizeof(output));

  int result = readBlock(OFFSET_X_H, 6, (uint8_t *)output);

  if (result == 0)
  {
    *x = twosComplement((output[0] << 8 | output[1]) << 4);
    *y = twosComplement((output[2] << 8 | output[3]) << 4);
    *z = twosComplement((output[4] << 8 | output[5]) << 4);
  }

  return result;
}

void Adxl355::setTrim(int32_t x, int32_t y, int32_t z)
{
  if (errorIfRunning())
    return;

  int16_t workx = (x >> 4);
  int16_t worky = (y >> 4);
  int16_t workz = (z >> 4);
  uint8_t hix = (workx & 0xff00) >> 8;
  uint8_t lox = workx & 0x00ff;
  uint8_t hiy = (worky & 0xff00) >> 8;
  uint8_t loy = worky & 0x00ff;
  uint8_t hiz = (workz & 0xff00) >> 8;
  uint8_t loz = workz & 0x00ff;

  write8(OFFSET_X_H, hix);
  write8(OFFSET_X_L, lox);
  write8(OFFSET_Y_H, hiy);
  write8(OFFSET_Y_L, loy);
  write8(OFFSET_Z_H, hiz);
  write8(OFFSET_Z_L, loz);
}

uint8_t Adxl355::getIntMap()
{
  uint8_t work = read8(INT_MAP);

  return work;
}
// Calibrate the Adxl355
void Adxl355::calibrateSensor(int fifoReadCount, bool verbose)
{
  bool debug = verbose;
  long fifoOut[32][3];
  int result;
  int readings = 0;
  long totalx = 0;
  long totaly = 0;
  long totalz = 0;

  memset(fifoOut, 0, sizeof(fifoOut));

  if (debug)
    Serial.printf("Calibrating device with %d FIFO reads\r\n", fifoReadCount);

  stop();
  setTrim(0, 0, 0);
  start();
  // delay(2000);

  for (int j = 0; j < fifoReadCount; j++)
  {
    if (debug)
      Serial.printf("Fifo read number %d\r\n", j + 1);

    while (!isFifoFull())
    {
      ;
      // delay(10);
    }

    if (-1 != (result = readFifoEntries((long *)fifoOut)))
    {
      if (debug)
        Serial.printf("Retrieved %d entries\r\n", result);
      readings += result;

      for (int i = 0; i < result; i++)
      {
        totalx += fifoOut[i][0];
        totaly += fifoOut[i][1];
        totalz += fifoOut[i][2];
      }
    }
    else
    {
      if (debug)
        Serial.println("Fifo read failed");
    }
  }

  long avgx = totalx / readings;
  long avgy = totaly / readings;
  long avgz = totalz / readings;

  if (debug)
    Serial.printf("\r\nTotal/Average X=%ld/%ld; Y=%ld/%ld; Z=%ld/%ld\r\n",
                  totalx, avgx,
                  totaly, avgy,
                  totalz, avgz);

  stop();

  // Set the trim with the calculated values
  setTrim(avgx, avgy, avgz);
  start();
  //delay(2000);
}
// Set up the Adxl355 with our required values
void Adxl355::initializeSensor(RANGE_VALUES range, ODR_LPF odr_lpf, bool verbose)
{
  bool debug = verbose;

  if (debug)
    Serial.println("Stopping...");
  stop();
  if (debug)
    Serial.println("Stopped");

  setRange(Adxl355::RANGE_VALUES::RANGE_2G);

  Adxl355::RANGE_VALUES rangeValue = getRange();

  switch (rangeValue)
  {
  case Adxl355::RANGE_VALUES::RANGE_2G:
    if (debug)
      Serial.println("Range 2g");
    break;
  case Adxl355::RANGE_VALUES::RANGE_4G:
    if (debug)
      Serial.println("Range 4g");
    break;
  case Adxl355::RANGE_VALUES::RANGE_8G:
    if (debug)
      Serial.println("Range 8g");
    break;
  default:
    if (debug)
      Serial.println("Unknown range");
    break;
  }

  setOdrLpf(odr_lpf);

  if (debug)
  {

    Serial.print("Low pass filter = ");
    Serial.println((String)getOdrLpf());
    //delay(200);
    Serial.println("High pass filter = " + (String)getHpfCorner());
    //delay(200);
  }
  //Set interrupt to FIFO FULL on INT1
  setIntMap(0x01);

  if (debug)
    Serial.println("Done initializing");
}

void Adxl355::setIntMap(uint8_t value)
{
  write8(INT_MAP, value);
}

int Adxl355::getFifoCount()
{
  uint8_t work = read8(FIFO_ENTRIES);

  return work;
}

Adxl355::STATUS_VALUES Adxl355::getStatus()
{
  uint8_t work = read8(STATUS);

  return (STATUS_VALUES)work;
}

bool Adxl355::isFifoFull()
{
  STATUS_VALUES work = getStatus();

  return (work & STATUS_VALUES::FIFO_FULL) ? true : false;
}

bool Adxl355::isFifoOverrun()
{
  STATUS_VALUES work = getStatus();

  return (work & STATUS_VALUES::FIFO_OVERRUN) ? true : false;
}

bool Adxl355::isDataReady()
{
  STATUS_VALUES work = getStatus();

  return (work & STATUS_VALUES::DATA_READY) ? true : false;
}

int Adxl355::readFifoEntries(long *output)
{
  int fifoCount = getFifoCount();
  uint8_t data[9];
  memset(data, 0, 9);

  unsigned long work[3];

  for (int i = 0; i < fifoCount / 3; i++)
  {
    int result = readBlock(FIFO_DATA, 9, (uint8_t *)data);

    if (result > 0)
    {
      for (int j = 0; j < 9; j += 3)
      {
        work[j / 3] = (data[0 + j] << 12) | (data[1 + j] << 4) | (data[2 + j] >> 4);
        output[i * 3 + j / 3] = twosComplement(work[j / 3]);
      }
    }
    else
    {
      return -1;
    }
  }

  return fifoCount / 3;
}

double Adxl355::valueToGals(long rawValue, int decimals)
{
  double slider = (decimals > 1) ? pow(10.0, (double)decimals) : 1.0;

  double result = ((double)rawValue / 260000.0) * 980.665;

  result = round(result * slider) / slider;

  return result;
}

int64_t Adxl355::valueToGalsInt(int32_t rawValue, int decimals)
{
  double slider = (decimals > 1) ? pow(10.0, (double)decimals) : 1.0;
  double work = valueToGals(rawValue, decimals);
  int64_t asInt = (int64_t)(work * slider);

  return asInt;
}

bool Adxl355::errorIfRunning()
{
  bool result = false;

  if (isRunning())
  {
    Serial.println("*** ERROR *** Sensor modification attempted when sensor is running");
    result = true;
  }

  return result;
}

long Adxl355::twosComplement(unsigned long value)
{
  value = -(value & (1 << (20 - 1))) + (value & ~(1 << (20 - 1)));

  return value;
}

uint8_t Adxl355::read8(uint8_t reg)
{
  uint8_t data = 0;

  byte regToSend = (reg << 1) | READ_BYTE;

  digitalWrite(_chipSelectPin, LOW);
  spi_obj->transfer(regToSend);
  data = spi_obj->transfer(0x00);
  digitalWrite(_chipSelectPin, HIGH);

  return data;
}

uint16_t Adxl355::read16(uint8_t reg)
{
  uint16_t data = 0;
  uint8_t oneByte;

  byte regToSend = (reg << 1) | READ_BYTE;

  digitalWrite(_chipSelectPin, LOW);
  spi_obj->transfer(regToSend);

  int i = 2;

  while (i--)
  {
    oneByte = spi_obj->transfer(0x00);
    data |= (oneByte << (i * 8));
  }

  digitalWrite(_chipSelectPin, HIGH);

  return data;
}

uint8_t Adxl355::readBlock(uint8_t reg, uint8_t length, uint8_t *output)
{

  byte regToSend = (reg << 1) | READ_BYTE;

  digitalWrite(_chipSelectPin, LOW);
  spi_obj->transfer(regToSend);

  int i = length;

  while (i)
  {
    *output++ = spi_obj->transfer(0x00);
    i--;
  }

  digitalWrite(_chipSelectPin, HIGH);

  return length - i;
}

void Adxl355::write8(uint8_t reg, uint8_t data)
{
  byte regToSend = (reg << 1) | WRITE_BYTE;

  digitalWrite(_chipSelectPin, LOW);
  spi_obj->transfer(regToSend);
  spi_obj->transfer(data);
  digitalWrite(_chipSelectPin, HIGH);
}