/*
----------------------------------------------------------------------------

This file is part of the Sanworks Bpod repository
Copyright (C) 2023 Sanworks LLC, Rochester, New York, USA

----------------------------------------------------------------------------

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

// Library for programming the Texas Instruments FDC2214 capacitive sensor

#ifndef FDC2214_h
#define FDC2214_h
#include "Arduino.h"
#include <Wire.h>

// Sensor I2C Address
#define ADDRESS 42

// Sensor Registers
#define REG_RCOUNT_CH0 8
#define REG_RCOUNT_CH1 9
#define REG_SETTLECOUNT_CH0 16
#define REG_SETTLECOUNT_CH1 17
#define REG_CLKDIVIDERS_CH0 20
#define REG_CLKDIVIDERS_CH1 21
#define REG_STATUS 24
#define REG_ERRCONFIG 25
#define REG_CONFIG 26
#define REG_MUXCONFIG 27
#define REG_RESET 28
#define REG_DRIVECURRENT_CH0 30
#define REG_DRIVECURRENT_CH1 31
#define REG_MFGID 126

class FDC2214
{
public:
  // Constructor
  FDC2214(byte clockEnable, byte intB, byte shutDown);
  void init();
  uint32_t readSensor();
  uint16_t readRegister16(byte regID);
  void writeRegister16(byte regID, byte msb, byte lsb);
  void sendByte(byte aByte);
  void set_RCOUNT(uint16_t value);
  void set_SETTLECOUNT(uint16_t value);
  void set_REF_DIVIDER(uint8_t value);
  void set_DRIVE_CURRENT(uint8_t value);
  void set_ACTIVE_CHANNEL(uint8_t value);
  union {
    byte uint8[4];
    uint16_t uint16[2];
    uint32_t uint32[1];
  } typeBuffer;
private:
  byte clockEnable = 0;
  byte intB;
  byte shutDown;
  byte activeChannel = 0;
  byte chConfigBit = B00000000;
  uint16_t registerBuffer = 0;
  uint32_t sensorValue = 0;
};
#endif
