/*
----------------------------------------------------------------------------

This file is part of the Sanworks LickStick repository
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

// Simplified Library for programming the FDC2214 capacitive sensor as installed on the Sanworks LickStick device
//
// Section and table numbers are those of the FDC2214-Q1 datasheet (TI SNOSCZ9A, October 2024). The register bit
// fields are named in FDC2214.h.
//
// The datasheet asks for a setting to be changed only while the sensor is asleep (6.4.1): each setter writes CONFIG
// asleep, then its registers, then CONFIG awake. Conversions restart 16384 internal clock cycles (about 0.4ms) after
// the wake write (6.4.3), and the data registers read 0 from the sleep write until the first conversion ends.
// CONFIG also holds the active channel, so every write of it goes through configValue().
//
// Nothing here is kept in a member between two I2C transfers except the settings themselves: readSensor() runs in
// the sketch's timer interrupt and the rest in loop(), and a buffer shared between them was the cause of wrong
// register values in firmware v2 as first released. The Wire object is still shared: see FDC2214.h.

#include <Arduino.h>
#include <Wire.h>
#include "FDC2214.h"

FDC2214::FDC2214(byte clockEnable, byte intB, byte shutDown) {
  clockEnablePin = clockEnable;
  intBPin = intB;
  shutDownPin = shutDown;
}

bool FDC2214::init(uint16_t rCount, uint16_t settleCount, uint8_t refDivider, uint8_t driveCurrent, uint8_t channel) {
  pinMode(clockEnablePin, OUTPUT);
  digitalWrite(clockEnablePin, HIGH); // Enable external clock oscillator IC
  pinMode(intBPin, INPUT);
  pinMode(shutDownPin, OUTPUT);
  digitalWrite(shutDownPin, HIGH); // Shutdown mode: returns every register to its default
  delay(1);
  digitalWrite(shutDownPin, LOW);
  delay(3); // The sensor answers on I2C at most 2ms after SD goes low (tSDWAKEUP, section 5.6)

  Wire.begin(); // join i2c bus
  Wire.setClock(400000); // I2C fast mode, the sensor's maximum (6.5.1)

  activeChannel = channel;
  bool ok = writeRegister16(REG_RESET, RESET_DEV); // Soft reset. Leaves the sensor asleep
  ok &= writeRegister16(REG_RCOUNT_CH0, rCount);
  ok &= writeRegister16(REG_RCOUNT_CH1, rCount);
  ok &= writeRegister16(REG_SETTLECOUNT_CH0, settleCount);
  ok &= writeRegister16(REG_SETTLECOUNT_CH1, settleCount);
  ok &= writeRegister16(REG_CLKDIVIDERS_CH0, CLKDIVIDERS_FIN_SEL_DIV2 | refDivider);
  ok &= writeRegister16(REG_CLKDIVIDERS_CH1, CLKDIVIDERS_FIN_SEL_DIV2 | refDivider);
  ok &= writeRegister16(REG_ERRCONFIG, ERRCONFIG_DRDY_2INT);
  ok &= writeRegister16(REG_MUXCONFIG, MUXCONFIG_SINGLE_CHANNEL);
  ok &= writeRegister16(REG_DRIVECURRENT_CH0, (uint16_t)driveCurrent << DRIVECURRENT_IDRIVE_SHIFT);
  ok &= writeRegister16(REG_DRIVECURRENT_CH1, (uint16_t)driveCurrent << DRIVECURRENT_IDRIVE_SHIFT);
  ok &= wake(); // MUST BE LAST: the other registers are programmed while the sensor is asleep
  return ok;
}

// CONFIG for the active channel, awake: configurable drive current, external clock, INTB enabled, normal current range
uint16_t FDC2214::configValue() {
  return CONFIG_BASE | ((uint16_t)activeChannel << CONFIG_ACTIVE_CHAN_SHIFT);
}

bool FDC2214::sleep() {
  return writeRegister16(REG_CONFIG, configValue() | CONFIG_SLEEP_MODE_EN);
}

bool FDC2214::wake() {
  return writeRegister16(REG_CONFIG, configValue());
}

// Writes one 16-bit register. Returns false if the sensor did not acknowledge.
// Called from loop() only (nWriteErrors is not safe to count from two contexts).
bool FDC2214::writeRegister16(byte regID, uint16_t value) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(regID);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)value);
  if (Wire.endTransmission() != 0) {
    nWriteErrors++;
    return false;
  }
  return true;
}

// Reads one 16-bit register: the register address, then a repeated start and two data bytes. The repeated start
// saves the stop condition and the bus idle time between the two halves of the read.
// Returns false, and leaves value alone, unless both bytes arrived. Counts no error: its callers do.
bool FDC2214::readRegister16(byte regID, uint16_t &value) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(regID);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((uint8_t)ADDRESS, (uint8_t)2, (uint8_t)true) != 2) {
    return false;
  }
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  value = ((uint16_t)msb << 8) | lsb;
  return true;
}

// Reads the active channel's conversion result: DATA_CHx first, then DATA_LSB_CHx, as the datasheet requires for
// the two halves to belong to the same conversion (table 6-4). The result is fSENSOR / FIN_SEL / fREF * 2^28.
// Called from the timer interrupt. The sensor converts continuously on its own schedule (every
// (RCOUNT*16 + 4) / fREF, 102.5us by default), so this returns the latest finished conversion, and returns it
// again if no new one has finished.
// A result of 0 means no conversion: the sensor is asleep, or has not finished its first conversion since it woke.
// A tank that oscillates cannot read 0.
bool FDC2214::readSensor(uint32_t &value) {
  uint8_t regAddress = REG_DATA_CH0 + activeChannel*2;
  uint16_t msb = 0;
  uint16_t lsb = 0;
  if (!readRegister16(regAddress, msb) || !readRegister16(regAddress + 1, lsb)) {
    nReadErrors++;
    return false;
  }
  uint32_t result = ((uint32_t)(msb & DATA_MSB_MASK) << 16) | lsb;
  if (result == 0) {
    return false;
  }
  value = result;
  return true;
}

uint32_t FDC2214::nErrors() {
  return nReadErrors + nWriteErrors;
}

// RCOUNT sets the conversion time: (RCOUNT*16 + 4) / fREF. Values below 0x0100 are reserved (6.6.10).
bool FDC2214::set_RCOUNT(uint16_t value) {
  bool ok = sleep();
  ok &= writeRegister16(REG_RCOUNT_CH0, value);
  ok &= writeRegister16(REG_RCOUNT_CH1, value);
  ok &= wake();
  return ok;
}

// SETTLECOUNT sets the wait before the first conversion after a wake: SETTLECOUNT*16 / fREF (6.6.18)
bool FDC2214::set_SETTLECOUNT(uint16_t value) {
  bool ok = sleep();
  ok &= writeRegister16(REG_SETTLECOUNT_CH0, value);
  ok &= writeRegister16(REG_SETTLECOUNT_CH1, value);
  ok &= wake();
  return ok;
}

// fREF = 40MHz / value. The datasheet needs fIN < fREF / 4 (table 6-1), which holds for 1 to 3 with this board's tanks.
// FIN_SEL is written with the same value as in init(), so the scale of the result does not depend on whether this
// was called.
bool FDC2214::set_REF_DIVIDER(uint8_t value) {
  bool ok = sleep();
  ok &= writeRegister16(REG_CLKDIVIDERS_CH0, CLKDIVIDERS_FIN_SEL_DIV2 | value);
  ok &= writeRegister16(REG_CLKDIVIDERS_CH1, CLKDIVIDERS_FIN_SEL_DIV2 | value);
  ok &= wake();
  return ok;
}

// value: 0-31, for 16uA to 1.571mA (6.6.31)
bool FDC2214::set_DRIVE_CURRENT(uint8_t value) {
  bool ok = sleep();
  ok &= writeRegister16(REG_DRIVECURRENT_CH0, (uint16_t)value << DRIVECURRENT_IDRIVE_SHIFT);
  ok &= writeRegister16(REG_DRIVECURRENT_CH1, (uint16_t)value << DRIVECURRENT_IDRIVE_SHIFT);
  ok &= wake();
  return ok;
}

// newChannel: 0 or 1. Writes CONFIG without putting the sensor to sleep first.
bool FDC2214::set_ACTIVE_CHANNEL(uint8_t newChannel) {
  activeChannel = newChannel;
  return wake();
}
