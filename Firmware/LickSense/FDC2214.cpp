/*
----------------------------------------------------------------------------

This file is part of the Sanworks repository
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

// Simplified Library for programming the FDC2214 capacitive sensor as installed on the Sanworks LickSense device


#include <Arduino.h>
#include <Wire.h>
#include "FDC2214.h"

FDC2214::FDC2214(byte clockEnable, byte intB, byte shutDown) {
  pinMode(clockEnable, OUTPUT);
  digitalWrite(clockEnable, HIGH); // Enable external clock oscillator IC
  pinMode(intB, INPUT); 
  pinMode(shutDown, OUTPUT); 
  digitalWrite(shutDown, HIGH); // Reset capacitive sensor IC
  delay(1);
  digitalWrite(shutDown, LOW);
}

void FDC2214::init() {
  Wire.begin(); // join i2c bus
  Wire.setClock(400000); // Set to I2C fast mode

  // Setup FDC2214 sensor IC
  delay(1);
  writeRegister16(REG_RESET, B10000000, B00000000); // Soft reset
  writeRegister16(REG_RCOUNT_CH0, B00000001, B00000000); // Default config: 256 ref clock cycles
  writeRegister16(REG_RCOUNT_CH1, B00000001, B00000000); 
  writeRegister16(REG_SETTLECOUNT_CH0, B00000000, B00001010); // Default config: 10 ref clock cycles
  writeRegister16(REG_SETTLECOUNT_CH1, B00000000, B00001010); 
  writeRegister16(REG_CLKDIVIDERS_CH0, B00010000, B00000001); // Default config: CH0_FIN_SEL = 2, CH0_FREF_DIVIDER = 1
  writeRegister16(REG_CLKDIVIDERS_CH1, B00010000, B00000001);
  writeRegister16(REG_ERRCONFIG, B00000000, B00000001); // Enable data-ready signal on intB pin
  writeRegister16(REG_MUXCONFIG, B00000010, B00001101); // Default config: Disable multiplexed channel sequence, sequence = [Ch0, Ch1], Deglitch filter 10MHz
  writeRegister16(REG_DRIVECURRENT_CH0, B11111000, B00000000); //Set Ch0 to maximum drive current
  writeRegister16(REG_DRIVECURRENT_CH1, B11111000, B00000000); 
  writeRegister16(REG_CONFIG, B00011110, B00000001); // MUST BE LAST - Other config regs can not be programmed while sleep mode is off! 
                                                     // Default config: Sleep mode off, configurable drive power, external clock, intB enabled, Normal current range
}

void FDC2214::sendByte(byte aByte) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(aByte); 
  Wire.endTransmission();
}

void FDC2214::writeRegister16(byte regID, byte msb, byte lsb) {
  typeBuffer.uint8[0] = regID;
  typeBuffer.uint8[1] = msb;
  typeBuffer.uint8[2] = lsb;
  Wire.beginTransmission(ADDRESS);
  Wire.write(typeBuffer.uint8, 3); 
  Wire.endTransmission();
}

uint16_t FDC2214::readRegister16(byte regID) {
  sendByte(regID);
  Wire.requestFrom(ADDRESS, 2);
  typeBuffer.uint8[1] = Wire.read();
  typeBuffer.uint8[0] = Wire.read();
  return typeBuffer.uint16[0];
}

uint32_t FDC2214::readSensor() {
  uint8_t regAddress = activeChannel*2;
  sendByte(regAddress);
  Wire.requestFrom(ADDRESS, 2);
  typeBuffer.uint8[3] = Wire.read();
  typeBuffer.uint8[2] = Wire.read();
  sendByte(regAddress+1);
  Wire.requestFrom(ADDRESS, 2);
  typeBuffer.uint8[1] = Wire.read();
  typeBuffer.uint8[0] = Wire.read();
  bitClear(typeBuffer.uint8[3], 5); // Clear watchdog error bit
  bitClear(typeBuffer.uint8[3], 4); // Clear amplitude warning bit
  return typeBuffer.uint32[0];
}

void FDC2214::set_RCOUNT(uint16_t value) {
  writeRegister16(REG_CONFIG, B00111110+chConfigBit, B00000001); // Sleep
  typeBuffer.uint16[0] = value;
  writeRegister16(REG_RCOUNT_CH0, typeBuffer.uint8[1], typeBuffer.uint8[0]); // Set RCOUNT - Ch0
  writeRegister16(REG_RCOUNT_CH1, typeBuffer.uint8[1], typeBuffer.uint8[0]); // Set RCOUNT - Ch1
  writeRegister16(REG_CONFIG, B00011110+chConfigBit, B00000001); // Wakeup
}

void FDC2214::set_SETTLECOUNT(uint16_t value) {
  writeRegister16(REG_CONFIG, B00111110+chConfigBit, B00000001); // Sleep
  typeBuffer.uint16[0] = value;
  writeRegister16(REG_SETTLECOUNT_CH0, typeBuffer.uint8[1], typeBuffer.uint8[0]); // Set SETTLECOUNT - Ch0
  writeRegister16(REG_SETTLECOUNT_CH1, typeBuffer.uint8[1], typeBuffer.uint8[0]); // Set SETTLECOUNT - Ch1
  writeRegister16(REG_CONFIG, B00011110+chConfigBit, B00000001); // Wakeup
}

void FDC2214::set_REF_DIVIDER(uint8_t value) {
  writeRegister16(REG_CONFIG, B00111110+chConfigBit, B00000001); // Sleep
  writeRegister16(REG_CLKDIVIDERS_CH0, B00100000, value); // Set REF_DIVIDER - Ch0
  writeRegister16(REG_CLKDIVIDERS_CH1, B00100000, value); // Set REF_DIVIDER - Ch1
  writeRegister16(REG_CONFIG, B00011110+chConfigBit, B00000001); // Wakeup
}

void FDC2214::set_DRIVE_CURRENT(uint8_t value) {
  writeRegister16(REG_CONFIG, B00111110+chConfigBit, B00000001); // Sleep
  writeRegister16(REG_DRIVECURRENT_CH0, value << 3, B00000000); // Set DRIVECURRENT - Ch0
  writeRegister16(REG_DRIVECURRENT_CH1, value << 3, B00000000); // Set DRIVECURRENT - Ch1
  writeRegister16(REG_CONFIG, B00011110+chConfigBit, B00000001); // Wakeup
}

void FDC2214::set_ACTIVE_CHANNEL(uint8_t newChannel) {
  activeChannel = newChannel;
  chConfigBit = newChannel*B01000000;
  writeRegister16(REG_CONFIG, B00011110+chConfigBit, B00000001);  
}
