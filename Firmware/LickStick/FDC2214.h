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

// Library for programming the Texas Instruments FDC2214 capacitive sensor, as installed on the Sanworks LickStick:
// channels 0 and 1 only, in single-channel mode, clocked by a 40MHz oscillator on CLKIN.
// Register addresses and bit fields are from the FDC2214-Q1 datasheet (TI SNOSCZ9A, October 2024), section 6.6.
//
// The class does not keep the sketch's two contexts apart. readSensor() is called from the timer interrupt, and
// everything else from loop(). Both use the Wire object, so the sketch must make sure the interrupt does not call
// readSensor() while loop() is inside init() or a setter (LickStick.ino does it with its sensorBusy flag).

#ifndef FDC2214_h
#define FDC2214_h
#include "Arduino.h"
#include <Wire.h>

// Sensor I2C Address (0x2A: the ADDR pin is grounded)
#define ADDRESS 42

// Sensor Registers. Each is 16 bits, sent MSB first.
#define REG_DATA_CH0 0     // DATA_CHx is at 2*channel: error flags, and bits 27:16 of the conversion result
#define REG_DATA_LSB_CH0 1 // DATA_LSB_CHx is at 2*channel + 1: bits 15:0 of the conversion result
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

// DATA_CHx (6.6.1). Bits 15:12 are flags; bits 13 and 12 (watchdog and amplitude) stay 0 as ERROR_CONFIG is set here
#define DATA_MSB_MASK 0x0FFF

// CLOCK_DIVIDERS_CHx (6.6.22)
#define CLKDIVIDERS_FIN_SEL_DIV2 0x2000 // FIN_SEL (bits 13:12) = b10: sensor frequency divided by 2. The datasheet's
                                        // setting for a single-ended sensor (7.1.1), which the drink tube is. b01
                                        // (divide by 1) is for differential sensors from 0.01 to 8.75MHz
#define CLKDIVIDERS_FREF_DIVIDER_MASK 0x03FF // FREF_DIVIDER (bits 9:0): fREF = 40MHz / FREF_DIVIDER. 0 is reserved

// ERROR_CONFIG (6.6.27)
#define ERRCONFIG_DRDY_2INT 0x0001 // Report data ready on the INTB pin. All other bits 0: the watchdog and amplitude
                                   // warnings are reported neither in the data registers nor on INTB

// CONFIG (6.6.28)
#define CONFIG_ACTIVE_CHAN_SHIFT 14        // ACTIVE_CHAN (bits 15:14): the channel converted in single-channel mode
#define CONFIG_SLEEP_MODE_EN 0x2000        // 1 = asleep: no conversions, and the data registers read 0
#define CONFIG_SENSOR_ACTIVATE_SEL 0x0800  // 1 = activate the sensor with the DRIVE_CURRENT setting (0 = full current)
#define CONFIG_REF_CLK_SRC 0x0200          // 1 = external clock on CLKIN (the board's 40MHz oscillator)
#define CONFIG_INTB_DIS 0x0080             // 1 = INTB pin disabled
#define CONFIG_HIGH_CURRENT_DRV 0x0040     // 1 = high sensor current (channel 0 only)
#define CONFIG_RESERVED 0x1401             // Reserved bits that must be written as 1: bits 12 and 10, and bit 0 of
                                           // bits 5:0 (000001)
// CONFIG as this board uses it. Add the channel, and CONFIG_SLEEP_MODE_EN to sleep
#define CONFIG_BASE (CONFIG_RESERVED | CONFIG_SENSOR_ACTIVATE_SEL | CONFIG_REF_CLK_SRC)

// MUX_CONFIG (6.6.29)
#define MUXCONFIG_RESERVED 0x0208      // Reserved bits 12:3 must be written as 00 0100 0001
#define MUXCONFIG_DEGLITCH_10MHZ 0x0005 // DEGLITCH (bits 2:0) = b101: the lowest filter setting above the tank's
                                        // frequency, about 5.4MHz
// AUTOSCAN_EN (bit 15) = 0: single-channel mode, on the channel CONFIG.ACTIVE_CHAN selects
#define MUXCONFIG_SINGLE_CHANNEL (MUXCONFIG_RESERVED | MUXCONFIG_DEGLITCH_10MHZ)

// RESET_DEV (6.6.30)
#define RESET_DEV 0x8000 // Returns every register to its default, which leaves the sensor asleep

// DRIVE_CURRENT_CHx (6.6.31)
#define DRIVECURRENT_IDRIVE_SHIFT 11 // IDRIVE (bits 15:11): 0-31, for 16uA to 1.571mA

// Limits of the settings (the firmware refuses values outside them: see LickStick.ino)
#define RCOUNT_MIN 256        // RCOUNT below 0x0100 is reserved (6.6.10)
#define REF_DIVIDER_MIN 1     // FREF_DIVIDER 0 is reserved (6.6.22)
#define DRIVE_CURRENT_MAX 31
#define N_CHANNELS 2          // Channels 2 and 3 are not connected on the board, and are never configured

class FDC2214
{
public:
  // Stores the pin numbers. Touches no hardware: a global object's constructor runs before the Arduino core is ready
  FDC2214(byte clockEnable, byte intB, byte shutDown);

  // Call from setup(). Starts the board's oscillator, resets the sensor, programs its registers and starts
  // conversions. Returns false if a register write was not acknowledged.
  bool init(uint16_t rCount, uint16_t settleCount, uint8_t refDivider, uint8_t driveCurrent, uint8_t channel);

  // Reads the active channel's 28-bit conversion result. Called from the timer interrupt.
  // Returns false, and leaves value alone, if the sensor has no result (it reads 0 while asleep, and until the first
  // conversion after it wakes) or if the I2C read failed.
  bool readSensor(uint32_t &value);

  bool readRegister16(byte regID, uint16_t &value);
  bool writeRegister16(byte regID, uint16_t value);

  // The setters below are called from loop(), and program channels 0 and 1 alike. All but set_ACTIVE_CHANNEL() put
  // the sensor to sleep meanwhile. Each returns false if a register write was not acknowledged.
  // None checks its argument: see the limits above.
  bool set_RCOUNT(uint16_t value);
  bool set_SETTLECOUNT(uint16_t value);
  bool set_REF_DIVIDER(uint8_t value);
  bool set_DRIVE_CURRENT(uint8_t value);
  bool set_ACTIVE_CHANNEL(uint8_t value);

  // Number of I2C transfers that failed since power-on (reads from the interrupt, and writes from loop())
  uint32_t nErrors();

private:
  bool sleep();
  bool wake();
  uint16_t configValue(); // CONFIG for the active channel, awake
  byte clockEnablePin;
  byte intBPin;
  byte shutDownPin;
  byte activeChannel = 0;
  volatile uint32_t nReadErrors = 0; // Counted by readSensor(), in the interrupt
  uint32_t nWriteErrors = 0;         // Counted by writeRegister16(), in loop()
};
#endif
