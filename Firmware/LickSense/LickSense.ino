/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks LickSense repository
  Copyright (C) Sanworks LLC, Rochester, New York, USA

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

#include <Wire.h>    // Arduino's builtin I2C class
#include "ArCOM.h"   // A wrapper to simplify transactions of different datatypes via USB serial
#include "FDC2214.h" // A class to interface with the Texas Instruments FDC2214 capacitive sensor IC

// Firmware version
#define FIRMWARE_VERSION 1

// Pin definitions
#define CLK_EN 10  // Clock enable
#define INTB 11    // Interrupt pin (configured below to report data ready)
#define SHTDWN 12  // Shutdown pin (used to reset device)
#define DIO1 15    // Digital I/O pin 1 (Middle position on green screw terminal header)
#define DIO2 14    // Digital I/O pin 2 (Right position on green screw terminal header)
#define LED_PIN 13 // LED

// Parameters
#define READ_INTERVAL 500 // Sensor read interval, units = µs
#define THRESH_DEFAULT 1900000 // Default touch detection threshold. Used from power-on until the PC connects.

// Setup interfaces
ArCOM USBCOM(Serial);
FDC2214 SensorIC(CLK_EN, INTB, SHTDWN);

// Timer used to read the sensor at even intervals
IntervalTimer readTimer;

// Program variables
byte opByte = 0;
byte inByte = 0;
byte usbStreaming = 0;
byte lickDetected = 0;
byte ledEnabled = 0;
uint16_t regValue = 0;
uint32_t thresholdValue = 0;
uint32_t nDebounceCycles = 20; // 1ms cycles = 50Hz max licking
uint32_t debounceCounter = 0;
uint32_t nSamplesToReturn = 0;
union {
    uint8_t uint8[4];
    uint16_t uint16[2];
    uint32_t uint32[1];
} sensorValue;

void setup() {
  Wire.begin();
  SensorIC.init();
  pinMode(DIO1, OUTPUT);
  digitalWrite(DIO1, LOW);
  pinMode(DIO2, OUTPUT);
  digitalWrite(DIO2, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  thresholdValue = THRESH_DEFAULT;
  readTimer.begin(readCycle, READ_INTERVAL);
}

void loop() {
  if (USBCOM.available() > 0) {
    opByte = USBCOM.readByte();
    switch(opByte) {
      case 'F': // Return firmware version
        USBCOM.writeUint32(FIRMWARE_VERSION);
      break;
      case 'R': // Read sensor value
        nSamplesToReturn = USBCOM.readUint32();
        usbStreaming = 1;
      break;
      case 'S': // Start/stop USB streaming
        nSamplesToReturn = 0; // Continuous streaming
        usbStreaming = USBCOM.readByte();
      break;
      case 'W': // Set read count
        regValue = USBCOM.readUint16();
        SensorIC.set_RCOUNT(regValue);
      break;
      case 'N': // Set settle count
        regValue = USBCOM.readUint16();
        SensorIC.set_SETTLECOUNT(regValue);
      break;
      case 'D': // Set reference divider
        inByte = USBCOM.readUint8();
        SensorIC.set_REF_DIVIDER(inByte);
      break;
      case 'C': // Set drive current
        inByte = USBCOM.readUint8();
        SensorIC.set_DRIVE_CURRENT(inByte);
      break;
      case 'T': // Set threshold
        thresholdValue = USBCOM.readUint32();
      break;
      case '!': // Set active channel
        inByte = USBCOM.readByte();
        SensorIC.set_ACTIVE_CHANNEL(inByte);
      break;
      case 'L': // set LED enable/disable
        ledEnabled = USBCOM.readByte();
      break;
    }
  }
}

void readCycle() {
  sensorValue.uint32[0] = SensorIC.readSensor();
  if (debounceCounter == 0) {
    if (sensorValue.uint32[0] > thresholdValue) {
      setTTL(0);
      if (lickDetected == 1) {
        debounceCounter = nDebounceCycles;
      }
      lickDetected = 0;
    } else {
      setTTL(1);
      if (lickDetected == 0) {
        debounceCounter = nDebounceCycles;
      }
      lickDetected = 1;
    }
  } else {
    debounceCounter--;
  }
  bitWrite(sensorValue.uint8[0], 0, lickDetected); // Use LSB to encode lick detection. Bit is read and cleared on PC side
  if (usbStreaming) {
    USBCOM.writeUint32(sensorValue.uint32[0]);
    if (nSamplesToReturn > 0) {
      nSamplesToReturn--;
      if (nSamplesToReturn == 0) {
        usbStreaming = false;
      }
    }
  }
}

void setTTL(uint8_t level) {
  digitalWriteFast(DIO1, level);
  digitalWriteFast(DIO2, level);
  if (ledEnabled) {
    digitalWriteFast(LED_PIN, level);
  }
}
