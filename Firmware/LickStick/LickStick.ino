/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks LickStick repository
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

// LickStick firmware, for Teensy 4.0 (Arduino IDE with the Teensy board package).
//
// A drink tube is part of an LC tank whose frequency a TI FDC2214 measures. This sketch reads the FDC2214 over
// I2C at a fixed rate, compares each reading with a threshold, drives two TTL outputs high while a lick is
// detected, and streams the readings to the PC on request.
//
// Two contexts:
// - readCycle() is the IntervalTimer callback. It reads the sensor, detects licks, sets the outputs and, while
//   streaming, puts the sample in sampleBuffer. It runs from power-on, with or without a PC, above the priority of
//   the USB interrupt, and never touches the USB port.
// - loop() sends the buffered samples to the PC and handles the PC's commands, one per pass. It is the only user
//   of the USB port, and may wait on it.
// The interrupt can run at any point in loop(). What the two share is listed under "Program variables" below, and
// the rules are in "How the firmware runs" in /AGENTS.md: read them before changing either.
//
// The USB protocol is documented in /Firmware/PROTOCOL.md. Its op codes and message layouts are fixed:
// installed copies of the MATLAB and Python classes rely on them. Add a new op instead of changing one.

#include <Wire.h>    // Arduino's builtin I2C class
#include <EEPROM.h>  // Settings stored by op 'E' (emulated in the Teensy's flash memory)
#include "ArCOM.h"   // A wrapper to simplify transactions of different datatypes via USB serial
#include "FDC2214.h" // A class to interface with the Texas Instruments FDC2214 capacitive sensor IC

// Firmware version, returned by op 'F'. A change in behavior gets a new number, a new file in
// /Firmware/Compiled, and the same number in the clients (see /AGENTS.md)
#define FIRMWARE_VERSION 2

// Pin definitions
#define CLK_EN 10  // Clock enable
#define INTB 11    // Interrupt pin (the FDC2214 is configured to report data ready on it; not read by this sketch)
#define SHTDWN 12  // Shutdown pin (used to reset device)
#define DIO1 15    // Digital I/O pin 1 (Middle position on green screw terminal header)
#define DIO2 14    // Digital I/O pin 2 (Right position on green screw terminal header)
#define LED_PIN 13 // LED

// Factory settings: used from power-on unless op 'E' has stored others
#define THRESH_DEFAULT 1900000        // Touch detection threshold. The sensor reads about 18 million (v1.0.1 board),
                                      // far above this, so no lick is detected until the PC sets a threshold. That
                                      // is deliberate: the right threshold depends on the tube and its cable, and a
                                      // threshold above the resting signal would hold the lick outputs high
#define SAMPLING_INTERVAL_DEFAULT 500 // Microseconds (2kHz)
#define RCOUNT_DEFAULT 256            // Conversion time 102.5us: (RCOUNT*16 + 4) / fREF
#define SETTLECOUNT_DEFAULT 10
#define REF_DIVIDER_DEFAULT 1         // fREF = 40MHz
#define DRIVE_CURRENT_DEFAULT 31      // Maximum
#define ACTIVE_CHANNEL_DEFAULT 0

// Limits of op 'I'. The limits of the sensor settings are in FDC2214.h
#define SAMPLING_INTERVAL_MIN 400     // Microseconds. readCycle() takes 241us (measured: see maxReadCycleMicros),
                                      // and loop() runs only in what is left of each interval
#define SAMPLING_INTERVAL_MAX 1000000

// Timing
#define DEBOUNCE_MICROS 10000         // After each change of the lick state, the state is held for this long
#define SENSOR_TIMEOUT_MICROS 250000  // A lick is ended if the sensor has given no reading for this long. Longer
                                      // than the first conversion after a setting op takes with any settings the
                                      // datasheet allows (160ms: rCount and settleCount 65535, refDivider 3)
#define TIMER_PRIORITY 64             // Of the sampling interrupt. Above USB (128), so that USB traffic cannot
                                      // delay a sample; below the system tick (32), which the I2C timeouts need

// Sample buffer
#define SAMPLE_BUFFER_SIZE 32768      // Samples (16s at 2kHz). Must be a power of 2
#define SEND_BUFFER_SIZE 2048         // Bytes of samples sent per pass of loop(), at most

// Streaming formats
#define STREAM_SAMPLES 1 // Ops 'S' and 'R': one uint32 sample word per sample
#define STREAM_INDEXED 2 // Op 'X': uint32 sample number, then the sample word

// Stored settings (op 'E')
#define STORED_SETTINGS_ADDRESS 0
#define STORED_SETTINGS_KEY 0x4C53544B // "LSTK". Change it if the layout of StoredSettings changes

// Setup interfaces
ArCOM USBCOM(Serial);
FDC2214 SensorIC(CLK_EN, INTB, SHTDWN);

// Timer used to read the sensor at even intervals
IntervalTimer readTimer;

// Program variables
//
// Shared by loop() and readCycle(). Each is written in one instruction, and loop() never needs two of them to
// change together while the interrupt is using them:
// - Written by loop(), read by readCycle(): thresholdValue, ledEnabled, nDebounceCycles, nSensorTimeoutCycles,
//   sensorBusy, sampleTail.
// - Written by readCycle(), read by loop(): sampleHead, nSamplesDropped, maxReadCycleMicros.
// - usbStreaming is set by loop() and cleared by either. sampleNumber, nSamplesToReturn and nSamplesDropped are
//   readCycle()'s while usbStreaming is set; loop() writes them only after clearing it (see startStreaming()).
volatile uint32_t thresholdValue = THRESH_DEFAULT;
volatile byte ledEnabled = 0;
volatile uint32_t nDebounceCycles = 20;      // DEBOUNCE_MICROS in samples. Set by setSamplingInterval()
volatile uint32_t nSensorTimeoutCycles = 500; // SENSOR_TIMEOUT_MICROS in samples. Set by setSamplingInterval()
volatile bool sensorBusy = false;            // True while loop() is writing the sensor's registers: readCycle() then
                                             // stays off the I2C bus
volatile byte usbStreaming = 0;              // True while readCycle() is to add its samples to sampleBuffer
volatile uint32_t nSamplesToReturn = 0;      // Samples left to send for op 'R'. 0 = no limit (continuous streaming)
volatile uint32_t sampleNumber = 0;          // Sampling intervals since streaming started
volatile uint32_t nSamplesDropped = 0;       // Samples lost since streaming started, because sampleBuffer was full
volatile uint32_t maxReadCycleMicros = 0;    // Longest run of readCycle() since op 'G' last asked

// Used by readCycle() only
byte lickDetected = 0;
uint32_t sensorValue = 0;      // The last reading the sensor gave. Repeated in the stream while it gives none
uint32_t debounceCounter = 0;
uint32_t nInvalidCycles = 0;   // Samples in a row for which the sensor gave no reading

// Used by loop() only
byte opByte = 0;
byte streamFormat = STREAM_SAMPLES; // Format of the samples in sampleBuffer
bool clientConnected = false;       // The PC's DTR signal at the last pass: high while a program has the port open
uint16_t nCommandErrors = 0;        // Commands refused since power-on (bad argument, unknown op, or cut short)
byte storedSettingsLoaded = 0;      // 1 if the settings at power-on came from op 'E', 0 if they were the factory's
float samplingInterval = SAMPLING_INTERVAL_DEFAULT; // Hardware timer period in microseconds (500 = 2kHz)
// The firmware's copy of the sensor settings. Each is updated only when the sensor acknowledged the new value
uint16_t rCount = RCOUNT_DEFAULT;
uint16_t settleCount = SETTLECOUNT_DEFAULT;
uint8_t refDivider = REF_DIVIDER_DEFAULT;
uint8_t driveCurrent = DRIVE_CURRENT_DEFAULT;
uint8_t activeChannel = ACTIVE_CHANNEL_DEFAULT;

// Samples waiting to be sent to the PC. readCycle() adds at sampleHead, and sendSamples() (loop()) takes from
// sampleTail. Empty when the two are equal; one slot stays free, so that full and empty differ.
// The buffer is what keeps the PC's pauses out of the sampling interrupt: while the PC is not reading, samples
// collect here (and are counted in nSamplesDropped once it is full), and sampling and lick detection carry on.
struct BufferedSample {
  uint32_t number; // sampleNumber when the sample was taken
  uint32_t word;   // The sample word, as sent (see readCycle())
};
DMAMEM BufferedSample sampleBuffer[SAMPLE_BUFFER_SIZE]; // In RAM2, with the USB buffers: 256kB
volatile uint32_t sampleHead = 0;
volatile uint32_t sampleTail = 0;
#define SAMPLE_BUFFER_MASK (SAMPLE_BUFFER_SIZE - 1)

// What op 'E' stores. The layout is fixed by STORED_SETTINGS_KEY
struct StoredSettings {
  uint32_t key;
  uint32_t thresholdValue;
  float samplingInterval;
  uint16_t rCount;
  uint16_t settleCount;
  uint8_t refDivider;
  uint8_t driveCurrent;
  uint8_t activeChannel;
  uint8_t ledEnabled;
  uint32_t checksum; // Of the bytes before it (see settingsChecksum())
};

// The reply to op 'G'. The layout is part of the protocol: see /Firmware/PROTOCOL.md
struct __attribute__((packed)) StatusReply {
  uint32_t thresholdValue;
  float samplingInterval;
  uint16_t rCount;
  uint16_t settleCount;
  uint8_t refDivider;
  uint8_t driveCurrent;
  uint8_t activeChannel;
  uint8_t ledEnabled;
  uint8_t streaming;            // 0, STREAM_SAMPLES or STREAM_INDEXED
  uint8_t storedSettingsLoaded;
  uint16_t nCommandErrors;
  uint32_t nSamplesDropped;
  uint32_t nSensorErrors;
  uint32_t maxReadCycleMicros;
};

void setup() {
  pinMode(DIO1, OUTPUT);
  digitalWrite(DIO1, LOW);
  pinMode(DIO2, OUTPUT);
  digitalWrite(DIO2, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  loadStoredSettings();
  SensorIC.init(rCount, settleCount, refDivider, driveCurrent, activeChannel);
  setSamplingInterval(samplingInterval);
  readTimer.priority(TIMER_PRIORITY);
  readTimer.begin(readCycle, samplingInterval);
}

// Sends buffered samples to the PC, then handles one command from the PC, then sends the command's reply.
// Ops 'F', 'G' and 'E' reply; 'R', 'S' and 'X' start or stop the sample stream; no other op sends anything.
// A command is refused, and counted in nCommandErrors, if an argument is outside its limits, if the op is unknown,
// or if its bytes stop arriving for 100ms (USBCOM then returns zeros: see ArCOM.h).
// The ops that write the sensor's registers set sensorBusy meanwhile, which keeps readCycle() off the I2C bus.
void loop() {
  bool dtr = Serial.dtr();
  if (clientConnected && !dtr) {
    stopStreaming(); // The program on the PC closed the port: the next one must not find a stream in progress
  }
  clientConnected = dtr;
  sendSamples();
  if (USBCOM.available() > 0) {
    opByte = USBCOM.readByte();
    switch(opByte) {
      case 'F': // Return firmware version
        USBCOM.writeUint32(FIRMWARE_VERSION);
      break;
      case 'R': { // Read sensor value: send the next nSamples samples, one per sampling interval
        uint32_t nSamples = USBCOM.readUint32();
        if (commandValid(true)) {
          startStreaming(STREAM_SAMPLES, nSamples); // 0 = continuous streaming
        }
      } break;
      case 'S': { // Start/stop USB streaming
        byte enable = USBCOM.readByte();
        if (commandValid(true)) {
          if (enable) {
            startStreaming(STREAM_SAMPLES, 0);
          } else {
            stopStreaming();
          }
        }
      } break;
      case 'X': { // Start/stop USB streaming, with each sample's number
        byte enable = USBCOM.readByte();
        if (commandValid(true)) {
          if (enable) {
            startStreaming(STREAM_INDEXED, 0);
          } else {
            stopStreaming();
          }
        }
      } break;
      case 'W': { // Set read count
        uint16_t value = USBCOM.readUint16();
        if (commandValid(value >= RCOUNT_MIN)) {
          sensorBusy = true;
          if (SensorIC.set_RCOUNT(value)) {
            rCount = value;
          }
          sensorBusy = false;
        }
      } break;
      case 'N': { // Set settle count. Every value is valid
        uint16_t value = USBCOM.readUint16();
        if (commandValid(true)) {
          sensorBusy = true;
          if (SensorIC.set_SETTLECOUNT(value)) {
            settleCount = value;
          }
          sensorBusy = false;
        }
      } break;
      case 'D': { // Set reference divider
        uint8_t value = USBCOM.readUint8();
        if (commandValid(value >= REF_DIVIDER_MIN)) {
          sensorBusy = true;
          if (SensorIC.set_REF_DIVIDER(value)) {
            refDivider = value;
          }
          sensorBusy = false;
        }
      } break;
      case 'C': { // Set drive current
        uint8_t value = USBCOM.readUint8();
        if (commandValid(value <= DRIVE_CURRENT_MAX)) {
          sensorBusy = true;
          if (SensorIC.set_DRIVE_CURRENT(value)) {
            driveCurrent = value;
          }
          sensorBusy = false;
        }
      } break;
      case 'T': { // Set threshold. The only setting the clients change during streaming
        uint32_t value = USBCOM.readUint32();
        if (commandValid(true)) {
          thresholdValue = value;
        }
      } break;
      case '!': { // Set active channel (0 or 1. The clients call them channels 1 and 2)
        uint8_t value = USBCOM.readByte();
        if (commandValid(value < N_CHANNELS)) {
          sensorBusy = true;
          if (SensorIC.set_ACTIVE_CHANNEL(value)) {
            activeChannel = value;
          }
          sensorBusy = false;
        }
      } break;
      case 'I': { // Set sampling interval (4 bytes: a float, in microseconds)
        float value = USBCOM.readFloat();
        if (commandValid(samplingIntervalValid(value))) {
          setSamplingInterval(value);
          readTimer.update(samplingInterval); // Takes effect when the interval in progress ends
        }
      } break;
      case 'L': { // set LED enable/disable. Disabling leaves the LED in its current state
        byte enable = USBCOM.readByte();
        if (commandValid(true)) {
          ledEnabled = enable;
        }
      } break;
      case 'G': // Return the settings and the status counters
        sendStatus();
      break;
      case 'E': { // Store the current settings as the power-on settings (1), or erase the stored settings (0)
        byte store = USBCOM.readByte();
        if (!USBCOM.timedOut()) { // A command cut short gets no reply: nobody is waiting for one
          if (commandValid(store <= 1)) {
            if (store) {
              storeSettings();
            } else {
              eraseStoredSettings();
            }
            USBCOM.writeByte(1);
          } else {
            USBCOM.writeByte(0);
          }
        }
      } break;
      default: // Not an op
        nCommandErrors++;
      break;
    }
    if (USBCOM.timedOut()) { // The command was cut short. Its case above did nothing
      nCommandErrors++;
      USBCOM.clearTimedOut();
    }
    USBCOM.flush();
  }
}

// True if the command just read may be carried out: all of its bytes arrived, and its arguments are valid.
// Counts a refusal for a bad argument. loop() counts the one for a command cut short.
bool commandValid(bool argumentsValid) {
  if (USBCOM.timedOut()) {
    return false;
  }
  if (!argumentsValid) {
    nCommandErrors++;
    return false;
  }
  return true;
}

// Written so that NaN is refused
bool samplingIntervalValid(float interval) {
  return (interval >= SAMPLING_INTERVAL_MIN) && (interval <= SAMPLING_INTERVAL_MAX);
}

// Sets the sampling interval, and the times that readCycle() counts in samples. The caller updates the timer.
void setSamplingInterval(float interval) {
  samplingInterval = interval;
  uint32_t nCycles = (uint32_t)(DEBOUNCE_MICROS / interval + 0.5f);
  nDebounceCycles = (nCycles > 0) ? nCycles : 1;
  nCycles = (uint32_t)(SENSOR_TIMEOUT_MICROS / interval + 0.5f);
  nSensorTimeoutCycles = (nCycles > 0) ? nCycles : 1;
}

void startStreaming(byte format, uint32_t nSamples) {
  usbStreaming = 0; // readCycle() adds no sample from here on, so the buffer and the counters are loop()'s to change
  if (format != streamFormat) {
    sampleTail = sampleHead; // Discard samples of the other format
    streamFormat = format;
  }
  nSamplesDropped = 0;
  sampleNumber = 0;
  nSamplesToReturn = nSamples;
  usbStreaming = 1;
}

// Stops streaming and discards the samples not yet sent. The clients discard what arrives after a stop anyway, and
// a device that is quiet soon after the stop can be told apart from one that is still streaming.
void stopStreaming() {
  usbStreaming = 0;
  sampleTail = sampleHead;
}

// Sends samples from sampleBuffer, as many as the USB port takes without waiting. Called once per pass of loop().
// The Teensy core's write waits up to 120ms once its buffers are full, which happens when the PC stops reading.
// availableForWrite() is the room left before that, so a write of no more never waits. Whole samples only, so that
// the PC's position in the stream stays aligned.
void sendSamples() {
  static uint8_t sendBuffer[SEND_BUFFER_SIZE];
  uint32_t nSamples = (sampleHead - sampleTail) & SAMPLE_BUFFER_MASK;
  if (nSamples == 0) {
    return;
  }
  uint32_t sampleSize = (streamFormat == STREAM_INDEXED) ? 8 : 4;
  uint32_t nBytesFree = Serial.availableForWrite();
  if (nBytesFree > SEND_BUFFER_SIZE) {
    nBytesFree = SEND_BUFFER_SIZE;
  }
  if (nSamples > nBytesFree / sampleSize) {
    nSamples = nBytesFree / sampleSize;
  }
  if (nSamples == 0) {
    return;
  }
  uint32_t tail = sampleTail;
  uint8_t *nextByte = sendBuffer;
  for (uint32_t i = 0; i < nSamples; i++) {
    if (streamFormat == STREAM_INDEXED) {
      memcpy(nextByte, &sampleBuffer[tail].number, 4);
      nextByte += 4;
    }
    memcpy(nextByte, &sampleBuffer[tail].word, 4);
    nextByte += 4;
    tail = (tail + 1) & SAMPLE_BUFFER_MASK;
  }
  Serial.write(sendBuffer, nSamples * sampleSize); // Sends nothing if USB is not connected: the samples are discarded
  sampleTail = tail;
}

// Reply to op 'G'
void sendStatus() {
  StatusReply status;
  status.thresholdValue = thresholdValue;
  status.samplingInterval = samplingInterval;
  status.rCount = rCount;
  status.settleCount = settleCount;
  status.refDivider = refDivider;
  status.driveCurrent = driveCurrent;
  status.activeChannel = activeChannel;
  status.ledEnabled = ledEnabled ? 1 : 0;
  status.streaming = usbStreaming ? streamFormat : 0;
  status.storedSettingsLoaded = storedSettingsLoaded;
  status.nCommandErrors = nCommandErrors;
  status.nSamplesDropped = nSamplesDropped;
  status.nSensorErrors = SensorIC.nErrors();
  status.maxReadCycleMicros = maxReadCycleMicros;
  maxReadCycleMicros = 0;
  USBCOM.writeByteArray((const byte*)&status, sizeof(status));
}

uint32_t settingsChecksum(const uint8_t *bytes, size_t nBytes) {
  uint32_t checksum = 0;
  for (size_t i = 0; i < nBytes; i++) {
    checksum = checksum * 31 + bytes[i];
  }
  return checksum;
}

// Op 'E' 1. EEPROM.put() writes only the bytes that differ from what is stored.
// The Teensy core masks all interrupts while it writes its flash memory, so sampling and lick detection pause
// for each byte written, and for longer when the core has to erase a flash sector to make room.
void storeSettings() {
  StoredSettings stored;
  stored.key = STORED_SETTINGS_KEY;
  stored.thresholdValue = thresholdValue;
  stored.samplingInterval = samplingInterval;
  stored.rCount = rCount;
  stored.settleCount = settleCount;
  stored.refDivider = refDivider;
  stored.driveCurrent = driveCurrent;
  stored.activeChannel = activeChannel;
  stored.ledEnabled = ledEnabled ? 1 : 0;
  stored.checksum = settingsChecksum((const uint8_t*)&stored, offsetof(StoredSettings, checksum));
  EEPROM.put(STORED_SETTINGS_ADDRESS, stored);
}

// Op 'E' 0. The device then starts with the factory settings
void eraseStoredSettings() {
  uint32_t noKey = 0;
  EEPROM.put(STORED_SETTINGS_ADDRESS, noKey);
}

// Called from setup(), before the sensor and the timer are started. Takes the stored settings if op 'E' stored a
// set that is whole and within the limits that the ops apply, and leaves the factory settings otherwise.
void loadStoredSettings() {
  StoredSettings stored;
  EEPROM.get(STORED_SETTINGS_ADDRESS, stored);
  if (stored.key != STORED_SETTINGS_KEY) {
    return;
  }
  if (stored.checksum != settingsChecksum((const uint8_t*)&stored, offsetof(StoredSettings, checksum))) {
    return;
  }
  if (!samplingIntervalValid(stored.samplingInterval) || (stored.rCount < RCOUNT_MIN)
      || (stored.refDivider < REF_DIVIDER_MIN) || (stored.driveCurrent > DRIVE_CURRENT_MAX)
      || (stored.activeChannel >= N_CHANNELS)) {
    return;
  }
  thresholdValue = stored.thresholdValue;
  samplingInterval = stored.samplingInterval;
  rCount = stored.rCount;
  settleCount = stored.settleCount;
  refDivider = stored.refDivider;
  driveCurrent = stored.driveCurrent;
  activeChannel = stored.activeChannel;
  ledEnabled = stored.ledEnabled;
  storedSettingsLoaded = 1;
}

// Timer interrupt, once per sampling interval. Reads the sensor, updates the lick state and the outputs, and
// buffers the sample for the PC while streaming.
// - readSensor() makes two blocking I2C reads, which take about half of the 500us default interval.
// - The sensor gives no reading while loop() is changing its settings (sensorBusy), while it is asleep, and until
//   its first conversion after it wakes. The lick state is then held, and the stream repeats the last reading: a
//   setting op must not look like a lick. If no reading comes for SENSOR_TIMEOUT_MICROS, a lick is ended, so that
//   a sensor that fails during a lick does not hold the outputs high.
// - A reading at or below the threshold is a lick: a tongue on the tube lowers the tank's frequency.
// - After each change of state, the state is held for nDebounceCycles samples (DEBOUNCE_MICROS).
// - Nothing here may use the USB port, or wait for anything but the I2C bus: this runs above the USB interrupt.
void readCycle() {
  uint32_t startMicros = micros();
  bool valid = false;
  if (!sensorBusy) {
    valid = SensorIC.readSensor(sensorValue); // Leaves sensorValue alone if the sensor has no reading
  }
  if (valid) {
    nInvalidCycles = 0;
  } else if (nInvalidCycles < nSensorTimeoutCycles) {
    nInvalidCycles++;
  } else if (lickDetected) {
    lickDetected = 0;
    setTTL(0);
  }
  if (debounceCounter == 0) {
    if (valid) {
      byte newState = (sensorValue <= thresholdValue) ? 1 : 0;
      setTTL(newState);
      if (newState != lickDetected) {
        debounceCounter = nDebounceCycles;
      }
      lickDetected = newState;
    }
  } else {
    debounceCounter--;
  }
  if (usbStreaming) {
    uint32_t nextHead = (sampleHead + 1) & SAMPLE_BUFFER_MASK;
    if (nextHead == sampleTail) {
      nSamplesDropped++; // The PC has not read for so long that the buffer is full
    } else {
      sampleBuffer[sampleHead].number = sampleNumber;
      sampleBuffer[sampleHead].word = (sensorValue & 0xFFFFFFFE) | lickDetected; // Use LSB to encode lick detection.
                                                                                 // Bit is read and cleared on PC side
      sampleHead = nextHead;
    }
    sampleNumber++;
    if (nSamplesToReturn > 0) {
      nSamplesToReturn--;
      if (nSamplesToReturn == 0) {
        usbStreaming = 0;
      }
    }
  }
  uint32_t elapsedMicros = micros() - startMicros;
  if (elapsedMicros > maxReadCycleMicros) {
    maxReadCycleMicros = elapsedMicros;
  }
}

void setTTL(uint8_t level) {
  digitalWriteFast(DIO1, level);
  digitalWriteFast(DIO2, level);
  if (ledEnabled) {
    digitalWriteFast(LED_PIN, level);
  }
}
