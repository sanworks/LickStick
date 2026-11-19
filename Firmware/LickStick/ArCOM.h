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

// ArCOM reads and writes the data types of the LickStick USB protocol (see /Firmware/PROTOCOL.md).
// It was derived from the general purpose ArCOM library by way of the Pulse Pal firmware's, and is tailored to this
// firmware: Teensy 4.0, USB serial only.
//
// How it behaves:
//
// - Reads return as soon as the bytes have arrived. If no new byte arrives for ARCOM_READ_TIMEOUT_MS, the read gives
//   up, returns zeros and sets timedOut(). Every read after that returns immediately, so the rest of a command that
//   was cut short costs nothing. loop() checks timedOut() once per pass, refuses the command and clears it.
// - Writes are collected in a buffer and sent by flush(), which loop() calls once per pass. One USB packet then
//   carries a whole reply. Nothing is sent until flush() is called.
// - flush() can wait: the Teensy core's write waits up to 120ms when the PC is not reading (TX_TIMEOUT_MSEC in
//   usb_serial.c), then discards data until the PC reads again. That is why this class is only used from loop(),
//   never from the sampling interrupt. The sample stream does not go through this class: see sendSamples() in
//   LickStick.ino.
// - The wire format is little-endian, and so is the Teensy, so the bytes of a value need no rearranging.

#ifndef ArCOM_h
#define ArCOM_h

#include "Arduino.h"

#define ARCOM_READ_TIMEOUT_MS 100  // Longest gap between bytes of one command before the command is abandoned
#define ARCOM_WRITE_BUFFER_SIZE 64 // Must hold the largest reply, which is 32 bytes (op 'G')

class ArCOM
{
  public:
    ArCOM(usb_serial_class &port) : port(port) {}

    // Number of bytes waiting to be read
    unsigned int available() {
      return port.available();
    }

    // True if a read gave up waiting for data. Stays true until clearTimedOut() is called.
    bool timedOut() {
      return readTimedOut;
    }

    void clearTimedOut() {
      readTimedOut = false;
    }

    // --- Reads. Each returns 0 (or leaves zeros in the array) if the data does not arrive. ---

    byte readByte() {
      byte value = 0;
      readBlock(&value, 1);
      return value;
    }

    byte readUint8() {
      return readByte();
    }

    uint16_t readUint16() {
      uint16_t value = 0;
      readBlock(&value, 2);
      return value;
    }

    uint32_t readUint32() {
      uint32_t value = 0;
      readBlock(&value, 4);
      return value;
    }

    float readFloat() {
      float value = 0;
      readBlock(&value, 4);
      return value;
    }

    void readByteArray(byte *values, size_t nValues) {
      readBlock(values, nValues);
    }

    // Reads nBytes into target. Returns false if no new byte arrived for ARCOM_READ_TIMEOUT_MS, in which case the
    // bytes that did not arrive are set to 0, so that a half-read command cannot act on whatever was in memory.
    bool readBlock(void *target, size_t nBytes) {
      uint8_t *nextByte = (uint8_t*)target;
      if (readTimedOut) { // A previous read of this command already failed
        memset(nextByte, 0, nBytes);
        return false;
      }
      size_t nBytesRead = 0;
      uint32_t lastByteTime = millis();
      while (nBytesRead < nBytes) {
        size_t nAvailable = available();
        if (nAvailable > 0) {
          if (nAvailable > (nBytes - nBytesRead)) {
            nAvailable = nBytes - nBytesRead;
          }
          nBytesRead += port.readBytes((char*)(nextByte + nBytesRead), nAvailable);
          lastByteTime = millis();
        } else if ((millis() - lastByteTime) > ARCOM_READ_TIMEOUT_MS) {
          memset(nextByte + nBytesRead, 0, nBytes - nBytesRead);
          readTimedOut = true;
          return false;
        }
      }
      return true;
    }

    // --- Writes. These fill the write buffer; flush() sends it. ---

    void writeByte(byte value) {
      writeBlock(&value, 1);
    }

    void writeUint8(byte value) {
      writeBlock(&value, 1);
    }

    void writeUint16(uint16_t value) {
      writeBlock(&value, 2);
    }

    void writeUint32(uint32_t value) {
      writeBlock(&value, 4);
    }

    void writeByteArray(const byte *values, size_t nValues) {
      writeBlock(values, nValues);
    }

    // Sends everything written since the last call. loop() calls this once per pass.
    void flush() {
      if (nBytesToWrite == 0) {
        return;
      }
      port.write(writeBuffer, nBytesToWrite);
      nBytesToWrite = 0;
      port.send_now(); // Send the packet now, instead of waiting for the USB transmit timer
    }

  private:
    usb_serial_class &port;
    bool readTimedOut = false;
    size_t nBytesToWrite = 0;
    uint8_t writeBuffer[ARCOM_WRITE_BUFFER_SIZE];

    // Adds bytes to the write buffer, sending it first if it is full
    void writeBlock(const void *source, size_t nBytes) {
      const uint8_t *nextByte = (const uint8_t*)source;
      while (nBytes > 0) {
        if (nBytesToWrite == ARCOM_WRITE_BUFFER_SIZE) {
          flush();
        }
        size_t nBytesThisPass = ARCOM_WRITE_BUFFER_SIZE - nBytesToWrite;
        if (nBytesThisPass > nBytes) {
          nBytesThisPass = nBytes;
        }
        memcpy(writeBuffer + nBytesToWrite, nextByte, nBytesThisPass);
        nBytesToWrite += nBytesThisPass;
        nextByte += nBytesThisPass;
        nBytes -= nBytesThisPass;
      }
    }
};

#endif
