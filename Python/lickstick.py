"""
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
"""

# Python class to interface with the LickStick device, a capacitive lick detector powered by the
# Texas Instruments FDC2214-Q1 sensor IC. A port of the MATLAB class (/MATLAB/LickStick.m), with the
# same property and method names. See "Known issues" in /AGENTS.md.
#
# Usage:
# from lickstick import LickStick
# L = LickStick('COM3')          # Where COM3 is the device's USB serial port. Programs the defaults
# L.rCount = 256                 # Update the device params by setting attributes
# L.autoSetThreshold()           # Automatically set the lick detection threshold
# values = L.readSensor(10)      # Read sensor values (units = bits)
# L.stream()                     # Launch the streaming GUI. Returns when its window is closed
# data = L.acquiredData          # Stored data from the last streaming GUI session
#
# To record while a protocol runs:
# L.stream(block=False)          # Returns at once. A thread records the samples from here on
# L.update()                     # Call regularly (it redraws the window and handles its buttons)
# L.endAcq()                     # Ends the recording and closes the window
#
# The sensor value falls when the drink tube is touched: a lick is detected while the value is at or
# below the threshold. See /USING.md for setup, the parameters, the recorded data and pitfalls, and
# /Firmware/PROTOCOL.md for the bytes each call sends.

import os
import serial
import struct
import threading
import numpy as np
import time
import datetime
import platform
import sys
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class LickStick:
    def __init__(self, portName):
        self._currentFirmwareVersion = 2  # The firmware version this class works with
        self._readTimeout = 3  # A read gives up when no byte has arrived for this long (s)
        self._initialized = False
        self._streaming = False

        # timeout=0.1: a read returns when the bytes it asked for have arrived, or after 100 ms with
        # those that have. _read_exact() repeats it. The baud rate is ignored by the device (USB CDC)
        self.Port = serial.Serial(portName, 480000000, timeout=0.1)
        try:
            self._handshake()
        except Exception:
            self.Port.close()
            raise

        # Set default internal parameters and constants
        self._EXT_CLK_FREQ = 40000000
        self._nDisplaySamples = 4000
        self._maxDisplayTime = 2
        self._nSamplesPerAllocation = 7200000  # acquiredData grows by this many samples (1 hour at 2 kHz)
        self._nAutoThreshSamples = 1000  # Samples that autoSetThreshold() measures
        self._autoThreshRangeMultiple = 5  # Range-widths that it puts the threshold below them

        self.info = {}
        self.info['measurementTime_us'] = 0
        self.acquiredData = {}
        self.gui = {}

        # Streaming. The reader thread logs to self.acquiredData, and the main thread plots from it:
        # both hold _dataLock while they use the data vectors or self.gui['acquiredDataPos']
        self._dataLock = threading.Lock()
        self._readerThread = None
        self._stopReader = False
        self._stopDeadline = 0
        self._autoThreshStartPos = None  # First sample of an AutoSet measurement in progress
        self._after_id = None

        # Set default user parameters. As in the MATLAB class, each assignment programs the device
        # (see the setters below). With the released firmware build, the four sensor settings each
        # put a ~10ms pulse on the device's lick outputs (fixed issue 1 in /AGENTS.md)
        self.rCount = 256
        self.settleCount = 10
        self.refDivider = 1
        self.driveCurrent = 31
        self.threshold = 18200000
        self.activeChannel = 1
        self.ledEnabled = False
        self.samplingRate = 2000

        self._initialized = True
        self.computeMeasurementTime()

    def _handshake(self):
        # A session that ended while streaming can have left the device sending samples (the
        # released firmware build goes on until it is told to stop). Bytes that arrive unasked are
        # that stream: stop it and discard them, or they would be read as the firmware version
        if self.Port.read(1):
            self.Port.write(struct.pack('BB', ord('S'), 0))
            self._discardInput()

        # Confirm firmware version
        self.Port.write(struct.pack('B', ord('F')))
        try:
            fv_data = self._read_exact(4)
        except TimeoutError:
            raise TimeoutError(f'No LickStick answered on {self.Port.port}') from None
        self._firmwareVersion = struct.unpack('<I', fv_data)[0]
        if self._firmwareVersion > self._currentFirmwareVersion:
            raise ValueError('Future firmware version detected. Please update LickStick.')
        elif self._firmwareVersion < self._currentFirmwareVersion:
            raise ValueError('Old firmware version detected. Please update firmware.')

    def _read_exact(self, size):
        # Reads size bytes from the device. Raises TimeoutError when none has arrived for
        # _readTimeout seconds (the clock restarts at each byte, so a long readSensor() is not cut off)
        data = bytearray()
        deadline = time.monotonic() + self._readTimeout
        while len(data) < size:
            chunk = self.Port.read(size - len(data))
            if chunk:
                data += chunk
                deadline = time.monotonic() + self._readTimeout
            elif time.monotonic() > deadline:
                raise TimeoutError(f'The device sent {len(data)} of the {size} bytes expected')
        return bytes(data)

    def _discardInput(self, maxTime=2):
        # Discards what the device sends, until it has sent nothing for 100 ms (the port's timeout).
        # Gives up after maxTime seconds, in case the port is not a LickStick and never stops
        deadline = time.monotonic() + maxTime
        while self.Port.read(4096) and time.monotonic() < deadline:
            pass

    @property
    def samplingRate(self):
        return self._samplingRate

    @samplingRate.setter
    def samplingRate(self, newRate):
        self.assertNotStreaming('samplingRate')
        if self._firmwareVersion > 1:
            if newRate < 500 or newRate > 2000:
                raise ValueError('The sampling rate must be between 500Hz and 2000Hz')
            self.Port.write(struct.pack('B', ord('I')) + struct.pack('<f', (1 / newRate) * 1000000))
        else:
            print('LickStick warning: firmware v1 has a fixed sampling rate. Sampling rate not changed.')
            newRate = 2000
        self._samplingRate = newRate
        self._nDisplaySamples = int(self.samplingRate * self._maxDisplayTime)

    @property
    def ledEnabled(self):
        return self._ledEnabled

    @ledEnabled.setter
    def ledEnabled(self, newState):
        self.assertNotStreaming('ledEnabled')
        self.Port.write(struct.pack('BB', ord('L'), int(newState)))
        self._ledEnabled = newState

    @property
    def activeChannel(self):
        return self._activeChannel

    @activeChannel.setter
    def activeChannel(self, newChannel):
        self.assertNotStreaming('activeChannel')
        # Channels are numbered 1 and 2, as labeled on the device, and sent as 0 and 1
        if newChannel not in (1, 2):
            raise ValueError('Active channel must be either 1 or 2')
        self.Port.write(struct.pack('BB', ord('!'), newChannel - 1))
        self._activeChannel = newChannel

    @property
    def rCount(self):
        return self._rCount

    @rCount.setter
    def rCount(self, newCount):
        self.assertNotStreaming('rCount')
        if newCount < 256 or newCount > 65535:
            raise ValueError('rCount must be in range [256 65535]')
        self.Port.write(struct.pack('<BH', ord('W'), newCount))
        self._rCount = newCount
        self.computeMeasurementTime()

    @property
    def settleCount(self):
        return self._settleCount

    @settleCount.setter
    def settleCount(self, newCount):
        self.assertNotStreaming('settleCount')
        if newCount < 2 or newCount > 65535:
            raise ValueError('settleCount must be in range [2 65535]')
        self.Port.write(struct.pack('<BH', ord('N'), newCount))
        self._settleCount = newCount
        self.computeMeasurementTime()

    @property
    def refDivider(self):
        return self._refDivider

    @refDivider.setter
    def refDivider(self, newDivider):
        self.assertNotStreaming('refDivider')
        if newDivider < 1 or newDivider > 255:
            raise ValueError('refDivider must be in range [1 255]')
        self.Port.write(struct.pack('BB', ord('D'), newDivider))
        self._refDivider = newDivider
        self.computeMeasurementTime()

    @property
    def driveCurrent(self):
        return self._driveCurrent

    @driveCurrent.setter
    def driveCurrent(self, newCurrent):
        self.assertNotStreaming('driveCurrent')
        if newCurrent < 0 or newCurrent > 31:
            raise ValueError('driveCurrent must be in range [0 31]')
        self.Port.write(struct.pack('BB', ord('C'), newCurrent))
        self._driveCurrent = newCurrent

    @property
    def threshold(self):
        return self._threshold

    @threshold.setter
    def threshold(self, newThreshold):
        # May be set during streaming: op 'T' does not disturb the sensor
        newThreshold = int(round(newThreshold))
        if newThreshold < 0 or newThreshold > 4294967295:
            raise ValueError('threshold must be in range [0 4294967295]')
        self.Port.write(struct.pack('<BI', ord('T'), newThreshold))
        self._threshold = newThreshold

    def readSensor(self, nSamples=1):
        self.assertNotStreaming('readSensor operation')
        self.Port.write(struct.pack('<BI', ord('R'), nSamples))
        data = self._read_exact(4 * nSamples)
        values = [struct.unpack('<I', data[i:i + 4])[0] for i in range(0, len(data), 4)]
        return values

    def autoSetThreshold(self):
        # Sets the threshold 5 range-widths (_autoThreshRangeMultiple) below the signal measured now.
        # Run when the subject is NOT licking
        self.assertNotStreaming('threshold')
        self.threshold = self._thresholdFromSamples(self.readSensor(self._nAutoThreshSamples))

    def _thresholdFromSamples(self, values):
        # The threshold that autoSetThreshold() and the GUI's AutoSet button derive from a
        # measurement of the resting signal. Not below 0, where the MATLAB class's uint32
        # arithmetic stops too
        vMax = int(max(values))
        vMin = int(min(values))
        vRange = vMax - vMin
        return max(0, vMin - (self._autoThreshRangeMultiple * vRange))

    def clearAcquiredData(self):
        # Clears the data in self.acquiredData. Intended to be used in a protocol, after
        # stream(block=False) and just before the main loop, to exclude the samples acquired during
        # manual threshold setup
        with self._dataLock:
            self.acquiredData['Sensor'] = np.zeros(self._nSamplesPerAllocation, dtype=np.uint32)
            self.acquiredData['TTL'] = np.zeros(self._nSamplesPerAllocation, dtype=np.uint8)
            self.acquiredData['nSamples'] = 0
            self.gui['acquiredDataPos'] = 1
            self.gui['plotPos'] = 1
            if self._autoThreshStartPos is not None:
                self._autoThreshStartPos = 1  # An AutoSet measurement in progress starts again

    def stream(self, block=True):
        # Launches the live streaming GUI and starts recording to self.acquiredData.
        # block=True (the default): runs the tkinter main loop, and returns when the window is closed.
        # block=False: returns at once, so that a protocol can run alongside. The reader thread goes
        # on recording whatever the protocol does, but the window is redrawn and its controls work
        # only while update() is called (or, at an interactive prompt, while Python waits for input).
        # End the recording with endAcq(), or by closing the window
        if self._guiOpen():
            self.gui['Fig'].lift()  # Already streaming: show the window
            return

        # Setup data structure
        self.acquiredData = {}
        self.acquiredData['nSamples'] = 0
        self.acquiredData['Sensor'] = np.zeros(self._nSamplesPerAllocation, dtype=np.uint32)
        self.acquiredData['TTL'] = np.zeros(self._nSamplesPerAllocation, dtype=np.uint8)
        self.acquiredData['Params'] = {}
        self.acquiredData['Params']['threshold'] = self.threshold
        self.acquiredData['Params']['samplingRate'] = self.samplingRate
        self.acquiredData['Params']['activeChannel'] = self.activeChannel
        self.acquiredData['Params']['rCount'] = self.rCount
        self.acquiredData['Params']['settleCount'] = self.settleCount
        self.acquiredData['Params']['refDivider'] = self.refDivider
        self.acquiredData['Params']['driveCurrent'] = self.driveCurrent
        self.acquiredData['Info'] = {}
        dateInfo = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.acquiredData['Info']['dateTime'] = dateInfo
        self.acquiredData['Info']['measurementTime_us'] = self.info['measurementTime_us']
        self.acquiredData['Info']['pcArchitecture'] = platform.machine()
        self.acquiredData['Info']['pcOS'] = platform.platform()
        self.acquiredData['Info']['matlabVersion'] = f'Python {sys.version.split()[0]}'  # Adapted

        # Setup GUI
        self.gui = {}
        self.gui['DisplayIntensities'] = np.full(self._nDisplaySamples, np.nan)
        self.gui['DisplayTTL'] = np.full(self._nDisplaySamples, np.nan)
        self.gui['DisplayTimes'] = np.full(self._nDisplaySamples, np.nan)

        root = tk.Tk()
        root.title('LickStick')
        root.geometry('1280x400+100+400')
        root.protocol('WM_DELETE_WINDOW', self.endAcq)
        root.configure(bg='white')
        try:
            # Replace the feather icon with the blank one beside this file
            root.iconbitmap(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'blank.ico'))
        except tk.TclError:
            pass  # The file is missing, or the platform takes no .ico: keep tkinter's icon
        self.gui['Fig'] = root

        fig = Figure(figsize=(12.8, 4))
        ax = fig.add_subplot(111)
        ax.set_ylabel('Sensor value (bits)', fontsize=18)
        ax.set_xlabel('Time (s)', fontsize=18)
        ax.set_xlim([0, self._maxDisplayTime])
        self.gui['Plot'] = ax

        # Adjust subplot parameters to make room for labels
        fig.subplots_adjust(left=0.1, right=0.95, top=0.9, bottom=0.2)  # Adjusted values

        canvas = FigureCanvasTkAgg(fig, master=root)
        canvas.draw()
        canvas.get_tk_widget().place(relx=0.07, rely=0.15, relwidth=0.9, relheight=0.75)
        self.gui['canvas'] = canvas

        self.gui['thresholdLabel'] = tk.Label(root, text='Thresh:', font=('Arial', 12), bg='white')
        self.gui['thresholdLabel'].place(x=5, y=15, width=70, height=20)

        self.gui['thresholdVar'] = tk.StringVar(value=str(self.threshold))
        self.gui['thresholdSet'] = tk.Entry(root, font=('Arial', 10, 'bold'), bg='white',
                                            textvariable=self.gui['thresholdVar'])
        self.gui['thresholdSet'].place(x=70, y=10, width=80, height=30)
        self.gui['thresholdSet'].bind('<Return>', lambda e: self.uiSetThreshold())

        self.gui['threshPlusBtn'] = tk.Button(root, text='+', bg='white', font=('Arial', 12),
                                              command=lambda: self.uiAdjustThreshold('+'))
        self.gui['threshPlusBtn'].place(x=155, y=26, width=25, height=13)

        self.gui['threshMinusBtn'] = tk.Button(root, text='-', bg='white', font=('Arial', 12),
                                               command=lambda: self.uiAdjustThreshold('-'))
        self.gui['threshMinusBtn'].place(x=155, y=10, width=25, height=13)

        self.gui['autoThreshButton'] = tk.Button(root, text='AutoSet', bg='white', font=('Arial', 12),
                                                 command=self.uiAutoSetThreshold)
        self.gui['autoThreshButton'].place(x=185, y=10, width=80, height=30)

        self.gui['tMaxLabel'] = tk.Label(root, text='tMax (s):', font=('Arial', 12), bg='white')
        self.gui['tMaxLabel'].place(x=1015, y=15, width=100, height=20)

        self.gui['tMaxVar'] = tk.StringVar(value=str(self._maxDisplayTime))
        self.gui['tMaxSet'] = tk.Entry(root, font=('Arial', 10, 'bold'), bg='white', textvariable=self.gui['tMaxVar'])
        self.gui['tMaxSet'].place(x=1100, y=10, width=70, height=30)
        self.gui['tMaxSet'].bind('<Return>', lambda e: self.uiSetTmax())

        self.gui['resetRangeButton'] = tk.Button(root, text='RngReset', bg='white', font=('Arial', 12),
                                                 command=self.UIresetRange)
        self.gui['resetRangeButton'].place(x=1175, y=10, width=100, height=30)

        self.gui['startStopButton'] = tk.Button(root, text='Stop', bg='white', font=('Arial', 12), command=self.uiStartStop)
        self.gui['startStopButton'].place(x=1175, y=365, width=100, height=30)

        self.gui['OscopeDataLine'], = ax.plot(self.gui['DisplayTimes'], self.gui['DisplayIntensities'], linewidth=1.5)
        self.gui['OscopeTTLLine'], = ax.plot(self.gui['DisplayTimes'], self.gui['DisplayTTL'], color='black',
                                             linewidth=1.5)
        self.gui['OscopeThreshLine'], = ax.plot([0, self._maxDisplayTime], [self.threshold, self.threshold], color='red',
                                                linestyle='--')

        # Setup GUI variables
        self.gui['DisplayPos'] = 1
        self.gui['SweepStartTime'] = 0
        self.gui['acquiredDataPos'] = 1  # Where the next sample goes in acquiredData (counted from 1)
        self.gui['plotPos'] = 1  # The next sample of acquiredData to plot (counted from 1)
        self.gui['FirstSample'] = 1
        self.gui['resetRangeFlag'] = False
        self.gui['Ymax'] = np.nan
        self.gui['Ymin'] = np.nan
        self.gui['y_inited'] = False
        self._autoThreshStartPos = None

        self.startAcq()
        # Added lines to make the window visible and focused
        root.update_idletasks()  # Ensures the window is fully created before trying to deiconify
        root.deiconify()  # Restores the window if it was minimized
        root.lift()  # Brings the window to the front
        root.attributes('-topmost', True)  # Makes the window topmost
        root.attributes('-topmost', False)  # Then allows other windows to go on top (optional, good practice)

        if block:
            root.mainloop()

    def update(self):
        # After stream(block=False): redraws the streaming window and runs the callbacks of its
        # controls. Call it regularly from the protocol's loop, as drawnow is called in MATLAB.
        # Recording does not depend on it. Does nothing once the window is closed
        if self._guiOpen():
            self.gui['Fig'].update()

    def __del__(self):
        if hasattr(self, 'Port') and self.Port.is_open:
            self.Port.close()

    def _guiOpen(self):
        return self.gui.get('Fig') is not None

    def _readStream(self):
        # Runs in the reader thread from startAcq() to stopAcq(): reads the samples as they arrive and
        # logs them. It is the only reader of the port meanwhile, and it touches nothing of the GUI,
        # which belongs to the main thread
        pending = b''
        while True:
            try:
                nWaiting = self.Port.in_waiting
                chunk = self.Port.read(nWaiting if nWaiting > 0 else 1)  # Waits 100 ms at most
            except (serial.SerialException, OSError) as e:
                print(f'LickStick: the stream ended, because the port could not be read ({e})')
                break
            if chunk:
                pending += chunk
                nBytes = (len(pending) // 4) * 4
                if nBytes > 0:
                    self._logSamples(np.frombuffer(pending[:nBytes], dtype='<u4'))
                    pending = pending[nBytes:]
            # After stopAcq() has told the device to stop, read on until it is quiet, so that the
            # recording has every sample that was sent
            if self._stopReader and (not chunk or time.monotonic() > self._stopDeadline):
                break

    def _logSamples(self, words):
        # Appends samples to self.acquiredData. Each sample is a uint32 whose lowest bit is the lick
        # detection state, and whose other bits are the sensor value (see 'Sample word' in
        # /Firmware/PROTOCOL.md). The samples carry no timestamp: a sample's time is its position in
        # the stream / samplingRate
        nNew = len(words)
        with self._dataLock:
            first = self.gui['acquiredDataPos'] - 1
            last = first + nNew
            if last > len(self.acquiredData['Sensor']):
                nMore = max(self._nSamplesPerAllocation, nNew)
                self.acquiredData['Sensor'] = np.concatenate(
                    (self.acquiredData['Sensor'], np.zeros(nMore, dtype=np.uint32)))
                self.acquiredData['TTL'] = np.concatenate(
                    (self.acquiredData['TTL'], np.zeros(nMore, dtype=np.uint8)))
            self.acquiredData['Sensor'][first:last] = words & np.uint32(0xFFFFFFFE)
            self.acquiredData['TTL'][first:last] = words & np.uint32(1)
            self.gui['acquiredDataPos'] = last + 1
            self.acquiredData['nSamples'] = last

    def _plotLoop(self):
        # Called by tkinter every 10 ms during streaming
        self._after_id = None
        if self._streaming and self._guiOpen():
            self.updatePlot()
        # The window can have been closed, or the stream stopped, during updatePlot()
        if self._streaming and self._guiOpen():
            self._after_id = self.gui['Fig'].after(10, self._plotLoop)

    def updatePlot(self):
        # Plots the samples that the reader thread has logged to self.acquiredData since the last
        # call, and finishes an AutoSet measurement once its samples are in
        autoThreshValues = None
        with self._dataLock:
            firstPos = self.gui['plotPos']
            lastPos = self.gui['acquiredDataPos']
            NewIntensities = self.acquiredData['Sensor'][firstPos - 1:lastPos - 1].copy()
            LickDetected = self.acquiredData['TTL'][firstPos - 1:lastPos - 1].copy()
            if (self._autoThreshStartPos is not None
                    and lastPos - self._autoThreshStartPos >= self._nAutoThreshSamples):
                start = self._autoThreshStartPos - 1
                autoThreshValues = self.acquiredData['Sensor'][start:start + self._nAutoThreshSamples].copy()
                self._autoThreshStartPos = None
        self.gui['plotPos'] = lastPos
        if autoThreshValues is not None:
            self.threshold = self._thresholdFromSamples(autoThreshValues)
            self._showThreshold()

        nIntensities = len(NewIntensities)
        if nIntensities > 0:
            # One-time y-limit init so we can see data immediately
            if not self.gui.get('y_inited', False):
                y_max = float(np.max(NewIntensities))
                y_min = float(np.min(NewIntensities))
                if y_max == y_min:
                    y_max += 1.0
                pad = max(1.0, 0.005 * (y_max - y_min))  # 0.5% headroom
                self.gui['Ymax'] = y_max
                self.gui['Ymin'] = y_min
                self.gui['Plot'].set_ylim([y_min - pad, y_max + pad])
                self.gui['y_inited'] = True

            # TTL line mapped after Ymin/Ymax exist
            NewDisplayTTL = (LickDetected.astype(float) * (self.gui['Ymax'] - self.gui['Ymin'])) + self.gui['Ymin']

            Div = self.samplingRate
            Times = np.arange(self.gui['DisplayPos'], self.gui['DisplayPos'] + nIntensities) / Div
            DisplayTime = Times[-1] - self.gui['SweepStartTime'] if len(Times) > 0 else 0
            self.gui['DisplayPos'] += nIntensities
            if DisplayTime >= self._maxDisplayTime:
                # The sweep is full. (It is empty if more than a sweep of samples came at once)
                if not np.all(np.isnan(self.gui['DisplayIntensities'])):
                    if self.gui['FirstSample'] or self.gui['resetRangeFlag']:
                        self.gui['Ymax'] = np.nanmax(self.gui['DisplayIntensities'])
                        self.gui['Ymin'] = np.nanmin(self.gui['DisplayIntensities'])
                        self.gui['FirstSample'] = 0
                        self.gui['resetRangeFlag'] = False
                    if np.nanmax(self.gui['DisplayIntensities']) > self.gui['Ymax']:
                        self.gui['Ymax'] = np.nanmax(self.gui['DisplayIntensities'])
                        self.gui['Plot'].set_ylim([self.gui['Ymin'] - (self.gui['Ymin'] * 0.0005),
                                                   self.gui['Ymax'] + (self.gui['Ymax'] * 0.0005)])
                    if np.nanmin(self.gui['DisplayIntensities']) < self.gui['Ymin']:
                        self.gui['Ymin'] = np.nanmin(self.gui['DisplayIntensities'])
                        self.gui['Plot'].set_ylim([self.gui['Ymin'] - (self.gui['Ymin'] * 0.0005),
                                                   self.gui['Ymax'] + (self.gui['Ymax'] * 0.0005)])
                self.resetSweep()
            else:
                SweepTimes = Times - self.gui['SweepStartTime']
                self.gui['DisplayIntensities'][
                self.gui['DisplayPos'] - nIntensities - 1: self.gui['DisplayPos'] - 1] = NewIntensities
                self.gui['DisplayTTL'][
                self.gui['DisplayPos'] - nIntensities - 1: self.gui['DisplayPos'] - 1] = NewDisplayTTL
                self.gui['DisplayTimes'][
                self.gui['DisplayPos'] - nIntensities - 1: self.gui['DisplayPos'] - 1] = SweepTimes

            self.gui['OscopeDataLine'].set_data(self.gui['DisplayTimes'], self.gui['DisplayIntensities'])
            self.gui['OscopeTTLLine'].set_data(self.gui['DisplayTimes'], self.gui['DisplayTTL'])
            self.gui['canvas'].draw()
            self.gui['canvas'].flush_events()

    def resetSweep(self):
        self.gui['DisplayIntensities'] = np.full(self._nDisplaySamples, np.nan)
        self.gui['DisplayTTL'] = np.full(self._nDisplaySamples, np.nan)
        self.gui['DisplayTimes'] = np.full(self._nDisplaySamples, np.nan)
        self.gui['DisplayPos'] = 1
        self.gui['SweepStartTime'] = 0

    def _showThreshold(self):
        # Shows self.threshold in the GUI's box and as the red line, and notes it in acquiredData
        self.acquiredData['Params']['threshold'] = self.threshold
        self.gui['thresholdVar'].set(str(self.threshold))
        self.gui['OscopeThreshLine'].set_ydata([self.threshold, self.threshold])
        self.gui['canvas'].draw()
        self.gui['canvas'].flush_events()

    def uiSetThreshold(self):
        # Callback of the Thresh box. Text that is not a threshold is replaced by the one in force
        try:
            self.threshold = float(self.gui['thresholdVar'].get())
        except (ValueError, OverflowError):
            pass
        self._showThreshold()

    def uiSetTmax(self):
        newTmax = float(self.gui['tMaxVar'].get())
        self._maxDisplayTime = newTmax
        self._nDisplaySamples = int(self.samplingRate * self._maxDisplayTime)
        self.gui['DisplayIntensities'] = np.full(self._nDisplaySamples, np.nan)
        self.gui['DisplayTTL'] = np.full(self._nDisplaySamples, np.nan)
        self.gui['DisplayTimes'] = np.full(self._nDisplaySamples, np.nan)
        self.gui['Plot'].set_xlim([0, self._maxDisplayTime])
        self.gui['OscopeThreshLine'].set_xdata([0, self._maxDisplayTime])
        self.resetSweep()
        self.gui['canvas'].draw()
        self.gui['canvas'].flush_events()

    def UIresetRange(self):
        self.gui['resetRangeFlag'] = True

    def uiAutoSetThreshold(self):
        # Callback of the AutoSet button. While streaming, the measurement is the next
        # _nAutoThreshSamples samples of the stream (updatePlot() sets the threshold when they are
        # in), so the recording is not paused. Run when the subject is NOT licking
        if self._streaming:
            with self._dataLock:
                self._autoThreshStartPos = self.gui['acquiredDataPos']
        else:
            self.autoSetThreshold()
            self._showThreshold()

    def uiAdjustThreshold(self, op):
        # Callback of the + and - buttons: moves the threshold by one increment. The stream goes on
        increment = 2000
        thresh = round(self.threshold / increment) * increment
        if op == '+':
            thresh = thresh + increment
        elif op == '-':
            thresh = thresh - increment
        self.threshold = min(max(thresh, 0), 4294967295)
        self._showThreshold()

    def uiStartStop(self):
        if self._streaming:
            self.stopAcq()
            self.gui['startStopButton']['text'] = 'Start'
        else:
            self.startAcq()
            self.gui['startStopButton']['text'] = 'Stop'

    def startAcq(self):
        self._streaming = True
        self._stopReader = False
        self.Port.write(struct.pack('BB', ord('S'), 1))
        self._readerThread = threading.Thread(target=self._readStream, name='LickStick reader', daemon=True)
        self._readerThread.start()
        # Store the ID returned by after
        self._after_id = self.gui['Fig'].after(10, self._plotLoop)

    def stopAcq(self):
        # Stops the stream. The reader thread logs the samples still on their way, and ends when the
        # device has been quiet for 100 ms (or after 2 s, if it never is)
        self._streaming = False
        self._autoThreshStartPos = None
        try:
            self.Port.write(struct.pack('BB', ord('S'), 0))
        except serial.SerialException:
            pass  # The device is gone. The reader thread has ended, and said so
        if self._readerThread is not None:
            self._stopDeadline = time.monotonic() + 2
            self._stopReader = True
            self._readerThread.join()
            self._readerThread = None
        # Cancel the pending after call if it exists
        if self._after_id:
            self.gui['Fig'].after_cancel(self._after_id)
            self._after_id = None

    def computeMeasurementTime(self):
        if self._initialized:
            SettleTime = (self.settleCount * 16) / (self._EXT_CLK_FREQ / self.refDivider)
            ConversionTime = ((self.rCount * 16) + 4) / (self._EXT_CLK_FREQ / self.refDivider)
            self.info['measurementTime_us'] = (SettleTime + ConversionTime) * 1000000

    def assertNotStreaming(self, paramName):
        if self._streaming:
            raise ValueError(
                f'Error: {paramName} cannot be set during USB data streaming. Close the streaming GUI first.')

    def endAcq(self):
        # Ends the recording and closes the streaming window (also the window's close button)
        if not self._guiOpen():
            return
        if self._streaming:
            self.stopAcq()
        # Cancel the pending after call before destroying the window
        if self._after_id:
            self.gui['Fig'].after_cancel(self._after_id)
            self._after_id = None
        root = self.gui['Fig']
        self.gui['Fig'] = None
        root.destroy()
        # Trim the preallocated data vectors to the samples received
        nSamples = self.gui['acquiredDataPos'] - 1
        self.acquiredData['Sensor'] = self.acquiredData['Sensor'][:nSamples].copy()
        self.acquiredData['TTL'] = self.acquiredData['TTL'][:nSamples].copy()
        self.acquiredData['nSamples'] = nSamples
