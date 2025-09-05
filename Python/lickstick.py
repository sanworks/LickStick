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

import serial
import struct
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
        self._currentFirmwareVersion = 2  # Assumed current version
        self.Port = serial.Serial(portName, 480000000, timeout=0)

        # Confirm firmware version
        self.Port.write(struct.pack('B', ord('F')))
        fv_data = self._read_exact(4)
        if len(fv_data) < 4:
            raise ValueError('No response from device')
        self._firmwareVersion = struct.unpack('<I', fv_data)[0]
        if self._firmwareVersion > self._currentFirmwareVersion:
            self.Port.close()
            raise ValueError('Future firmware version detected. Please update LickStick.')
        elif self._firmwareVersion < self._currentFirmwareVersion:
            self.Port.close()
            raise ValueError('Old firmware version detected. Please update firmware.')

        # Set default user parameters
        self._rCount = 256
        self._settleCount = 10
        self._refDivider = 1
        self._driveCurrent = 31
        self._threshold = 18200000
        self._activeChannel = 1
        self._ledEnabled = False
        
        # Set default internal parameters and constants
        self._EXT_CLK_FREQ = 40000000
        self._samplingRate = 2000
        self._streaming = False
        self._nDisplaySamples = 4000
        self._maxDisplayTime = 2

        self.info = {}
        self.info['measurementTime_us'] = 0

        self.sampleDataTemplate = np.zeros(36000000, dtype=np.uint32)
        self._initialized = True
        self.computeMeasurementTime()

        self._after_id = None # Initialize _after_id

    def _read_exact(self, size):
        data = b''
        while len(data) < size:
            chunk = self.Port.read(size - len(data))
            if chunk:
                data += chunk
            if hasattr(self, 'gui') and 'Fig' in self.gui:
                self.gui['Fig'].update_idletasks()
            #time.sleep(0.01)
        return data

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
        self._nDisplaySamples = self.samplingRate * self._maxDisplayTime

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
        self.Port.write(struct.pack('<BI', ord('T'), newThreshold))
        self._threshold = newThreshold

    def readSensor(self, nSamples=1):
        self.assertNotStreaming('readSensor operation')
        self.Port.write(struct.pack('<BI', ord('R'), nSamples))
        data = self._read_exact(4 * nSamples)
        values = [struct.unpack('<I', data[i:i + 4])[0] for i in range(0, len(data), 4)]
        return values

    def autoSetThreshold(self):
        self.assertNotStreaming('threshold')
        nSamplesToMeasure = 1000
        rangeMultiple = 5
        self.Port.write(struct.pack('<BI', ord('R'), nSamplesToMeasure))
        data = self._read_exact(4 * nSamplesToMeasure)
        Values = [struct.unpack('<I', data[i:i + 4])[0] for i in range(0, len(data), 4)]
        vMax = max(Values)
        vMin = min(Values)
        vRange = vMax - vMin
        newThreshold = vMin - (rangeMultiple * vRange)
        self.threshold = newThreshold

    def clearAcquiredData(self):
        self.acquiredData['Sensor'] = self.sampleDataTemplate.copy()
        self.acquiredData['TTL'] = np.zeros(36000000, dtype=np.uint8)
        self.gui['acquiredDataPos'] = 1

    def stream(self):
        # Setup data structure
        self.acquiredData = {}
        self.acquiredData['nSamples'] = 0
        self.acquiredData['Sensor'] = self.sampleDataTemplate.copy()
        self.acquiredData['TTL'] = np.zeros(36000000, dtype=np.uint8)
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
        root.iconbitmap("blank.ico") # Remove the feather icon
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
        self.gui['acquiredDataPos'] = 1
        self.gui['FirstSample'] = 1
        self.gui['resetRangeFlag'] = False
        self.gui['Ymax'] = np.nan
        self.gui['Ymin'] = np.nan
        self.gui['y_inited'] = False

        self.startAcq()
        # Added lines to make the window visible and focused
        root.update_idletasks()  # Ensures the window is fully created before trying to deiconify
        root.deiconify()  # Restores the window if it was minimized
        root.lift()  # Brings the window to the front
        root.attributes('-topmost', True)  # Makes the window topmost
        root.attributes('-topmost', False)  # Then allows other windows to go on top (optional, good practice)

        root.mainloop()

    def __del__(self):
        if hasattr(self, 'Port') and self.Port.is_open:
            self.Port.close()

    def updatePlot(self):
        if not self._streaming:
            # If streaming is stopped, cancel any pending after calls and return
            if self._after_id:
                self.gui['Fig'].after_cancel(self._after_id)
                self._after_id = None
            return

        BytesAvailable = self.Port.in_waiting
        if BytesAvailable > 1:
            nBytesToRead = (BytesAvailable // 4) * 4
            data = self.Port.read(nBytesToRead)
            nIntensities = nBytesToRead // 4
            lickDetectedBytes = [data[i] for i in range(0, nBytesToRead, 4)]
            LickDetected = [byte & 1 for byte in lickDetectedBytes]
            data = bytearray(data)
            for j, i in enumerate(range(0, nBytesToRead, 4)):
                data[i] = lickDetectedBytes[j] & ~1
            NewIntensities = struct.unpack('<' + 'I' * nIntensities, data)
            NewIntensities = np.array(NewIntensities, dtype=np.uint32)

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
            NewDisplayTTL = np.array(
                [(ld * (self.gui['Ymax'] - self.gui['Ymin'])) + self.gui['Ymin'] for ld in LickDetected],
                dtype=float
            )

            self.acquiredData['Sensor'][
            self.gui['acquiredDataPos'] - 1:self.gui['acquiredDataPos'] + nIntensities - 1] = NewIntensities
            self.acquiredData['TTL'][
            self.gui['acquiredDataPos'] - 1:self.gui['acquiredDataPos'] + nIntensities - 1] = LickDetected
            self.gui['acquiredDataPos'] += nIntensities

            Div = self.samplingRate
            Times = np.arange(self.gui['DisplayPos'], self.gui['DisplayPos'] + nIntensities) / Div
            DisplayTime = Times[-1] - self.gui['SweepStartTime'] if len(Times) > 0 else 0
            self.gui['DisplayPos'] += nIntensities
            if DisplayTime >= self._maxDisplayTime:
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

        # Only schedule the next update if streaming is still active
        if self._streaming:
            self._after_id = self.gui['Fig'].after(10, self.updatePlot)


    def resetSweep(self):
        self.gui['DisplayIntensities'] = np.full(self._nDisplaySamples, np.nan)
        self.gui['DisplayTTL'] = np.full(self._nDisplaySamples, np.nan)
        self.gui['DisplayTimes'] = np.full(self._nDisplaySamples, np.nan)
        self.gui['DisplayPos'] = 1
        self.gui['SweepStartTime'] = 0

    def uiSetThreshold(self):
        newThreshold = float(self.gui['thresholdVar'].get())
        self.threshold = int(newThreshold)
        self.acquiredData['Params']['threshold'] = self.threshold
        self.gui['OscopeThreshLine'].set_ydata([newThreshold, newThreshold])
        self.gui['canvas'].draw()
        self.gui['canvas'].flush_events()

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
        wasStreaming = False
        if self._streaming:
            wasStreaming = True
            self.stopAcq()
        self.autoSetThreshold()
        self.gui['thresholdVar'].set(str(self.threshold))
        self.uiSetThreshold()
        if wasStreaming:
            self.startAcq()

    def uiAdjustThreshold(self, op):
        wasStreaming = False
        if self._streaming:
            wasStreaming = True
            self.stopAcq()
        increment = 2000
        thresh = round(self.threshold / increment) * increment
        if op == '+':
            self.threshold = thresh + increment
        elif op == '-':
            self.threshold = thresh - increment
        self.gui['thresholdVar'].set(str(self.threshold))
        self.uiSetThreshold()
        if wasStreaming:
            self.startAcq()

    def uiStartStop(self):
        if self._streaming:
            self.stopAcq()
            self._streaming = False
            self.gui['startStopButton']['text'] = 'Start'
        else:
            self.startAcq()
            self._streaming = True
            self.gui['startStopButton']['text'] = 'Stop'

    def startAcq(self):
        self._streaming = True
        self.Port.write(struct.pack('BB', ord('S'), 1))
        # Store the ID returned by after
        self._after_id = self.gui['Fig'].after(10, self.updatePlot)

    def stopAcq(self):
        self._streaming = False
        self.Port.write(struct.pack('BB', ord('S'), 0))
        self.Port.reset_input_buffer()
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
        if self._streaming:
            self.stopAcq()
        # Cancel the pending after call before destroying the window
        if self._after_id:
            self.gui['Fig'].after_cancel(self._after_id)
            self._after_id = None
        self.gui['Fig'].destroy()
        nSamples = self.gui['acquiredDataPos'] - 1
        self.acquiredData['Sensor'] = self.acquiredData['Sensor'][:nSamples]
        self.acquiredData['TTL'] = self.acquiredData['TTL'][:nSamples]
        self.acquiredData['nSamples'] = nSamples