%{
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
%}

% LickStick class to interface with the LickStick device, a capacitive lick
% detector powered by the Texas Instruments FDC2214-Q1 sensor IC.
%
% Usage:
% L = LickStick('COM3');       % Where COM3 is the device's USB serial port
% L.field = value;             % Update the device params by setting class fields
% L.autoSetThreshold;          % Automatically set the lick detection threshold
% values = L.readSensor;       % Read the current sensor value (units = bits)
% L.stream;                    % Launch a streaming GUI
% data = L.acquiredData;       % Stored data from last streaming GUI session
% clear L                      % Clear L when finished, releasing the serial port

classdef LickStick < handle
    properties
        Port                   % USB Serial port (Wrapped with ArCOM class)
        activeChannel          % The active channel index (0 or 1)
        rCount                 % Number of crystal clock counts comprising duration to measure capacitance.
                               % Higher rCount = better detection, but adds latency
        settleCount            % Number of clock counts to pause before measuring
        refDivider             % Factor to divide the crystal clock base frequency (40MHz) used for counts
        driveCurrent           % Current driving the sensor. Units = steps. Range = [0, 31] for [16µA, 1571µA]
                               % More current = better detection, higher odds of interference to other systems
        threshold              % Lick detection threshold. Units = bits. Range = [0, 4294967295]
        samplingRate           % Rate at which the FDC2214-Q1 is polled for new measurements
        ledEnabled             % If enabled, board LED remains on during detected licks
        
        acquiredData           % Data acquired while running the streaming GUI
    end

    properties (SetAccess = protected)
        info                   % Struct with information about the system
    end

    properties (SetAccess = immutable)
        firmwareVersion        % Firmware version returned from device
    end

    properties (Access = private)
        currentFirmwareVersion  % Current firmware version expected from device
        streamTimer             % MATLAB timer (to poll serial buffer during USB streaming)
        initialized = false;    % True after the constructor finishes executing
        streaming = false;      % Flag to indicate when streaming data via USB
        gui = struct;           % Struct of handles for GUI elements
        nDisplaySamples = 4000; % #samples to show when streaming to live plot
        maxDisplayTime = 2;     % When streaming to plot, show up to last 2 seconds
        extClkFreq = 40000000;  % Fixed frequency of crystal clock on the device (Hz) 
        sampleDataTemplate;     % Template for preallocated sample data
    end

    methods
        function obj = LickStick(portName)
            % Constructor function to set up an instance of LickStick
            % Args: portName (the USB serial port name, e.g. 'COM3')

            % Clear orphaned timers from previous instances
            obj.clearTimers(portName);

            % Setup USB serial port
            obj.Port = ArCOM_LickStick(portName, 480000000);
            
            % Confirm firmware version
            obj.Port.write('F', 'uint8');
            fv = obj.Port.read(1, 'uint32');
            if (fv > obj.currentFirmwareVersion)
                obj.Port = [];
                error('Future firmware version detected. Please update LickStick.')
            elseif (fv < obj.currentFirmwareVersion)
                obj.Port = [];
                error('Old firmware version detected. Please update firmware.')
            end
            obj.firmwareVersion = fv;

            % Set default parameters
            obj.rCount = 256;
            obj.settleCount = 10;
            obj.refDivider = 1;
            obj.driveCurrent = 31;
            obj.threshold = 18200000;
            obj.activeChannel = 0;
            obj.ledEnabled = false;
            obj.samplingRate = 2000; % Frequency at which FDC2214-Q1 is polled for new measurements (Hz)

            % Populate system info  
            obj.info.measurementTime_us = 0; % Actual measurement time in µs, computed from rCount, settleCount, 
                                             % refDivider and extClkFreq. 

            % Preallocate
            obj.sampleDataTemplate = uint32(zeros(1,36000000)); % Preallocate 5 hours of samples

            % Finish setup
            obj.initialized = true;
            obj.computeMeasurementTime;
        end

        function set.samplingRate(obj, newRate)
            % Set sampling rate
            % Args: newRate, the new sampling rate in Hz
            obj.assertNotStreaming('samplingRate');
            if obj.firmwareVersion > 1
                if (newRate < 500) || (newRate > 2000)
                    error('The sampling rate must be between 500Hz and 2000Hz');
                end
                obj.Port.write(['I' typecast(single((1/newRate)*1000000), 'uint8')]);
            else
                warning('LickStick warning: firmware v1 has a fixed sampling rate. Sampling rate not changed.')
                newRate = 2000;
            end
            obj.samplingRate = newRate;
            obj.nDisplaySamples = obj.samplingRate*obj.maxDisplayTime;
        end

        function set.ledEnabled(obj, newState)
            % Callback function triggered when ledEnabled is set.
            % Args: newState, the new state of LED enbale (false or true)
            obj.assertNotStreaming('ledEnabled');
            obj.Port.write(['L' uint8(newState)], 'uint8');
            obj.ledEnabled = newState;
        end

        function set.activeChannel(obj, newChannel)
            % Callback function triggered when activeChannel is set.
            % Args: newChannel, the new active channel index (0 or 1)
            obj.assertNotStreaming('activeChannel');
            if ~(newChannel == 0 || newChannel == 1)
                error ('Error: Active channel must be either 0 or 1')
            end
            obj.Port.write(['!' newChannel], 'uint8');
            obj.activeChannel = newChannel;
        end

        function set.rCount(obj, newCount)
            % Callback function triggered when rCount is set.
            % Args: newCount, the new value of rCount (units = clock pulses)
            obj.assertNotStreaming('rCount');
            if newCount < 256 || newCount > 65535
                error ('Error: rCount must be in range [256 65535]')
            end
            obj.Port.write('W', 'uint8', newCount, 'uint16');
            obj.rCount = newCount;
            obj.computeMeasurementTime;
        end

        function set.settleCount(obj, newCount)
            % Callback function triggered when settleCount is set.
            % Args: newCount, the new value of settleCount (units = clock pulses)
            obj.assertNotStreaming('settleCount');
            if newCount < 2 || newCount > 65535
                error ('Error: settleCount must be in range [2 65535]')
            end
            obj.Port.write('N', 'uint8', newCount, 'uint16');
            obj.settleCount = newCount;
            obj.computeMeasurementTime;
        end

        function set.refDivider(obj, newDivider)
            % Callback function triggered when refDivider is set.
            % Args: newDivider, the new value of refDivider (units = factor)
            obj.assertNotStreaming('refDivider');
            if newDivider < 1 || newDivider > 255
                error ('Error: refDivider must be in range [1 255]')
            end
            obj.Port.write(['D' newDivider], 'uint8');
            obj.refDivider = newDivider;
            obj.computeMeasurementTime;
        end

        function set.driveCurrent(obj, newCurrent)
            % Callback function triggered when driveCurrent is set.
            % Args: newCurrent, the new value of driveCurrent 
            %       units = increments, range = [0, 31]
            obj.assertNotStreaming('driveCurrent');
            if newCurrent < 0 || newCurrent > 31
                error ('Error: driveCurrent must be in range [0 31]')
            end
            obj.Port.write(['C' newCurrent], 'uint8');
            obj.driveCurrent = newCurrent;
        end

        function set.threshold(obj, newThreshold)
            % Callback function triggered when threshold is set.
            % Args: newThreshold, the new value of threshold 
            %       units = bits, range = [0, 4294967295]
            obj.Port.write('T', 'uint8', newThreshold, 'uint32');
            obj.threshold = newThreshold;
        end

        function values = readSensor(obj, varargin)
            % Reads the current value of the sensor
            % Optional argument = nSamples (contiguous)
            % Returns: values, the sensor reading(s) in bits (uint32)
            obj.assertNotStreaming('readSensor operation');
            nSamples = 1;
            if nargin > 1
                nSamples = varargin{1};
            end
            obj.Port.write('R', 'uint8', nSamples, 'uint32');
            values = obj.Port.read(nSamples, 'uint32');
        end

        function autoSetThreshold(obj)
            % Automatically sets the threshold.
            % IMPORTANT: Run when the subject is NOT licking. For human
            % testing, run while finger is 3mm from drink tube end.
            % The range of the signal is measured in bits, and the threshold is
            % set equal to 3 range-widths below the measured range.
            obj.assertNotStreaming('threshold');
            nSamplesToMeasure = 1000;
            rangeMultiple = 5;
            obj.Port.write('R', 'uint8', nSamplesToMeasure, 'uint32'); % Request 10k samples
            Values = obj.Port.read(nSamplesToMeasure, 'uint32');
            vMax = max(Values); vMin = min(Values); vRange = vMax-vMin;
            newThreshold = vMin-(rangeMultiple*vRange);
            obj.threshold = newThreshold;
        end

        function clearAcquiredData(obj)
            % Clears acquired data in obj.acquiredData. Intended to be used in protocol
            % file just before main loop to exclude samples acquired during manual threshold setup
            obj.acquiredData.Sensor = obj.sampleDataTemplate;
            obj.acquiredData.TTL = uint8(obj.sampleDataTemplate);
            obj.gui.acquiredDataPos = 1;
        end

        function stream(obj)
            % Launch the live streaming GUI

            % Setup data structure
            obj.acquiredData = struct;
            obj.acquiredData.nSamples = 0;
            obj.acquiredData.Sensor = obj.sampleDataTemplate;
            obj.acquiredData.TTL = uint8(obj.sampleDataTemplate);
            obj.acquiredData.Params = struct; 
            obj.acquiredData.Params.threshold = obj.threshold;
            obj.acquiredData.Params.samplingRate = obj.samplingRate;
            obj.acquiredData.Params.activeChannel = obj.activeChannel;
            obj.acquiredData.Params.rCount = obj.rCount;
            obj.acquiredData.Params.settleCount = obj.settleCount;
            obj.acquiredData.Params.refDivider = obj.refDivider;
            obj.acquiredData.Params.driveCurrent = obj.driveCurrent;
            obj.acquiredData.Info = struct;
            dateInfo = datestr(now, 30);
            dateInfo(dateInfo == 'T') = '_';
            obj.acquiredData.Info.dateTime = dateInfo;
            obj.acquiredData.Info.measurementTime_us = obj.info.measurementTime_us;
            obj.acquiredData.Info.pcArchitecture = computer('arch');
            obj.acquiredData.Info.pcOS = system_dependent('getos');
            obj.acquiredData.Info.matlabVersion = version('-release');

            % Setup GUI handle structure
            obj.gui = struct;
            obj.gui.DisplayIntensities = nan(1,obj.nDisplaySamples);
            obj.gui.DisplayTTL = nan(1,obj.nDisplaySamples);
            obj.gui.DisplayTimes = nan(1,obj.nDisplaySamples);

            % Setup GUI figure and UI elements
            obj.gui.Fig  = figure('name','Sensor Stream','numbertitle','off', 'MenuBar', 'none','Position',[100,400,1400,480], 'CloseRequestFcn', @(h,e)obj.endAcq());
            obj.gui.Plot = axes('units','normalized', 'position',[.07 .15 .9 .75]); ylabel('Sensor value (bits)', 'FontSize', 18); xlabel('Time (s)', 'FontSize', 18);
            obj.gui.thresholdLabel = uicontrol('Style', 'text', 'Position', [5 10 70 20], 'FontSize', 12, 'String', 'Thresh:');
            obj.gui.thresholdSet = uicontrol('Style', 'edit', 'Position', [80 10 70 20], 'FontSize', 10,...
                'BackgroundColor', [1 1 1], 'FontWeight', 'bold', 'Callback',@(h,e)obj.uiSetThreshold,...
                'String',num2str(obj.threshold));
            obj.gui.autoThreshButton = uicontrol('Style', 'pushbutton', 'Position', [160 10 80 20], 'FontSize', 12, 'String', 'AutoSet', 'Callback',@(h,e)obj.uiAutoSetThreshold);
            set(gca, 'xlim', [0 obj.maxDisplayTime]);
            obj.gui.tMaxLabel = uicontrol('Style', 'text', 'Position', [1090 10 100 20], 'FontSize', 12, 'String', 'tMax (s):');
            obj.gui.tMaxSet = uicontrol('Style', 'edit', 'Position', [1200 10 70 20], 'FontSize', 10,...
                'BackgroundColor', [1 1 1], 'FontWeight', 'bold', 'Callback',@(h,e)obj.uiSetTmax,...
                'String',num2str(obj.maxDisplayTime));
            obj.gui.resetRangeButton = uicontrol('Style', 'pushbutton', 'Position', [1290 10 100 20], 'FontSize', 12, 'String', 'RngReset', 'Callback',@(h,e)obj.UIresetRange);
            %set(gca, 'ylim', [15500000 16000000]);
            obj.gui.startStopButton = uicontrol('Style', 'pushbutton', 'Position', [1290 440 100 30], 'FontSize', 12, 'String', 'Stop', 'Callback',@(h,e)obj.uiStartStop);
            Xdata = nan(1,obj.nDisplaySamples); Ydata = nan(1,obj.nDisplaySamples);
            obj.gui.OscopeDataLine = line([Xdata,Xdata],[Ydata,Ydata], 'LineWidth', 1.5);
            obj.gui.OscopeTTLLine = line([Xdata,Xdata],[Ydata,Ydata], 'Color','black', 'LineWidth', 1.5);
            obj.gui.OscopeThreshLine = line([0, obj.nDisplaySamples],[obj.threshold,obj.threshold], 'Color','red','LineStyle','--');
            
            % Setup GUI variables
            obj.gui.DisplayPos = 1;
            obj.gui.SweepStartTime = 0;
            obj.gui.acquiredDataPos = 1;
            obj.gui.FirstSample = 1;
            obj.gui.resetRangeFlag = false;
            obj.gui.Ymax = NaN; obj.gui.Ymin = NaN;
            drawnow;

            % Setup & start GUI timer. The callback reads new data from the
            % serial port, logs the data and updates the plot.
            obj.streamTimer = timer('TimerFcn',@(h,e)obj.updatePlot(), 'ExecutionMode', 'fixedRate', 'Period', 0.01, 'Tag', ['LS_' obj.Port.PortName]);
            obj.startAcq;
        end

        function delete(obj)
            obj.Port = []; % Trigger the ArCOM port's destructor function (closes and releases port)
        end
    end
    methods (Access = private) % Internal methods
        function updatePlot(obj)
            BytesAvailable = obj.Port.bytesAvailable;
            if BytesAvailable > 1
                nBytesToRead = floor(BytesAvailable/4)*4;
                NewIntensities = obj.Port.read(nBytesToRead, 'uint8');
                nIntensities = length(NewIntensities)/4;
                lickDetectedBytes = NewIntensities(1:4:end);
                LickDetected = bitget(lickDetectedBytes, 1);
                NewDisplayTTL = (double(LickDetected)*double(obj.gui.Ymax-obj.gui.Ymin)) + double(obj.gui.Ymin);
                NewIntensities(1:4:end) = bitset(lickDetectedBytes, 1, 0);
                NewIntensities = typecast(NewIntensities, 'uint32');
                obj.acquiredData.Sensor(obj.gui.acquiredDataPos:obj.gui.acquiredDataPos+nIntensities-1) = NewIntensities;
                obj.acquiredData.TTL(obj.gui.acquiredDataPos:obj.gui.acquiredDataPos+nIntensities-1) = double(LickDetected);
                obj.gui.acquiredDataPos = obj.gui.acquiredDataPos + nIntensities;
                Div = obj.samplingRate; % Polling frequency (Hz), determined by READ_INTERVAL (us) in firmware. 
                Times = (obj.gui.DisplayPos:obj.gui.DisplayPos+nIntensities-1)/Div;
                DisplayTime = (Times(end)-obj.gui.SweepStartTime);
                obj.gui.DisplayPos = obj.gui.DisplayPos + nIntensities;
                if DisplayTime >= obj.maxDisplayTime
                    if obj.gui.FirstSample || obj.gui.resetRangeFlag
                        obj.gui.Ymax = max(obj.gui.DisplayIntensities);
                        obj.gui.Ymin = min(obj.gui.DisplayIntensities);
                        obj.gui.FirstSample = 0;
                        obj.gui.resetRangeFlag = false;
                    end
                    if max(obj.gui.DisplayIntensities) > obj.gui.Ymax
                        obj.gui.Ymax = max(obj.gui.DisplayIntensities);
                        set(obj.gui.Plot, 'ylim', [obj.gui.Ymin-(obj.gui.Ymin*0.0005) obj.gui.Ymax+(obj.gui.Ymax*0.0005)]);
                    end
                    if min(obj.gui.DisplayIntensities) < obj.gui.Ymin
                        obj.gui.Ymin = min(obj.gui.DisplayIntensities);
                        set(obj.gui.Plot, 'ylim', [obj.gui.Ymin-(obj.gui.Ymin*0.0005) obj.gui.Ymax+(obj.gui.Ymax*0.0005)]);
                    end
                    obj.resetSweep;
                else
                    SweepTimes = Times-obj.gui.SweepStartTime;
                    obj.gui.DisplayIntensities(obj.gui.DisplayPos-nIntensities:obj.gui.DisplayPos-1) = NewIntensities;
                    obj.gui.DisplayTTL(obj.gui.DisplayPos-nIntensities:obj.gui.DisplayPos-1) = NewDisplayTTL;
                    obj.gui.DisplayTimes(obj.gui.DisplayPos-nIntensities:obj.gui.DisplayPos-1) = SweepTimes;
                end
                set(obj.gui.OscopeTTLLine,'xdata',[obj.gui.DisplayTimes, obj.gui.DisplayTimes], 'ydata', [obj.gui.DisplayTTL, obj.gui.DisplayTTL]);
                set(obj.gui.OscopeDataLine,'xdata',[obj.gui.DisplayTimes, obj.gui.DisplayTimes], 'ydata', [obj.gui.DisplayIntensities, obj.gui.DisplayIntensities]); drawnow;
            end
            
        end

        function resetSweep(obj)
            obj.gui.DisplayIntensities(1:obj.nDisplaySamples) = NaN;
            obj.gui.DisplayTTL(1:obj.nDisplaySamples) = NaN;
            obj.gui.DisplayTimes(1:obj.nDisplaySamples) = NaN;
            obj.gui.DisplayPos = 1;
            obj.gui.SweepStartTime = 0;
        end

        function uiSetThreshold(obj)
            newThreshold = str2double(get(obj.gui.thresholdSet, 'String'));
            obj.threshold = newThreshold;
            obj.acquiredData.Params.threshold = newThreshold;
            set(obj.gui.OscopeThreshLine, 'ydata', [newThreshold,newThreshold]);
        end

        function uiSetTmax(obj)
            newTmax = str2double(get(obj.gui.tMaxSet, 'String'));
            obj.maxDisplayTime = newTmax;
            obj.nDisplaySamples = obj.samplingRate*obj.maxDisplayTime;
            set(obj.gui.Plot, 'xlim', [0 obj.maxDisplayTime]);
            obj.resetSweep;
        end

        function UIresetRange(obj)
            obj.gui.resetRangeFlag = true;
        end

        function uiAutoSetThreshold(obj)
            wasStreaming = false;
            if obj.streaming == 1
                wasStreaming = true;
                obj.stopAcq;
            end
            obj.autoSetThreshold;
            set(obj.gui.thresholdSet, 'string', num2str(obj.threshold))
            obj.uiSetThreshold;
            if wasStreaming
                obj.startAcq;
            end
        end

        function uiStartStop(obj)
            if obj.streaming
                obj.stopAcq;
                obj.streaming = false;
                set(obj.gui.startStopButton, 'string', 'Start');
            else
                obj.startAcq;
                obj.streaming = true;
                set(obj.gui.startStopButton, 'string', 'Stop');
            end
        end

        function startAcq(obj)
            obj.streaming = true;
            obj.Port.write(['S' 1], 'uint8');
            start(obj.streamTimer);
        end

        function stopAcq(obj)
            obj.streaming = false;
            obj.Port.write(['S' 0], 'uint8');
            stop(obj.streamTimer);
            pause(.1);
            obj.Port.flush;
        end

        function clearTimers(obj, portString)
            % Destroy any orphaned timers from previous instances
            T = timerfindall;
            for i = 1:length(T)
                thisTimer = T(i);
                thisTimerTag = get(thisTimer, 'tag');
                if strcmp(thisTimerTag, ['LS_' portString])
                    warning('off');
                    delete(thisTimer);
                    warning('on');
                end
            end
        end

        function computeMeasurementTime(obj)
            if obj.initialized
                SettleTime = (obj.settleCount*16)/(obj.extClkFreq/obj.refDivider);
                ConversionTime = ((obj.rCount*16)+4)/(obj.extClkFreq/obj.refDivider);
                obj.info.measurementTime_us = (SettleTime + ConversionTime)*1000000;
            end
        end

        function assertNotStreaming(obj, paramName)
            if obj.streaming
                error(['Error: ' paramName ' cannot be set during USB data streaming. '...  
                       'Close the streaming GUI first.'])
            end
        end

        function endAcq(obj)
            % End acquisition
            if obj.streaming
            obj.stopAcq;
            end
            delete(obj.streamTimer);
            obj.streamTimer = [];

            % Close GUI
            delete(obj.gui.Fig);

            % Trim preallocated data vectors to actual length
            nSamples = obj.gui.acquiredDataPos-1;
            obj.acquiredData.Sensor = obj.acquiredData.Sensor(1:nSamples);
            obj.acquiredData.TTL = obj.acquiredData.TTL(1:nSamples);
            obj.acquiredData.nSamples = nSamples;
        end
    end
end