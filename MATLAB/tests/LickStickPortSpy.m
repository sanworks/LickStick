% LickStickPortSpy stands between a LickStick object and its serial port, and
% notes the op code and the time of every command the object sends.
%
% Usage (LickStickClassTest does this):
% L = LickStick('COM4');
% spy = LickStickPortSpy(L.Port);
% L.Port = spy;                % From here on, L talks to the device through the spy
% ...
% spy.ops                      % The op codes sent, in order, e.g. 'STTTS'
% spy.times                    % When each was sent, in seconds since the spy was made

classdef LickStickPortSpy < handle
    properties
        Port            % The ArCOM_LickStick object that owns the serial port
        PortName        % Its port name (LickStick tags its timer with it)
        ops = '';       % Op code of each command sent
        times = [];     % Time of each, in seconds since the spy was made
    end
    properties (Access = private)
        startTime
    end
    methods
        function obj = LickStickPortSpy(port)
            obj.Port = port;
            obj.PortName = port.PortName;
            obj.startTime = tic;
        end

        function write(obj, varargin)
            % Every command's first byte is its op code (/Firmware/PROTOCOL.md)
            obj.ops(end+1) = char(varargin{1}(1));
            obj.times(end+1) = toc(obj.startTime);
            obj.Port.write(varargin{:});
        end

        function varargout = read(obj, varargin)
            [varargout{1:nargout}] = obj.Port.read(varargin{:});
        end

        function bytesAvailable = bytesAvailable(obj)
            bytesAvailable = obj.Port.bytesAvailable;
        end

        function flush(obj)
            obj.Port.flush;
        end

        function delete(obj)
            obj.Port = []; % Releases the serial port
        end
    end
end
