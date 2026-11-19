% Test of the MATLAB class (LickStick.m) on a connected device. Nothing needs to
% touch the sensor.
%
% Usage:
% nFailed = LickStickClassTest('COM4');              % With /MATLAB on the path
% nFailed = LickStickClassTest('COM4', 'plot.png');  % Also saves an image of the live plot
%
% It connects as a user's script would, and opens the streaming window for a few
% seconds, pressing its buttons from the code. Each check prints PASS, FAIL or
% INFO. It works with both builds of firmware v2 (/AGENTS.md, "Two builds of v2");
% what it cannot verify without op G it reports as INFO.
%
% It does not judge the window by eye: whether the plot looks right, and whether
% a lick shows, needs a person. It does not run a recording long enough (1 hour)
% to lengthen the data vectors. It leaves the device with the class's defaults,
% and the threshold 18,200,000.
%
% Keep the drink tube still while it runs.

function nFailed = LickStickClassTest(portName, varargin)
    plotFile = '';
    if nargin > 1
        plotFile = varargin{1};
    end
    nFailed = 0;

    % --- The constructor programs the class's defaults, and checks the firmware version
    L = LickStick(portName);
    L.rCount = 300;
    L.driveCurrent = 20;
    L.threshold = 5;
    clear L
    L = LickStick(portName);
    pause(.1);
    settings = readSettings(L);
    if isempty(settings)
        report('connect', 'INFO', ['connected; this firmware build does not answer op G, so the settings '...
                                   'on the device could not be read back']);
    else
        ok = isequal(settings, [18200000 500 256 10 1 31 0 0]);
        report('connect', ok, ['after connecting, the device held [' num2str(settings) ']']);
    end
    report('version', L.firmwareVersion == 2, ['firmware version ' num2str(L.firmwareVersion) ' accepted']);

    % --- A session that ended while streaming does not spoil the next connection
    L.Port.write(['S' 1], 'uint8');
    pause(.1);
    clear L
    try
        L = LickStick(portName);
        ok = plausible(L.readSensor(100));
        report('reconnect', ok, 'connected after a session that ended while streaming');
    catch err
        report('reconnect', false, ['could not connect after a session that ended while streaming: ' err.message]);
        return
    end

    % --- The automatic threshold
    L.autoSetThreshold;
    resting = L.readSensor(1000);
    ok = (L.threshold < min(resting)) && (double(L.threshold) > 0.98*double(min(resting)));
    report('autoset', ok, ['autoSetThreshold gave ' num2str(L.threshold) ' with the signal at ' num2str(min(resting))]);
    L.threshold = 18200000;

    % --- The streaming window: the threshold controls do not pause the recording.
    % A spy on the port notes what the class sends, and when
    spy = LickStickPortSpy(L.Port);
    L.Port = spy;
    problems = {};
    L.stream;
    fig = findobj('Type', 'figure', 'Name', 'LickStick');
    threshBox = findobj(fig, 'Style', 'edit', 'Position', [70 10 80 30]);
    pause(1);
    nSoFar = L.acquiredData.nSamples; % Counts up while streaming
    if nSoFar == 0
        problems{end+1} = 'nSamples was 0 after a second of streaming';
    end
    thresholdBefore = L.threshold;
    for i = 1:10
        press(fig, '+'); pause(.03);
    end
    if L.threshold ~= thresholdBefore + 20000
        problems{end+1} = ['10 presses of + moved the threshold by ' num2str(L.threshold - thresholdBefore)];
    end
    for i = 1:10
        press(fig, '-'); pause(.03);
    end
    press(fig, 'AutoSet');
    pause(1.5);
    nAuto = L.acquiredData.nSamples;
    resting = L.acquiredData.Sensor(nAuto-999:nAuto);
    autoThreshold = L.threshold;
    if ~(autoThreshold < min(resting) && autoThreshold > 0.98*double(min(resting)))
        problems{end+1} = ['AutoSet gave ' num2str(autoThreshold) ' with the signal at ' num2str(min(resting))];
    end
    if L.acquiredData.Params.threshold ~= autoThreshold || str2double(get(threshBox, 'String')) ~= autoThreshold
        problems{end+1} = 'AutoSet did not update the Thresh box or Params';
    end
    % A threshold above the signal, typed into the box, makes a lick in the recording
    typeInto(threshBox, '4294967295');
    pause(.3);
    typeInto(threshBox, num2str(autoThreshold));
    lickTime = diff(spy.times(end-1:end)); % Between the two threshold commands
    pause(.3);
    typeInto(threshBox, 'not a number');
    if L.threshold ~= autoThreshold || str2double(get(threshBox, 'String')) ~= autoThreshold
        problems{end+1} = 'text that is not a number changed the threshold';
    end
    try
        L.rCount = 512;
        problems{end+1} = 'rCount could be set while streaming';
    catch
    end
    if ~isempty(plotFile)
        exportgraphics(fig, plotFile);
    end
    pause(.5);
    close(fig);
    data = L.acquiredData;
    % From the command that started the stream to the one that stopped it
    if ~strcmp(spy.ops([1 end]), 'SS') || any(spy.ops(2:end-1) ~= 'T')
        problems{end+1} = ['during the recording the class sent the ops ' spy.ops ', not S, T ... T, S'];
    end
    streamTime = spy.times(end) - spy.times(1);
    expected = streamTime*L.samplingRate;
    if abs(data.nSamples - expected) > 0.005*expected + 50
        problems{end+1} = sprintf('%d samples recorded, %.0f expected', data.nSamples, expected);
    end
    if length(data.Sensor) ~= data.nSamples || length(data.TTL) ~= data.nSamples
        problems{end+1} = 'Sensor and TTL were not trimmed to nSamples';
    end
    if ~plausible(data.Sensor)
        problems{end+1} = 'the recording holds values that are not sensor readings';
    end
    % Before AutoSet the default threshold may be above the signal (a lick throughout): count from there
    nLick = sum(data.TTL(nAuto:end));
    if abs(nLick - lickTime*L.samplingRate) > 0.05*L.samplingRate
        problems{end+1} = sprintf('a threshold above the signal for %.2f s was recorded as a lick of %d samples', lickTime, nLick);
    end
    if ~plausible(L.readSensor(50))
        problems{end+1} = 'readSensor after the window closed returned wrong values';
    end
    if isempty(problems)
        report('stream', true, sprintf(['%d samples in the %.2f s between the start and stop commands (%.0f expected), '...
            'through 21 button presses that sent only op T; a threshold above the signal for %.2f s was recorded '...
            'as a lick of %d samples'], data.nSamples, streamTime, expected, lickTime, nLick));
    else
        report('stream', false, strjoin(problems, '; '));
    end

    % --- clearAcquiredData restarts the recording, and a second call of stream does not
    problems = {};
    L.stream;
    L.stream;
    fig = findobj('Type', 'figure', 'Name', 'LickStick');
    if numel(fig) ~= 1
        problems{end+1} = [num2str(numel(fig)) ' windows are open after two calls of stream'];
    end
    pause(1);
    nBefore = L.acquiredData.nSamples;
    L.clearAcquiredData;
    nCleared = L.acquiredData.nSamples; % The stream timer can run between two statements, so not always 0
    pause(.5);
    L.stream; % Still the same recording
    close(fig);
    data = L.acquiredData;
    sinceClear = spy.times(end) - spy.times(end-1) - 1; % The clear came at least 1 s after the start
    if nBefore < 0.2*L.samplingRate || nCleared > 0.2*L.samplingRate
        problems{end+1} = sprintf('%d samples before clearAcquiredData, %d right after', nBefore, nCleared);
    end
    if data.nSamples < 0.4*L.samplingRate || data.nSamples > sinceClear*L.samplingRate || length(data.Sensor) ~= data.nSamples
        problems{end+1} = sprintf('%d samples in the recording, cleared at most %.2f s before it ended', data.nSamples, sinceClear);
    end
    if ~plausible(data.Sensor)
        problems{end+1} = 'the recording holds values that are not sensor readings';
    end
    if isempty(problems)
        report('clear', true, sprintf('%d samples before clearAcquiredData, %d right after, %d when the window closed', nBefore, nCleared, data.nSamples));
    else
        report('clear', false, strjoin(problems, '; '));
    end

    L.threshold = 18200000;
    clear L
    fprintf('\n%d failed\n', nFailed);

    function report(name, outcome, detail)
        if ischar(outcome)
            label = outcome;
        elseif outcome
            label = 'PASS';
        else
            label = 'FAIL';
            nFailed = nFailed + 1;
        end
        fprintf('%-4s  %s: %s\n', label, name, detail);
    end
end

function settings = readSettings(L)
    % The device's settings, read back with op G (/Firmware/PROTOCOL.md): threshold,
    % sampling interval, rCount, settleCount, refDivider, driveCurrent, channel, LED.
    % Empty if the build does not answer it (the read then takes 3 s)
    L.Port.write('G', 'uint8');
    try
        reply = L.Port.read(32, 'uint8');
    catch
        settings = [];
        return
    end
    settings = [double(typecast(reply(1:4), 'uint32')) double(typecast(reply(5:8), 'single'))...
                double(typecast(reply(9:12), 'uint16')) double(reply(13:16))];
end

function ok = plausible(values)
    % True if every value is near the others: readings of the resting sensor, with
    % no byte out of place
    values = double(values);
    middle = median(values);
    ok = ~isempty(values) && middle > 1e6 && all(abs(values - middle) < 0.05*middle);
end

function press(fig, label)
    % Runs the callback of the pushbutton with this label, as a click does
    button = findobj(fig, 'Style', 'pushbutton', 'String', label);
    callback = get(button, 'Callback');
    callback(button, []);
end

function typeInto(editBox, text)
    % Sets an edit box's text and runs its callback, as typing and Enter do
    set(editBox, 'String', text);
    callback = get(editBox, 'Callback');
    callback(editBox, []);
end
