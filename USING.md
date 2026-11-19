# Using LickStick

This guide is for writing experiment code with a LickStick: what the device does, the standard
calls in MATLAB and Python, the parameters, the data it records, and the mistakes that are easy
to make. It is written for scientists and for AI agents that help them. To change this
repository instead (the firmware, the classes), see `AGENTS.md`.

It describes the released firmware (`/Firmware/Compiled/LickStickFirmware_v2.hex`, which
`LoadLickStickFirmware` loads) with the classes as they are in this repository. Statements
marked **(measured)** were checked on a LickStick v1.0.1 in November 2026.

The firmware source in `/Firmware/LickStick` is newer than that file, under the same version
number. A device loaded with a build of the source behaves as described here, except that
**(measured)**:

- changing a sensor setting, and so connecting, no longer pulses the outputs (pitfall 1);
- channel 2 works (pitfall 2);
- the sensor value is near 18 million from power-on, before any class has connected
  (pitfall 4);
- the computer can stop reading for about 17 s, not 1.5 s, before samples are lost, and the
  device's lick detection does not pause meanwhile (pitfall 5);
- the hold time after each edge is 10 ms at any sampling rate (pitfall 7).

## What the device does

- The drink tube is part of a tuned circuit whose frequency a sensor chip (TI FDC2214-Q1)
  measures. The **sensor value** is that frequency as an integer, near 18 million on the test
  device. A tongue on the tube adds capacitance, so
  **the value falls during a lick**.
- 2000 times a second, the device compares the value with the **threshold**. While the value is
  at or below it, a lick is detected: the two digital outputs, DIO1 (middle screw terminal) and
  DIO2 (right screw terminal), are high (3.3 V), and the board's LED is on if `ledEnabled` is
  set. After each change, the outputs hold their state for 20 samples (10 ms at 2 kHz).
- Detection runs on the device, with or without a computer. A computer is needed to set the
  threshold, which the device forgets when it loses power, and to record the signal.
- Over USB, the device sends each sample to the computer: the value, and whether a lick was
  detected.

## Before an experiment

1. **Install.** MATLAB: add `/MATLAB` to the path. Python: install pyserial, numpy and
   matplotlib (`pip install -r Python/requirements.txt`), and run your script from the
   `/Python` folder, or add that folder to `sys.path`. The Python class has been tested on
   Windows only.
2. **Find the port.** `serialportlist` in MATLAB; `python -m serial.tools.list_ports` in Python.
   If several are listed, unplug the device and see which one disappears. Only one program at a
   time can have the port open.
3. **Check the firmware.** The classes need firmware v2. To load it, run
   `LoadLickStickFirmware('COM3')` in MATLAB (in `/Firmware/Compiled`; Windows and Linux).
4. **Connect and set the sensor up before the behavior system starts counting licks.**
   Connecting, and changing any sensor setting, puts a 10 ms pulse on the outputs
   (pitfall 1).
5. **Set the threshold after every connection, with the animal in place and not licking**:
   `autoSetThreshold`, then look at the live plot (`stream`) while the tube is touched.
   Connecting sets the class's default threshold, which is not right for every setup
   (pitfall 3).
6. **Leave the device powered.** Its settings last until it loses power or restarts, or a
   class connects again. They survive closing the connection, so the outputs keep working
   after `clear L`.

## Standard calls

| | MATLAB | Python |
|---|---|---|
| Connect | `L = LickStick('COM3');` | `from lickstick import LickStick`<br>`L = LickStick('COM3')` |
| Set a parameter | `L.rCount = 512;` | `L.rCount = 512` |
| Set the threshold from the resting signal | `L.autoSetThreshold;` | `L.autoSetThreshold()` |
| Set the threshold by hand | `L.threshold = 18050000;` | `L.threshold = 18050000` |
| Read sensor values | `v = L.readSensor(100);` | `v = L.readSensor(100)` |
| Open the live plot and record | `L.stream;` (returns at once) | `L.stream()` (returns when the window closes), or<br>`L.stream(block=False)` (returns at once) |
| Keep the plot alive from your own loop | `drawnow;` or `pause(t);` | `L.update()` (after `stream(block=False)`) |
| Drop what was recorded so far | `L.clearAcquiredData;` | `L.clearAcquiredData()` |
| End the recording | `close(findobj('Type', 'figure', 'Name', 'LickStick'));`, or close the window | `L.endAcq()`, or close the window |
| Get the recording | `data = L.acquiredData;` | `data = L.acquiredData` |
| Disconnect | `clear L` (end the recording first) | `del L`, or end the script (end the recording first) |

A wrong port, or a device that does not answer, gives an error within about 3 s in both
classes. So does firmware of another version than 2.

`readSensor(n)` returns n consecutive samples and takes n / samplingRate seconds. In MATLAB a
read gives up after 3 s, so ask for less than 3 s of samples (6000 at 2 kHz). In Python a read
gives up only when nothing has arrived for 3 s.

`autoSetThreshold` reads 1000 samples and sets the threshold 5 times their range below their
minimum (not below 0). On the test device at rest the range was 4752, which puts the threshold
0.13% under the resting value **(measured)**.

### The live plot

`stream` opens a window that shows the last 2 s of the sensor value (blue), the threshold
(red, dashed) and the lick output (black: low at the bottom of the plot, high at the top), and
records every sample it receives. Calling it again while the window is open only brings the
window to the front.

- **MATLAB.** `stream` returns at once, and a timer reads the samples about 40 times a second. The timer
  runs while MATLAB is idle or in `pause` or `drawnow`, so a protocol that runs alongside must
  call one of them often. (In R2025b it also ran between the statements of a loop that called
  neither **(measured)**, so `acquiredData` can change from one statement to the next.) The
  plot can make a `pause` last longer than asked (pitfall 11).
- **Python.** A thread reads and records the samples, whatever your code does. The window is
  drawn by tkinter: `L.stream()` runs its main loop and returns when the window is closed;
  after `L.stream(block=False)` the window is redrawn, and its controls work, only when your
  code calls `L.update()`. Recording does not depend on `update()` **(measured: no sample
  missing after 1.5 s without it)**.

| Control | Does |
|---|---|
| Thresh box | Sets the threshold to the number typed, on Enter. Text that is not a number from 0 to 4294967295 is ignored |
| + and − | Move the threshold by 2000, about one step of the sensor's resolution at `rCount` 256 |
| AutoSet | Sets the threshold as `autoSetThreshold` does, from the next 1000 samples (0.5 s at 2 kHz). Press it while the animal is not licking |
| tMax | The time span of the plot, in seconds |
| RngReset | Fits the vertical range to the signal again at the next sweep |
| Stop / Start | Stops and starts the stream. Nothing is recorded while it is stopped |

None of the three threshold controls interrupts the recording **(measured: no sample missing
through 21 presses, in MATLAB and in Python)**. Closing the window ends the recording and
trims `acquiredData` to the samples received.

## Parameters

Each is a property of the object. Assigning one checks its range and programs the device at
once. All but `threshold` raise an error while the live plot is streaming.

| Property | Default | Range | Meaning |
|---|---|---|---|
| `threshold` | 18200000 | 0 to 4294967295 | A lick is detected while the sensor value is at or below it |
| `activeChannel` | 1 | 1 or 2 | Which of the board's two sensor inputs is measured. Use 1 (pitfall 2) |
| `samplingRate` | 2000 | 500 to 2000 Hz | How often the sensor is read, compared with the threshold, and sent to the computer |
| `rCount` | 256 | 256 to 65535 | Length of one measurement, in units of 16 clock cycles. Higher gives finer resolution and a longer measurement |
| `settleCount` | 10 | 2 to 65535 | Wait before the first measurement after the sensor wakes, in units of 16 clock cycles |
| `refDivider` | 1 | 1 to 255 | Divides the 40 MHz reference clock. Leave at 1 (pitfall 6) |
| `driveCurrent` | 31 | 0 to 31 | Current driving the tube's circuit: 16 µA to 1.571 mA |
| `ledEnabled` | false | true, false | The board's LED follows the lick output |

`L.info.measurementTime_us` is the time one measurement takes with the current settings:
106.5 µs with the defaults. At `rCount` 256 the resting signal on the test device took 6
distinct values, about 1900 apart, with a range of 4752; at `rCount` 2559 it took 23, with a
range of 2648 **(measured)**. A measurement longer than the sampling interval (`rCount` above
1249 at 2 kHz) makes consecutive samples repeat.

Both classes send these defaults to the device when the object is created (pitfall 3). A
class knows only what it sent: it does not read the device's settings.

## The recording

`acquiredData`:

| Field | Contents |
|---|---|
| `Sensor` | The sensor value of every sample received (uint32) |
| `TTL` | The lick output at every sample: 1 while a lick was detected (uint8) |
| `nSamples` | Number of samples received so far |
| `Params` | `threshold`, `samplingRate`, `activeChannel`, `rCount`, `settleCount`, `refDivider`, `driveCurrent`. The threshold is the last one set |
| `Info` | `dateTime` when `stream` was called (`yyyymmdd_HHMMSS`), `measurementTime_us`, `pcArchitecture`, `pcOS`, `matlabVersion` (in Python, the Python version) |

In MATLAB it is a struct, in Python a dict with NumPy arrays. It is held in memory only: save
it yourself (`save`, `numpy.savez`). While the plot window is open, `Sensor` and `TTL` are
longer than `nSamples` (they start at 7.2 million samples, 1 hour at 2 kHz, and are lengthened
by as much when full): use only the first `nSamples` of each. Closing the window trims them.

Sample n was taken n / samplingRate seconds after the stream started, provided the Stop
button was not used and no sample was lost (pitfall 5). Lick onsets are the rising edges of
`TTL`: `find(diff(double(data.TTL)) == 1) + 1` in MATLAB,
`np.flatnonzero(np.diff(data['TTL'].astype(int)) == 1) + 1` in Python.

## Recipes

### Set up, then record a session (MATLAB)

```matlab
L = LickStick('COM3');     % Programs the defaults. Puts pulses on DIO1 and DIO2 (pitfall 1)
L.autoSetThreshold;        % Animal in place, not licking
L.stream;                  % Live plot. Check that licks cross the threshold
% ... start the behavior system ...
L.clearAcquiredData;       % Recording starts here
% ... run the session ...
close(findobj('Type', 'figure', 'Name', 'LickStick'));   % Ends the recording
data = L.acquiredData;
save('licks.mat', 'data');
clear L
```

### Set up, then record a session (Python)

```python
import numpy as np
from lickstick import LickStick   # Run from the /Python folder, or add it to sys.path

L = LickStick('COM3')      # Programs the defaults. Puts pulses on DIO1 and DIO2 (pitfall 1)
L.autoSetThreshold()       # Animal in place, not licking
L.stream(block=False)      # Live plot. Returns at once: a thread records from here on
# ... check that licks cross the threshold, calling L.update() in a loop ...
# ... start the behavior system ...
L.clearAcquiredData()      # Recording starts here
while session_is_running():   # Your protocol's loop
    ...
    L.update()             # Redraws the plot and runs its buttons
L.endAcq()                 # Ends the recording and closes the window
data = L.acquiredData
np.savez('licks.npz', Sensor=data['Sensor'], TTL=data['TTL'])
```

With nothing to run alongside, `L.stream()` alone shows the plot and returns when its window
is closed.

### Use the outputs without recording

Connect, run `autoSetThreshold`, check the plot, close it, and disconnect. The device goes on
detecting with those settings until it loses power. After a power cut it detects nothing until
a computer sets the threshold again (pitfall 4).

## Pitfalls

1. **Changing a sensor setting pulses the outputs.** Each assignment to `rCount`,
   `settleCount`, `refDivider` or `driveCurrent` makes the device report a lick for 10.5 ms
   **(measured)**, because the sensor reads zero while it restarts. Creating the object, in
   MATLAB or Python, assigns all four. Do it before the behavior system starts counting, and
   do not change these during a session. `threshold`, `activeChannel` and `ledEnabled` do not
   do this.
2. **Channel 2 is unreliable.** After `rCount` is assigned (which creating the object does),
   channel 2 returned only the values 268435454, 17895696 and 16777216 in about a quarter of
   tries **(measured)**. When it does work, its measurement length is not quite what `rCount`
   asks: the device replaces the low byte of channel 2's `rCount` with 8. Use channel 1. If
   you must use channel 2, read some samples after every `rCount` assignment, and assign it
   again if they are those three values.
3. **Connecting sets every parameter to its default, the threshold included.** A threshold
   set in an earlier session is gone as soon as a new object is created, in MATLAB and in
   Python. The default, 18.2 million, was above the test device's resting value of 18.06
   million **(measured)**, so the device reported a lick all the time from the moment of
   connecting until the threshold was set. Set the threshold after every connection, and
   connect before the behavior system starts counting.
4. **Nothing is stored on the device.** After a power cut or a restart, the threshold is far
   below the signal and no lick is detected until a computer sets it again. Until a class has
   connected, the sensor value is also twice as large (36 million instead of 18 million
   **(measured)**), which matters only to other software that reads it.
5. **Samples are lost if the computer stops reading.** The stream has no counter, so a loss
   cannot be seen in the data. When nothing read the port for more than about 1.5 s, samples
   were discarded (1710 of 10000 after 2.5 s) and the device's own lick detection paused for
   about 130 ms **(measured)**. In MATLAB the port is read by a timer, which waits while your
   code computes without `pause` or `drawnow` (in releases before R2025b at least), and while
   MATLAB draws or saves. In Python it is read by a thread, which only something that holds
   the whole interpreter can stop. Keep the computer free during a session, and prefer the
   outputs to the recording for lick times that matter.
6. **Leave `refDivider` at 1.** The sensor's datasheet needs the reference clock to be more
   than four times the measured frequency, which holds up to 3 here. The classes accept up to
   255.
7. **The hold time after each edge is 20 samples, not 10 ms.** At `samplingRate` 500 it is
   40 ms, and licks shorter than that, or closer together, are merged or lengthened.
8. **End the recording before disconnecting.** While the plot window is open, its timer
   (MATLAB) or its thread (Python) keeps the object, and the port, alive: `clear L` and
   `del L` do nothing until the window is closed.
9. **A recording exists only in memory until you save it.** If MATLAB or Python ends during a
   session, it is lost. To save during a session, copy the first `nSamples` samples of
   `Sensor` and `TTL`. An hour at 2 kHz takes 36 MB. In MATLAB, lengthening the vectors after
   each hour has not been run on a device.
10. **The Stop button leaves a gap.** Nothing is recorded between Stop and Start, and nothing
    in `acquiredData` marks where. Do not use it in a session whose sample times matter.
11. **MATLAB: a `pause` beside the open plot can last longer than asked.** In R2025b, 60
    calls of `pause(.1)` beside the plot took 6.2 to 9.1 s where 6 s was asked, and in 2 runs
    of 9 one of them lasted more than 1 s (2.8 s at most) **(measured)**. No sample is lost.
    A protocol that needs exact timing should take it from the behavior system, not from
    `pause`. (Before November 2026 the class was far worse here: the same calls took 19 to
    47 s. Update `LickStick.m` if yours is older.) Other MATLAB releases were not tried.
    Python is not affected.

## More

- The USB protocol, for other languages: `/Firmware/PROTOCOL.md`.
- A check of a device from the command line: `/Python/tests/lickstick_hardware_test.py`.
- The causes of the pitfalls, and what is being done about them: "Known issues" in
  `/AGENTS.md`.
