# Working on this repository

LickStick is an open source lick detector for rodent neuroscience experiments. A drink tube is
one plate of a capacitor; a Texas Instruments FDC2214-Q1 measures the frequency of an LC tank
that includes it, a Teensy 4.0 reads the FDC2214 over I2C 2000 times a second, compares each
reading with a threshold, and drives two TTL outputs high while the animal's tongue touches the
tube. Over USB, a MATLAB or Python class sets the sensor up, reads and logs the signal, and
shows it in a live plot with controls for the threshold. The current firmware is v2.

This file is about changing the repository. To use a LickStick from MATLAB or Python, for
example to write or check an experiment script, read `/USING.md`: the standard calls, the
parameters, the data format and the pitfalls. Keep it up to date when a change alters what
users see.

A path that starts with `/` is in this repository.

## Layout

| Path | Contents |
|---|---|
| `/USING.md` | Using the device from MATLAB and Python: setup, standard calls, parameters, pitfalls |
| `/Firmware/LickStick/` | The firmware, an Arduino sketch for Teensy 4.0. `LickStick.ino`: pins, `setup()`, the sample buffer and the command handler in `loop()`, and `readCycle()`, the sampling interrupt that detects licks. `FDC2214.h/.cpp`: the sensor driver, with the register bit fields named in the header. `ArCOM.h`: typed reads with a timeout, and buffered writes, on the USB serial port |
| `/Firmware/PROTOCOL.md` | The USB serial protocol, as the firmware source implements it: ops, the status reply, the sample stream, power-on state, value limits, and how the released build differs |
| `/Firmware/Compiled/` | Released firmware, one file per version (`LickStickFirmware_v2.hex`), `LoadLickStickFirmware.m`, which loads the newest onto a device from MATLAB, and the `tycmd` uploader it runs. **The v2 file is older than the source** (see "Two builds of v2") |
| `/MATLAB/LickStick.m` | The MATLAB class, with the streaming GUI. `ArCOM_LickStick.m` wraps the serial port; `ArCOM_DoubleSidedBuffer.m` is its receive buffer |
| `/MATLAB/tests/LickStickClassTest.m` | Checks the MATLAB class on a connected device, pressing the GUI's buttons from code. `LickStickPortSpy.m` stands between the class and its port and notes each command sent |
| `/Python/lickstick.py` | The Python class, a port of the MATLAB class (tkinter and matplotlib GUI), with a thread that reads the stream |
| `/Python/tests/lickstick_hardware_test.py` | Checks the firmware on a connected device, through the raw protocol |
| `/Python/tests/lickstick_class_test.py` | Checks the Python class on a connected device, pressing the GUI's buttons from code |
| `/CAD/` | Circuit board (Eagle), enclosure and drink tube mount (SketchUp, 3MF), labels. `/CAD/PCB/v1_0/` is the current board |

The FDC2214-Q1 datasheet (TI document SNOSCZ9) is the reference for everything in
`FDC2214.cpp`. It is not in the repository. Section and table numbers in the code comments and
in `/Firmware/PROTOCOL.md` are those of revision A (October 2024).

## The hardware, as the firmware sees it

From `/CAD/PCB/v1_0/LickStick v1_0_1.sch`:

| Teensy pin | Net | Purpose |
|---|---|---|
| 18, 19 | SDA, SCL | I2C to the FDC2214 at address 0x2A (ADDR pin grounded), 4.7 kΩ pullups, 400 kHz (the FDC2214's maximum) |
| 10 | CLK_EN | Enables the 40 MHz oscillator that clocks the FDC2214 (CLKIN) |
| 11 | INTB | The FDC2214's interrupt output. The firmware enables its data-ready signal but never reads the pin |
| 12 | SHUTDOWN | The FDC2214's SD pin. High resets every register |
| 15 | DIO1 | Lick output, middle screw terminal |
| 14 | DIO2 | Lick output, right screw terminal. Same signal as DIO1 |
| 13 | LED | Teensy's LED |

Each of the FDC2214's channels 0 and 1 has an LC tank on the board between its INxA and INxB
pins (C9 15 pF, C10 20 pF), brought out to a 4-pin connector. Channels 2 and 3 are not
connected. The clients number the channels 1 and 2; the firmware and the FDC2214 number them 0
and 1.

## The protocol ties everything together

The firmware and its two clients encode the same serial protocol, documented in
`/Firmware/PROTOCOL.md`. A change on one side usually needs a matching change on the others:
make them together, and update `PROTOCOL.md` in the same commit.

- **Op codes and message layouts are fixed**, because installed copies of the clients and
  loaded firmware rely on them. Add a new op instead of changing an existing one.
- **A change in what the firmware does gets a new `FIRMWARE_VERSION`** (`LickStick.ino`), a new
  `/Firmware/Compiled/LickStickFirmware_vN.hex` (the loader takes the highest N), and the same
  number in the clients: `_currentFirmwareVersion` in `lickstick.py` and
  `currentFirmwareVersion` in `LickStick.m`. A client that must still work with older firmware
  checks the version before it sends a newer op, as `samplingRate` does for op `I`.
- **The clients mirror each other.** `lickstick.py` has the MATLAB class's properties, methods,
  GUI layout and `acquiredData` fields under the same names. A fix to one usually belongs in
  both.
- **The clients read no setting back.** A client's copy of a setting is right only if that
  client sent it, so both constructors send every setting: a new object puts the device back
  to the class defaults, the threshold included. The firmware source now has op `G` for
  reading them, and the clients do not use it yet.
- **Both constructors refuse a firmware version other than `currentFirmwareVersion`**, older
  or newer (tried with builds that answer 1 and 3). The `firmwareVersion > 1` test in
  `samplingRate` is therefore never false: it is kept as the pattern for an op that a later
  version adds.

### Two builds of v2

The firmware source was changed in November 2026 (sample buffer, read timeouts, argument
checks, ops `G`, `X` and `E`, and the fixes listed under "Fixed in the firmware source"), and
the maintainer chose to keep `FIRMWARE_VERSION` at 2 for it. Until the release is rebuilt,
`/Firmware/Compiled/LickStickFirmware_v2.hex` is the build from before, so two firmwares in
use answer `F` with 2:

- Op `G` tells them apart: the earlier build sends no reply. A client that uses `G`, `X` or `E`
  must cope with that, with a read timeout.
- Both clients work with either build: `lickstick_class_test.py` and `LickStickClassTest.m`
  pass with both (measured: 7 runs of 7 with the released build after the last change, and
  with the source build).
- `/USING.md` describes the released build, and says which pitfalls the source build ends.
- Replacing the released file is the maintainer's decision. Do not rebuild it unasked.

## How the firmware runs

Two contexts share the sensor and the I2C bus:

- **`readCycle()`** is the `IntervalTimer` callback, every 500 µs by default. It reads the
  active channel's two data registers over I2C (blocking: the whole interrupt takes 241 µs,
  measured, and op `G` reports its longest run), applies the threshold and the debounce, sets
  the lick outputs, and, while streaming, puts the sample in `sampleBuffer`. It runs at
  priority 64: above the USB interrupt (128), below the system tick (32).
- **`loop()`**, on each pass, stops the stream if the PC closed the port, sends buffered
  samples to USB (`sendSamples()`), handles one USB command, and sends its reply. Ops `W`,
  `N`, `D`, `C` and `!` write FDC2214 registers from here.

The interrupt can run at any point in `loop()`. Rules that follow from it:

1. **The I2C bus belongs to one context at a time.** `loop()` sets `sensorBusy` around every
   call of a `SensorIC` setter, and `readCycle()` then skips its read. A new op that touches
   the sensor must do the same. `FDC2214` keeps nothing in a member between two transfers but
   the settings themselves: a buffer shared by `readSensor()` and the setters was the cause of
   fixed issue 2.
2. **The sensor gives no reading while it sleeps**, and until its first conversion after it
   wakes: its data registers read 0. Every setter except `set_ACTIVE_CHANNEL()` sleeps it, as
   the datasheet asks (section 6.4.1: change a setting only in sleep mode). `readSensor()`
   returns false for a reading of 0 and for a failed transfer, and `readCycle()` then holds
   the lick state and repeats the last reading in the stream. A 0 must never reach the
   threshold comparison: it is below any threshold, so it would count as a lick. If no reading
   comes for `SENSOR_TIMEOUT_MICROS`, a lick in progress is ended.
3. **Only `loop()` uses the USB port.** `Serial.write()` waits up to 120 ms when the PC is not
   reading (`TX_TIMEOUT_MSEC` in the Teensy core's `usb_serial.c`), then discards data until
   the PC reads again. `sendSamples()` writes no more than `Serial.availableForWrite()`, which
   never waits, and whole samples only. Replies go through `USBCOM.flush()`, which can wait,
   but that only delays `loop()`. Nothing that `readCycle()` calls may use `Serial` or `USBCOM`.
4. **Variables that both contexts use** are single words, `volatile`, each written in one
   instruction, and listed with their owner under "Program variables" in `LickStick.ino`.
   `loop()` changes the stream's counters only after it has cleared `usbStreaming`
   (`startStreaming()`). Keep it so: masking the timer interrupt would cost samples.
5. **`CONFIG` is written last, and holds the sleep bit and the active channel.** The setters
   write it with `sleep()` and `wake()`, which build it with `configValue()`; a new setter
   must too, or it switches the channel back to 0. Reserved bits must be written as the
   datasheet gives them (`CONFIG_RESERVED` in `FDC2214.h`; section 6.6.28).
6. **A command's bytes must arrive within 100 ms of each other.** After that, `ArCOM`'s reads
   return zeros, and `loop()` refuses the command. Every op that reads an argument calls
   `commandValid()` with its range check before it acts, and does nothing if that is false. A
   new op must too.
7. **A time that `readCycle()` counts in samples is set in `setSamplingInterval()`**, so that
   it follows op `I` (the debounce, the sensor timeout).
8. **The lick outputs must keep working without a PC.** Detection runs from power-on, whether
   or not a client is connected, and a behavior system may be reading DIO1 and DIO2 while a
   client changes settings. The one thing that still pauses it is a write to the emulated
   EEPROM (op `E`): the Teensy core masks all interrupts while it writes flash.

The register values that `FDC2214::init()` writes are named, field by field, in `FDC2214.h`.

## Working on the clients

- **A property assignment programs the device at once**, after a range check. All except
  `threshold` are refused while streaming (`assertNotStreaming()`), because their ops sleep the
  sensor.
- **The handshake copes with a device that is still streaming.** After opening the port, a
  constructor waits 50 to 100 ms; if bytes arrive unasked, it sends `S` 0 and discards input
  until 100 ms pass without any (2 s at most), then sends `F`. The released build streams
  until it is told to stop, so without this the reply to `F` was a sample. No reply to `F`
  within 3 s is an error in both.
- **While streaming, one reader owns the port's input.** In MATLAB it is a timer
  (`streamTimer`, tagged `LS_<port>`, `fixedSpacing`, 10 ms after each callback ends: fixed
  issue 16) whose callback, `updatePlot()`, reads, logs
  and plots; `stream()` returns at once. In Python it is a thread (`_readStream()`) that reads
  and logs, while `updatePlot()`, called by tkinter every 10 ms in the main thread, plots what
  the thread logged; `stream()` runs the tkinter main loop until the window closes, and
  `stream(block=False)` returns at once and leaves the window to `update()`. Other code may
  write to the port meanwhile (op `T`), and must not read from it: every method that reads a
  reply calls `assertNotStreaming()` first.
- **The reader can run between any two statements of the other code.** In Python that is plain
  threading: the thread and the main thread hold `_dataLock` whenever they use the data
  vectors or `gui['acquiredDataPos']`, and only the main thread touches tkinter. In MATLAB the
  timer's callback was seen to run between the statements of a loop with no `pause` or
  `drawnow` (R2025b, measured: 54 times in a 1 s loop), where older releases ran it only when
  the event queue was flushed. Write MATLAB code that the callback can interrupt at any
  statement, as `clearAcquiredData` explains.
- **The threshold controls do not stop the stream.** `+`, `-` and the Thresh box send op `T`.
  AutoSet notes the position of the next sample (`autoThreshStartPos`), and `updatePlot()`
  sets the threshold once 1000 more samples are logged. Only the Stop button and closing the
  window send `S` 0. `stopAcq()` then logs the samples still on their way before it discards
  anything, so a recording holds every sample sent between the two `S` ops.
- **`acquiredData`** holds what was streamed: `Sensor` (uint32 values, lick bit cleared), `TTL`
  (uint8, the lick bit), `nSamples`, `Params` and `Info`. Both vectors start at 7.2 million
  samples (1 hour at 2 kHz), are lengthened by as much when they fill, and are trimmed when
  the window closes; `nSamples` counts up while streaming. Sample n was taken
  n / samplingRate seconds after streaming started only if no sample was lost and the Stop
  button was not used: the stream the clients ask for (op `S`) carries no counter (known
  issue 14).
- **MATLAB serial ports.** `ArCOM_LickStick` is a copy of the general ArCOM class: it uses
  `serialport` on R2019b and newer, and PsychToolbox's IOPort or `serial` before. Its TCP and
  Octave branches are not used here. Its read timeout (3 s) is timed with `tic` and `toc`: the
  general class compares the seconds field of `clock`, and a read that began in the last 3 s
  of a minute never timed out (seen: a test run hung on an op the device did not answer).
- **Stale comments were a source of confusion.** When a change alters a number or a unit that a
  comment states, change the comment in the same edit.

## Known issues

Found in a review in November 2026. "Measured" means reproduced on a LickStick v1.0.1; the
name in brackets is the check that shows it, in `lickstick_hardware_test.py` for the firmware
and in `lickstick_class_test.py` and `LickStickClassTest.m` for the clients. Fix an issue in
the firmware and every client together, then update this list, `PROTOCOL.md` and the pitfalls
in `USING.md`. The numbers are kept from the review, so that comments and messages that cite
them stay right. An issue that is fixed in part is in more than one list.

### Open

11. **`refDivider` above 3 breaks the datasheet's fIN < fREF / 4.** The firmware and the
    clients accept up to 255.
12. **Sensor faults are barely visible.** The firmware source counts failed I2C transfers
    (`nSensorErrors` in the reply to op `G`), and nothing reads the count. The FDC2214's
    watchdog and amplitude warnings are not enabled (`ERROR_CONFIG` holds only the data-ready
    bit).
13. **Smaller things.** Disabling the LED during a lick leaves it on. `set_ACTIVE_CHANNEL()`
    writes `CONFIG` with the sensor awake, where the datasheet asks for sleep (6.4.1).
    `acquiredData` exists only in memory until the user saves it: a session that crashes
    loses its recording. MATLAB's `threshold` setter stores a value outside 0 to 4294967295
    as given, while the device gets the limit (Python refuses it).
14. **The clients do not use what the firmware source added**: the settings read-back (op
    `G`), the sample numbers (op `X`), which show where samples were lost, the lost sample
    count, and the stored settings (op `E`), without which a device still detects nothing
    after a power cut. They cannot tell the two builds of v2 apart by version (see "Two builds
    of v2").
15. **The factory threshold detects nothing.** `THRESH_DEFAULT` (1,900,000) is far below the
    signal, on purpose: the right threshold depends on the tube and its cable. A device
    detects licks from power-on only after op `E` stored a threshold.

17. **In MATLAB's dark theme the lick trace does not show**: it is drawn in black, on a
    background that R2025b makes near black (seen in an image of the plot).

### Fixed in the clients

In `LickStick.m` and `lickstick.py`, with either firmware build. Each check passes with both
builds (measured).

6. **Neither client stopped a stream or discarded input before its handshake** (`reconnect`,
   and `handshake` in Python). With the released build, a session that ended while streaming
   left the next one reading a sample as the firmware version. Now: the handshake rule under
   "Working on the clients".
7. **MATLAB never checked the firmware version.** `currentFirmwareVersion` was never
   assigned, and a comparison with `[]` is false. Now 2, and both classes refuse a build that
   answers 1 or 3 (tried by hand: the tests do not load firmware).
8. **The GUI's threshold buttons paused the stream** (`stream`). `+`, `-` and AutoSet stopped
   and restarted it, 100 ms or more in MATLAB, and nothing marked the gap in `acquiredData`.
   Now they send only op `T`, and a recording has the samples of its whole duration: 9852 of
   9863 in MATLAB and 14971 of 14971 in Python, through 21 presses.
9. **The Python class did not program the device when it connected** (`connect`), so its
   properties described the device only by chance. Now as in MATLAB. Also fixed:
   `_read_exact()` had no timeout (`timeout`, `longread`: it now gives up after 3 s without a
   byte); `stream()` needed `blank.ico` in the working directory, and blocked, so that
   `clearAcquiredData()` could not be used (`stream`, `clear`, `blocking`); a negative
   automatic threshold raised `struct.error` (`threshold`); `stopAcq()` discarded input
   while samples were still arriving.
13. **Both classes allocated 300 MB or more for `acquiredData`**, MATLAB 288 MB of that for an
    instant as doubles, and a recording longer than 36 million samples made MATLAB lengthen
    its vectors at every timer call and Python raise an error. Now 36 MB, lengthened by an
    hour of samples when full (`stream` in Python, with a small allocation; in MATLAB the
    lengthening has not been run).

16. **In MATLAB R2025b, the open plot held up the code beside it.** `updatePlot()` takes
    about 18 ms, longer than the 10 ms period of its timer, which was `fixedRate`: a callback
    was always due, and a `pause` returned late. 60 calls of `pause(.1)` beside the plot took
    19, 30 and 47 s with the class as it was before the review (one pause lasted 40 s). The
    timer is now `fixedSpacing`, which counts its period from the end of a callback: the same
    calls took 6.2 to 9.1 s in 9 runs, and in 2 of the 9 one pause still lasted more than 1 s
    (2.8 s at most; cause not found). A period of 20 ms was no better, and `fixedRate` at
    50 ms gave 7.0 to 7.5 s. Measured with `matlab -batch`; earlier releases, and a session
    with the desktop open, were not tried. The plot is now redrawn about 40 times a second.
    The Python class was not affected: its reader is a thread.

Found on the way, and fixed: `ArCOM_LickStick`'s read timeout failed in the last 3 s of every
minute (see "MATLAB serial ports").

### Fixed in the firmware source

Not yet in `/Firmware/Compiled/LickStickFirmware_v2.hex`. Each check failed with that build
and passes with the source build (measured).

1. **A sensor setting op made a false lick** (`falselick`). Ops `W`, `N`, `D`, `C` put the
   sensor to sleep, the samples read meanwhile were 0, and the lick outputs went high for
   10.5 ms. Now: rule 2.
2. **`rCount` and `settleCount` were programmed wrongly on channel 2** (`channel2`,
   `channel1`). `set_RCOUNT()` and `set_SETTLECOUNT()` passed bytes of a shared buffer to
   `writeRegister16()`, which overwrote it, and `readSensor()`, in the interrupt, used the
   same buffer: channel 2 got the wrong low byte, or 0 (a reserved value: seen after 14 of 60
   `W` ops). A write from `loop()` could also be lost to the interrupt: 1 of 1500 `W` ops on
   channel 1. Now: rule 1 (0 of 1500 with the source build).
3. **The sensor value halved at a client's first `D` op** (`poweron`). `init()` programmed
   `FIN_SEL` b01 and `set_REF_DIVIDER()` b10, so the same device read 36 million from Python
   and 18 million from MATLAB. Now b10 in both, as the datasheet asks for a single-ended
   sensor: 18 million from power-on. Op `E` stores the settings, threshold included.
4. **Samples were lost, and detection paused, when the PC stopped reading** (`stall`,
   `overflow`, `indexed`). The interrupt wrote to USB itself, and waited 120 ms there once the
   buffers were full. Now: rule 3, a buffer of 32768 samples, a count of the samples dropped
   beyond it, and op `X`, which sends each sample's number.
5. **A command cut short took the following commands' bytes** (`truncated`, `limits`). Now:
   rule 6, and every argument is range-checked on the device.
6. **Streaming was not stopped when a client disconnected** (`reconnect`). Now `loop()` stops
   it when the PC's DTR signal falls.
10. **The debounce was 20 samples, whatever the sampling rate** (`debounce`): 40 ms at 500 Hz.
    Now 10 ms at any rate (rule 7). It applies after both edges, so no lick shorter than that
    is reported, and none closer to the last than that.
11. **The firmware checked no argument** (`limits`). A sampling interval shorter than
    `readCycle()` takes would have left `loop()` no time. Now the device refuses an interval
    under 400 µs, a channel index above 1, and the sensor's reserved values.
13. **`FDC2214`'s constructor never stored its pin numbers**, set the pins up before `setup()`
    ran, and started the sensor 1 ms after releasing SD, where the datasheet allows it 2 ms to
    wake (section 5.6). Now `init()` does all of it, and waits 3 ms.

Tested and not found: 200 replies to `F` sent during streaming arrived whole, among the
samples (100 of 100 with the source build).

Tried and of little use: reading each data register with a repeated start, in place of a stop
and a start, shortened the interrupt from 244 to 241 µs (measured). Its length is the 90 or so
bits of two register reads at 400 kHz, the FDC2214's maximum, and the data-ready signal on
INTB would not shorten a read. Only an I2C driver that does not block would free that time,
and nothing needs it now that the interrupt no longer competes with USB.

## Checks

Without a device:

```bash
# Compile the firmware (arduino-cli ships with the Arduino IDE; needs the teensy:avr core, 1.62.0 used here)
arduino-cli compile --fqbn teensy:avr:teensy40 Firmware/LickStick

# Python: syntax
python -m py_compile Python/lickstick.py Python/tests/lickstick_hardware_test.py Python/tests/lickstick_class_test.py

# MATLAB: static analysis. The messages about set methods that access other properties, and
# about datestr and now, were there before the review
matlab -batch "checkcode('MATLAB/LickStick.m')"
```

A change that is meant to leave the firmware's behavior alone (comments, renames) should
compile to the same `.hex`: build before and after with `--build-path`, and compare the files.

With a device (pyserial and numpy; `/Python/requirements.txt`):

```bash
python Python/tests/lickstick_hardware_test.py COM4            # every check but the power-on and stored settings ones
python Python/tests/lickstick_hardware_test.py COM4 --reset    # restarts the device several times; stores and erases settings (op E)

# Load a build onto the device (the path is the one given to --build-path)
Firmware/Compiled/tycmd.exe upload --board @COM4 build/LickStick.ino.hex
```

With the firmware source loaded, every check passes (21 results; the 20 s `overflow` check
makes a run about 2 minutes long). With the released `LickStickFirmware_v2.hex`, the checks
of fixed issues 1 to 6 fail and those that use op `G` report that it was not answered. A
change must not turn a check to FAIL. Close MATLAB's and Python's connections first: only one
program can hold the port. Nothing has to touch the sensor, but it must be left still.

The clients, with a device (matplotlib too for Python). Each opens the streaming window for a
few seconds at a time and presses its buttons from code:

```bash
python Python/tests/lickstick_class_test.py COM4                                  # 9 results, about 30 s
matlab -batch "addpath('MATLAB', 'MATLAB/tests'); LickStickClassTest('COM4');"    # 6 results, about 1 minute
```

Both pass with either firmware build; with the released build, `connect` is INFO, because
the settings cannot be read back without op `G`. They time the stream by the class's own
`S` commands (Python: `startAcq()` and the moment before `endAcq()`; MATLAB:
`LickStickPortSpy`), because the count of samples that MATLAB shows while streaming is behind
the stream by a varying amount.

There are no offline tests of the clients yet. Only a device with a person at it can test the
lick signal itself, the TTL outputs on a scope, and how the plot looks. When you finish a
change, state plainly what was and was not tested, and give the user a short list of things
to check on the device.

## Style

- Match the surrounding code, and keep existing names for variables, functions, ops and
  properties.
- Comments explain why, and record the traps: interrupt context, fixed numbers, datasheet
  constraints with their section.
- MATLAB follows Richard Johnson's MATLAB Style Guidelines 2.0; Python follows PEP 8, except
  that `lickstick.py` keeps the MATLAB class's camelCase names so that the two read alike.
- Work on the `develop` branch. `main` holds the latest release.
- Commit messages list the changes per component, e.g. `-Firmware: Fixed X -MATLAB: Added Y`.
- Pulse Pal (github.com/sanworks/PulsePal and PulsePal-Firmware) went through the same
  process, and its `AGENTS.md` files describe solutions to several of the issues above. Taken
  over in the firmware so far: an `ArCOM` with read timeouts and buffered replies, and a timer
  interrupt above USB priority. In the clients: a handshake that discards input until the
  device is quiet. Still to take over in the clients: offline tests against a simulated port
  (`LickStickPortSpy.m` shows where one would go in MATLAB).
