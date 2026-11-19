# LickStick USB serial protocol

This page documents the protocol of firmware v2 as the source in `/Firmware/LickStick`
implements it. The MATLAB class (`/MATLAB/LickStick.m`) and the Python class
(`/Python/lickstick.py`) are its two clients.

**Two builds answer "version 2".** The source was changed in November 2026 without a new
version number. `/Firmware/Compiled/LickStickFirmware_v2.hex` is the build from before that
change, and differs as listed under "The earlier build of v2" at the end of this page. A client
can tell the two apart with op `G`, which the earlier build does not answer.

Statements marked **(measured)** were checked on a LickStick v1.0.1 in November 2026 with
`/Python/tests/lickstick_hardware_test.py`. Unmarked statements were read from the code.

## Transport

- The device is a Teensy 4.0, which appears as a USB CDC serial port. The baud rate a client
  asks for is ignored: transfers run at USB speed.
- All multi-byte values are little-endian.
- A command is one op byte, followed by its arguments. There is no framing byte, no checksum and
  no length field.
- Only ops `F`, `G`, `E`, `R`, `S` and `X` send anything back. No other op is acknowledged; a
  client that wants to know whether its settings arrived reads them back with op `G`.
- `loop()` handles one command per pass, in the order the bytes arrive.
- **A command is refused**, with no reply (op `E` replies 0), when:
  - an argument is outside the device's limits (see "Value limits");
  - the op byte is not in the table below;
  - its bytes stop arriving for 100 ms. The bytes that did arrive are dropped, and the next byte
    is taken as an op **(measured: an `F` sent 0.5 s after half of a `T` was answered)**.

  Each refusal adds 1 to `nCommandErrors` in the reply to op `G`.
- Sensor sampling and the lick outputs run in a timer interrupt, and do not wait for anything
  the PC does.
- **Closing the port stops the stream.** When the PC's DTR signal falls, which happens when a
  program closes the port, the device stops streaming and discards its unsent samples
  **(measured: the device was quiet when the port was opened again)**. Both clients raise DTR
  when they open the port. A program that never raises it gets no such stop.

## Ops

| Op | Byte | Arguments | Reply | Effect |
|---|---|---|---|---|
| `F` | 70 | none | uint32 firmware version | |
| `G` | 71 | none | 32 bytes: see "Status" | Returns the settings and the status counters |
| `R` | 82 | uint32 nSamples | nSamples sample words | Sends the next nSamples samples, one per sampling interval, then stops. nSamples = 0 starts continuous streaming, as `S` 1 |
| `S` | 83 | uint8 enable | sample words while enabled | 1 starts continuous streaming, 0 stops it. Any nonzero value starts it |
| `X` | 88 | uint8 enable | 8-byte records while enabled | As `S`, but each sample word follows its sample number (see "Sample stream"). 0 stops any stream |
| `T` | 84 | uint32 threshold | none | Lick detection threshold. Takes effect at the next sample. Safe during streaming |
| `W` | 87 | uint16 rCount | none | FDC2214 `RCOUNT` for both sensor channels. Sleeps and wakes the sensor |
| `N` | 78 | uint16 settleCount | none | FDC2214 `SETTLECOUNT` for both sensor channels. Sleeps and wakes the sensor |
| `D` | 68 | uint8 refDivider | none | FDC2214 `FREF_DIVIDER` for both sensor channels. Sleeps and wakes the sensor |
| `C` | 67 | uint8 driveCurrent | none | FDC2214 `IDRIVE` (0-31) for both sensor channels. Sleeps and wakes the sensor |
| `!` | 33 | uint8 channel | none | Active sensor channel index: 0 or 1 (the clients' channels 1 and 2). Written without sleeping the sensor |
| `I` | 73 | float32 interval | none | Sampling interval in microseconds. Takes effect when the interval in progress ends |
| `L` | 76 | uint8 enable | none | 1: the board LED follows the lick output. 0: the LED keeps its last state |
| `E` | 69 | uint8 store | uint8: 1 done, 0 refused | 1 stores the current settings as the power-on settings. 0 erases the stored settings (see "Stored settings") |

While the sensor sleeps for `W`, `N`, `D` or `C` it gives no reading. The device then holds the
lick state and repeats the last reading in the stream, so these ops are not seen on the lick
outputs or in the data **(measured)**.

### Status (reply to op `G`)

| Offset | Type | Field |
|---|---|---|
| 0 | uint32 | threshold |
| 4 | float32 | sampling interval, µs |
| 8 | uint16 | rCount |
| 10 | uint16 | settleCount |
| 12 | uint8 | refDivider |
| 13 | uint8 | driveCurrent |
| 14 | uint8 | active channel index (0 or 1) |
| 15 | uint8 | LED enabled (0 or 1) |
| 16 | uint8 | streaming: 0 off, 1 sample words (`S`, `R`), 2 indexed (`X`) |
| 17 | uint8 | 1 if the settings at power-on were stored ones (op `E`), 0 if the factory's |
| 18 | uint16 | nCommandErrors: commands refused since power-on |
| 20 | uint32 | nSamplesDropped: samples lost since streaming last started (see "Sample stream") |
| 24 | uint32 | nSensorErrors: I2C transfers with the sensor that failed since power-on |
| 28 | uint32 | Longest run of the sampling interrupt since the last op `G`, µs **(measured: 241)** |

The sensor settings are the firmware's copies, each updated when the sensor acknowledged the
write of the new value. The firmware does not read the sensor's registers back.

## Sample stream

Each sample is one uint32 **sample word**:

| Bits | Contents |
|---|---|
| 31-28 | 0 |
| 27-1 | The upper 27 bits of the FDC2214's 28-bit conversion result |
| 0 | Lick detected: 1 while the lick output is high |

Clients read bit 0 as the lick state, then clear it to get the sensor value.

- **Ops `S` and `R`** send the sample words alone, with no header, counter or timestamp. A
  client's only time base is then the number of samples it has received.
- **Op `X`** sends 8 bytes per sample: a uint32 **sample number**, then the sample word. The
  number counts sampling intervals from 0 at the start of streaming, so a step greater than 1
  between two records is a number of lost samples at that point **(measured)**.

Samples wait in a buffer on the device until the PC takes them: 32768 samples (16 s at 2 kHz),
beyond the 12 kB or so that the USB buffers of the device and the PC hold. While the PC does not
read, sampling and lick detection carry on **(measured: with the PC idle for 2.5 s, none of
10000 samples was lost, and a threshold change sent meanwhile took effect at the right
sample)**. When the buffer is full, new samples are dropped and counted in `nSamplesDropped`
**(measured: 6481 after 20 s idle, and the sample numbers of op `X` skipped 6481 in one gap)**.

- **Stopping** (`S` 0, `X` 0, or the port closing) discards the samples the device has not sent
  yet. Bytes already in the USB buffers still arrive: a client should read until the port is
  quiet before it sends an op that replies.
- **Replies during streaming.** A reply to `F`, `G` or `E` sent during streaming arrives whole,
  between two samples **(measured: 100 of 100 replies to `F`)**, but nothing marks it. Stop the
  stream first.
- The sensor gives no reading while it sleeps for a setting op, until its first conversion
  after it wakes, and when an I2C read fails (counted in `nSensorErrors`). The stream then
  repeats the last reading. The word is 0 only before the first reading after power-on.

## Sensor value

The value is `fSENSOR / 2 / fREF * 2^28` (datasheet equation 2 with `FIN_SEL` = b10, the
datasheet's setting for a single-ended sensor, section 7.1.1), with fREF = 40 MHz / refDivider:
about 18.06 million at rest on the test device **(measured, at power-on and after op `D`)**.

## Lick detection

Runs in the sampling interrupt (`readCycle()`), with or without a USB connection.

- A lick is detected while the value is at or below the threshold. A touch adds capacitance,
  which lowers the sensor's frequency and so the value.
- After each change of state, the state is held for 10 ms (`DEBOUNCE_MICROS`), counted in
  samples: 20 at 2 kHz, 5 at 500 Hz **(measured: the shortest lick lasted 10.5 ms at 2 kHz and
  12 ms at 500 Hz)**.
- While the sensor gives no reading, the state is held. If it gives none for 250 ms
  (`SENSOR_TIMEOUT_MICROS`), a lick in progress is ended.
- While a lick is detected, DIO1 (pin 15) and DIO2 (pin 14) are high (3.3 V), and the LED
  (pin 13) too if op `L` enabled it.

## Power-on state

The factory settings, unless op `E` stored others:

| Setting | Value |
|---|---|
| threshold | 1,900,000 (below any value the sensor gives, so no lick is detected until a client sends op `T`) |
| sampling interval | 500 µs (2 kHz) |
| active channel | index 0 |
| rCount | 256 |
| settleCount | 10 |
| refDivider | 1 |
| driveCurrent | 31 |
| LED | disabled |

Streaming is always off at power-on.

### Stored settings

`E` 1 stores the eight settings above, as they are at that moment, in the Teensy's emulated
EEPROM. At power-on the device takes them if the stored set is whole (key and checksum) and
within the limits below, and the factory settings otherwise **(measured: all eight in force
after a restart, and the factory settings again after `E` 0 and a restart)**. A device with a
stored threshold detects licks with no PC connected.

The Teensy core masks all interrupts while it writes its flash memory, so sampling and lick
detection pause during `E`. The pause was too short to measure from the PC when 8 settings
changed (under 1 ms) **(measured)**. It will be longer on the rare write for which the core
first erases a flash sector: that case was not provoked or measured. Send `E` when a pause does
not matter, not during a session.

## Value limits

The device refuses values outside the middle column. The clients refuse values outside the
right-hand column before they send anything.

| Setting | Device accepts | Clients accept | Notes |
|---|---|---|---|
| sampling interval | 400 µs to 1 s | 500 to 2000 µs (2000 to 500 Hz) | The sampling interrupt takes 241 µs of each interval **(measured)**: two I2C register reads at 400 kHz, the sensor's maximum. `loop()` runs in the rest |
| rCount | 256-65535 | 256-65535 | Below 256 is reserved (datasheet 6.6.10). Conversion time is `(rCount * 16 + 4) / fREF`: 102.5 µs at 256. Above 1249 it exceeds the 500 µs sampling interval, and consecutive samples repeat **(measured at 2559 and 5375)** |
| settleCount | 0-65535 | 2-65535 | Settle time is `settleCount * 16 / fREF`, once per wake in single-channel mode |
| refDivider | 1-255 | 1-255 | 0 is reserved (datasheet 6.6.22). The datasheet needs fIN < fREF / 4 (table 6-1). With fIN near 2.7 MHz, only 1 to 3 meet it |
| driveCurrent | 0-31 | 0-31 | 16 µA to 1.571 mA (datasheet 6.6.31). The datasheet asks for a sensor amplitude of 1.2 to 1.8 V peak |
| threshold | 0-4294967295 | 0-4294967295 | MATLAB limits a value outside the range to it (the cast to uint32). Python refuses it |
| active channel | 0 or 1 | 1 or 2 | The clients send their number minus 1 |

## The earlier build of v2

`/Firmware/Compiled/LickStickFirmware_v2.hex` was built before the changes of November 2026. It
answers `F` with 2 as well, and differs as follows. Each fault was measured on that build, and
has a check in `/Python/tests/lickstick_hardware_test.py`.

- **No ops `G`, `X` or `E`.** They are discarded without a reply, as any unknown op is. Nothing
  can be read back or stored.
- **No argument is checked.**
- **Sensor setting ops produce a false lick.** During `W`, `N`, `D` and `C` the samples read
  are 0, which is below the threshold, so the lick output goes high for 10.5 ms, and two
  samples of 0 are streamed.
- **`W` and `N` program channel index 1 wrongly.** Its register gets the register address of
  channel 0 as its low byte, or 0x0000 when the sampling interrupt runs between the two writes.
  `RCOUNT_CH1` = 0 is reserved, and channel index 1 then reads `0x0FFFFFFE`, `0x01111110` and
  `0x01000000` in turn: after 14 of 60 `W` ops. A register write could also be lost: after 1 of
  1500 `W` ops, channel index 0 kept its previous `RCOUNT`.
- **The value halves at the first op `D`.** `FIN_SEL` is b01 at power-on and b10 after `D`:
  36.13 million at power-on, 18.06 million after `D` 1.
- **A stalled client loses samples without notice.** The interrupt writes each sample to USB
  itself. When the PC has not read for about 1.5 s at 2 kHz, the next write waits 120 ms inside
  the interrupt, with no sampling and no lick detection, and samples are then discarded until
  the PC reads again: 692 of 10000 after a 2 s stall, 2704 after 3 s.
- **A command cut short takes the next commands' bytes** as its missing arguments: reads of
  arguments wait for ever.
- **Streaming survives the client.** A client that connects later finds the device still
  streaming, and would read a sample word as the reply to `F`. Both classes therefore send
  `S` 0 and discard input before `F`, if bytes arrive unasked when they open the port
  **(measured: both connect to a device left streaming)**.
- **The debounce is 20 samples at any sampling rate**: 40 ms at 500 Hz.
- A failed I2C read gives the sample `0xCFFFFFFE`.
