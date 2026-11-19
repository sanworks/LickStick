"""Hardware test for LickStick firmware. Needs a connected device; nothing needs to touch the sensor.

    python tests/lickstick_hardware_test.py COM4
    python tests/lickstick_hardware_test.py COM4 --reset     # also checks the power-on state and op E
    python tests/lickstick_hardware_test.py COM4 --only stall truncated

It talks to the device with the raw protocol (/Firmware/PROTOCOL.md), not through lickstick.py,
so it tests the firmware alone. Each check prints PASS, FAIL or INFO. The exit code is the number
of failed checks.

The checks named falselick, channel2, poweron, stall, truncated and reconnect each show a fault
of firmware v2 as first released (LickStickFirmware_v2.hex of 2025), and fail with it. The checks
that use op G, X or E need the firmware in /Firmware/LickStick, and fail with "no reply to op G"
without it.

The test leaves the device as a MATLAB session would (rCount 256, settleCount 10, refDivider 1,
driveCurrent 31, channel 1, 2 kHz) with the threshold 1% below the resting signal. With --reset,
it also stores and erases settings on the device (op E), which ends with none stored, and the
device is restarted at the end (Windows and Linux; uses /Firmware/Compiled/tycmd), which restores
its power-on state.

Requires pyserial and numpy. Keep the drink tube, and anything connected to it, still while the
test runs: several checks assume a steady signal.
"""
import argparse
import platform
import struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import serial

FIRMWARE_VERSION = 2
SAMPLING_RATE = 2000  # Hz, the firmware's power-on rate
SAMPLE_BUFFER_SIZE = 32768  # Samples the firmware buffers while the PC does not read
DEBOUNCE_MS = 10
REPO_ROOT = Path(__file__).resolve().parents[2]

# Reply to op G (/Firmware/PROTOCOL.md)
STATUS_FORMAT = '<IfHHBBBBBBHIII'
STATUS_FIELDS = ('threshold', 'samplingInterval', 'rCount', 'settleCount', 'refDivider', 'driveCurrent',
                 'activeChannel', 'ledEnabled', 'streaming', 'storedSettingsLoaded', 'nCommandErrors',
                 'nSamplesDropped', 'nSensorErrors', 'maxReadCycleMicros')
FACTORY_SETTINGS = {'threshold': 1900000, 'samplingInterval': 500.0, 'rCount': 256, 'settleCount': 10,
                    'refDivider': 1, 'driveCurrent': 31, 'activeChannel': 0, 'ledEnabled': 0}

# Values a channel reads when its RCOUNT register holds a reserved value (bit 0 cleared)
INVALID_RCOUNT_VALUES = {0x0FFFFFFE, 0x01111110, 0x01000000}

results = []  # (name, outcome, detail)


def report(name, outcome, detail):
    results.append((name, outcome, detail))
    print(f'{outcome:4s}  {name}: {detail}', flush=True)


def open_port(port_name):
    """Opens the port, stops a stream an earlier session may have left on, and discards its bytes."""
    port = serial.Serial(port_name, 115200, timeout=0.02)  # The baud rate is ignored: USB CDC
    time.sleep(0.2)
    port.write(b'S\x00')
    time.sleep(0.3)
    port.reset_input_buffer()
    return port


def read_exact(port, n_bytes, timeout):
    data = bytearray()
    start = time.perf_counter()
    while len(data) < n_bytes and (time.perf_counter() - start) < timeout:
        data += port.read(n_bytes - len(data))
    return bytes(data)


def read_until_quiet(port, quiet_s=0.5):
    data = bytearray()
    last = time.perf_counter()
    while time.perf_counter() - last < quiet_s:
        chunk = port.read(65536)
        if chunk:
            data += chunk
            last = time.perf_counter()
    return bytes(data), last


def split(raw):
    """Sample words -> (sensor values, lick bits)."""
    words = np.frombuffer(raw[:len(raw) // 4 * 4], dtype='<u4')
    return words & 0xFFFFFFFE, words & 1


def read_samples(port, n_samples, sampling_rate=SAMPLING_RATE):
    """Op 'R'. Returns (values, lick bits, seconds taken)."""
    start = time.perf_counter()
    port.write(struct.pack('<BI', ord('R'), n_samples))
    raw = read_exact(port, 4 * n_samples, n_samples / sampling_rate * 2 + 3)
    values, lick = split(raw)
    return values, lick, time.perf_counter() - start


def read_status(port):
    """Op 'G'. Returns the device's settings and counters as a dict. The stream must be off."""
    port.write(b'G')
    reply = read_exact(port, struct.calcsize(STATUS_FORMAT), 1)
    if len(reply) != struct.calcsize(STATUS_FORMAT):
        raise RuntimeError(f'no reply to op G ({len(reply)} bytes): the firmware predates it')
    return dict(zip(STATUS_FIELDS, struct.unpack(STATUS_FORMAT, reply)))


def run_lengths(values):
    """Lengths of the runs of equal consecutive samples, without the first and last (cut short).
    The sensor converts continuously, and a sample repeats until the next conversion ends, so the
    run length is the conversion time in samples."""
    change = np.flatnonzero(np.diff(values) != 0)
    edges = np.concatenate(([0], change + 1, [len(values)]))
    return np.diff(edges)[1:-1]


def set_client_defaults(port):
    """The settings the MATLAB class programs when it connects."""
    port.write(struct.pack('<BH', ord('W'), 256))
    port.write(struct.pack('<BH', ord('N'), 10))
    port.write(b'D\x01')
    port.write(b'C\x1f')
    port.write(b'!\x00')
    port.write(b'I' + struct.pack('<f', 1e6 / SAMPLING_RATE))
    port.write(b'L\x00')
    time.sleep(0.3)
    port.reset_input_buffer()


def set_threshold_below_signal(port):
    values, _, _ = read_samples(port, 1000)
    threshold = int(values.mean() * 0.99)
    port.write(struct.pack('<BI', ord('T'), threshold))
    return threshold


# ---------------------------------------------------------------------------------------------
# Checks. Each takes the open port and returns it (a check may close and reopen it).
# ---------------------------------------------------------------------------------------------

def check_handshake(port, args):
    port.write(b'F')
    reply = read_exact(port, 4, 1)
    if len(reply) == 4 and struct.unpack('<I', reply)[0] == FIRMWARE_VERSION:
        report('handshake', 'PASS', f'firmware v{FIRMWARE_VERSION}')
    else:
        report('handshake', 'FAIL', f'reply to F was {reply.hex() or "nothing"}')
    return port


def check_signal(port, args):
    values, lick, seconds = read_samples(port, 4000)
    rate = len(values) / seconds
    detail = (f'{len(values)} samples in {seconds:.3f} s ({rate:.0f} Hz), mean {values.mean():.0f}, '
              f'sd {values.std():.0f}, peak to peak {int(values.max()) - int(values.min())}, '
              f'{len(np.unique(values))} distinct values')
    ok = (len(values) == 4000 and abs(rate - SAMPLING_RATE) < 0.01 * SAMPLING_RATE
          and (values >> 28).max() == 0 and (values > 0).all())
    report('signal', 'PASS' if ok else 'FAIL', detail)
    return port


def check_setting_ops_false_lick(port, args):
    """W, N, D and C put the sensor to sleep. The readings of 0 meanwhile must not count as a lick,
    nor appear in the stream."""
    threshold = set_threshold_below_signal(port)
    ops = {'W': struct.pack('<BH', ord('W'), 256), 'N': struct.pack('<BH', ord('N'), 10),
           'D': b'D\x01', 'C': b'C\x1f', '!': b'!\x00', 'T': struct.pack('<BI', ord('T'), threshold)}
    failed = []
    for name, command in ops.items():
        port.write(struct.pack('<BI', ord('R'), 1600))
        time.sleep(0.3)
        port.write(command)  # Arrives while the 0.8 s of samples are being sent
        values, lick = split(read_exact(port, 6400, 4))
        if lick.any() or (values == 0).any() or len(values) != 1600:
            failed.append(f'{name}: {len(values)} samples, {int((values == 0).sum())} zero samples, '
                          f'lick output high for {lick.sum() / SAMPLING_RATE * 1000:.1f} ms')
        time.sleep(0.1)
        port.reset_input_buffer()
    if failed:
        report('setting ops leave lick output alone', 'FAIL', '; '.join(failed))
    else:
        report('setting ops leave lick output alone', 'PASS', 'no zero sample or lick during ' + ', '.join(ops))
    return port


def check_channel_2_rcount(port, args):
    """W must program channel index 1 with the same value as channel index 0."""
    port.write(b'!\x01')
    time.sleep(0.1)
    n_trials = 40
    n_invalid = 0
    for _ in range(n_trials):
        time.sleep(np.random.random() * 0.002)  # Vary the timing against the sampling interrupt
        port.write(struct.pack('<BH', ord('W'), 256))
        time.sleep(0.05)
        port.reset_input_buffer()
        values, _, _ = read_samples(port, 200)
        if set(np.unique(values).tolist()) <= INVALID_RCOUNT_VALUES:
            n_invalid += 1
    report('channel 2 reads after rCount is set', 'FAIL' if n_invalid else 'PASS',
           f'invalid readings after {n_invalid} of {n_trials} W ops')

    # With rCount 0x09FF a conversion lasts 2.05 samples, so no sample stands alone. If the low
    # byte of channel index 1's register is 0x08 instead, it lasts 1.85 samples, and some do.
    for attempt in range(20):
        port.write(struct.pack('<BH', ord('W'), 0x09FF))
        time.sleep(0.05)
        port.reset_input_buffer()
        values, _, _ = read_samples(port, 4000)
        if not set(np.unique(values).tolist()) <= INVALID_RCOUNT_VALUES:
            break
    runs = run_lengths(values)
    n_single = int((runs == 1).sum())
    report('channel 2 conversion time matches rCount', 'FAIL' if n_single else 'PASS',
           f'rCount 0x09FF: {n_single} of {len(runs)} conversions lasted one sample (expected none)')
    port.write(struct.pack('<BH', ord('W'), 256))
    port.write(b'!\x00')
    time.sleep(0.2)
    port.reset_input_buffer()
    return port


def check_channel_1_rcount(port, args):
    """Channel index 0's RCOUNT follows op W: alternate two values and check the conversion time."""
    n_trials = args.rcount_trials
    wrong = asleep = 0
    for i in range(n_trials):
        r_count, expected_min_run = ((0x09FF, 2), (0x14FF, 4))[i % 2]
        time.sleep(np.random.random() * 0.002)
        port.write(struct.pack('<BH', ord('W'), r_count))
        time.sleep(0.04)
        port.reset_input_buffer()
        values, _, _ = read_samples(port, 240)
        if (values == 0).all():
            asleep += 1
        elif run_lengths(values).min() != expected_min_run:
            wrong += 1
    port.write(struct.pack('<BH', ord('W'), 256))
    time.sleep(0.2)
    port.reset_input_buffer()
    report('channel 1 conversion time matches rCount', 'FAIL' if (wrong or asleep) else 'PASS',
           f'{n_trials} W ops: {wrong} with the wrong conversion time, {asleep} left the sensor asleep')
    return port


def check_host_stall(port, args):
    """No sample may be lost while the PC does not read for a few seconds, and lick detection must
    carry on meanwhile: a threshold change sent during the stall shows up at the right samples."""
    n_samples = 10000
    stall_s = 2.5
    values, _, _ = read_samples(port, 1000)
    below = int(values.mean() * 0.99)
    above = int(values.mean() * 1.01)
    port.write(struct.pack('<BI', ord('T'), below))
    time.sleep(0.05)
    start = time.perf_counter()
    port.write(struct.pack('<BI', ord('R'), n_samples))
    time.sleep(1.0)
    port.write(struct.pack('<BI', ord('T'), above))  # Lick output high from 1.0 s ...
    time.sleep(0.5)
    port.write(struct.pack('<BI', ord('T'), below))  # ... to 1.5 s
    time.sleep(stall_s - 1.5)
    raw, last_byte_time = read_until_quiet(port)
    n_received = len(raw) // 4
    extra_ms = (last_byte_time - start - n_samples / SAMPLING_RATE) * 1000
    detail = (f'PC idle for {stall_s} s: {n_samples - n_received} of {n_samples} samples lost, '
              f'stream ended {extra_ms:.0f} ms late')
    report('no samples lost when the PC stalls', 'PASS' if n_received == n_samples else 'FAIL', detail)

    _, lick = split(raw)
    high = np.flatnonzero(lick)
    if len(high) == 0:
        report('lick detection during the stall', 'FAIL', 'the threshold change made no lick')
    else:
        first_s = high[0] / SAMPLING_RATE
        length_s = len(high) / SAMPLING_RATE
        one_run = (high[-1] - high[0] + 1) == len(high)
        ok = one_run and abs(first_s - 1.0) < 0.1 and abs(length_s - 0.5) < 0.1
        report('lick detection during the stall', 'PASS' if ok else 'FAIL',
               f'threshold raised from 1.0 to 1.5 s: lick bit high from {first_s:.3f} s for {length_s:.3f} s'
               + ('' if one_run else ', with gaps'))
    return port


def check_indexed_stream(port, args):
    """Op X: each sample comes with its number, and the numbers have no gap while the PC reads."""
    port.write(b'X\x01')
    raw = read_exact(port, 8 * 4000, 5)
    port.write(b'X\x00')
    rest, _ = read_until_quiet(port, 0.3)
    records = np.frombuffer(raw, dtype='<u4').reshape(-1, 2)
    steps = np.diff(records[:, 0].astype(np.int64))
    status = read_status(port)
    ok = (len(records) == 4000 and records[0, 0] == 0 and (steps == 1).all()
          and (records[:, 1] >> 28).max() == 0 and status['nSamplesDropped'] == 0 and status['streaming'] == 0)
    report('indexed stream', 'PASS' if ok else 'FAIL',
           f'{len(records)} samples, numbers {records[0, 0]} to {records[-1, 0]}, '
           f'{int((steps != 1).sum())} gaps, {len(rest)} bytes after the stop')
    return port


def check_buffer_overflow(port, args):
    """When the PC stays away for longer than the device can buffer, the lost samples are counted,
    and the sample numbers of op X show where they were lost."""
    stall_s = SAMPLE_BUFFER_SIZE / SAMPLING_RATE + 4
    port.write(b'X\x01')
    time.sleep(stall_s)
    raw = bytearray()
    end = time.perf_counter() + 2  # Long enough to read everything buffered, and samples taken since
    while time.perf_counter() < end:
        raw += port.read(65536)
    port.write(b'X\x00')
    rest, _ = read_until_quiet(port)
    raw += rest
    status = read_status(port)
    records = np.frombuffer(bytes(raw[:len(raw) // 8 * 8]), dtype='<u4').reshape(-1, 2)
    steps = np.diff(records[:, 0].astype(np.int64))
    n_missing = int((steps - 1).sum())
    ok = (len(raw) % 8 == 0 and status['nSamplesDropped'] > 0 and (steps >= 1).all()
          and n_missing == status['nSamplesDropped'])
    report('lost samples are counted', 'PASS' if ok else 'FAIL',
           f'PC idle for {stall_s:.0f} s: device counted {status["nSamplesDropped"]} lost samples; '
           f'{len(records)} received, sample numbers skip {n_missing} in {int((steps > 1).sum())} gaps')
    return port


def check_truncated_command(port, args):
    """A command cut short must be abandoned, not completed with the next commands' bytes."""
    threshold = set_threshold_below_signal(port)
    time.sleep(0.05)
    before = read_status(port)
    port.write(b'T\x10\x20')  # Op T with 2 of its 4 argument bytes
    time.sleep(0.5)
    n_sent = 0
    reply = b''
    while n_sent < 6 and len(reply) < 4:
        port.write(b'F')
        n_sent += 1
        reply = read_exact(port, 4, 0.3)
    port.write(struct.pack('<BI', ord('T'), threshold))
    time.sleep(0.05)
    port.reset_input_buffer()
    after = read_status(port)
    if n_sent == 1 and after['nCommandErrors'] == before['nCommandErrors'] + 1:
        report('truncated command times out', 'PASS', 'F answered after an incomplete T, which was counted as an error')
    else:
        report('truncated command times out', 'FAIL',
               f'{n_sent - 1} F commands went unanswered after an incomplete T; '
               f'{after["nCommandErrors"] - before["nCommandErrors"]} command errors counted')
    return port


def check_argument_limits(port, args):
    """The device refuses arguments outside its limits, and unknown ops, and counts each refusal."""
    before = read_status(port)
    bad_commands = [struct.pack('<BH', ord('W'), 255), b'D\x00', b'C\x20', b'!\x02',
                    b'I' + struct.pack('<f', 100.0), b'I' + struct.pack('<f', float('nan')), b'E\x02', b'z']
    for command in bad_commands:
        port.write(command)
        time.sleep(0.02)
    time.sleep(0.1)
    refusal = port.read(10)  # Op E replies 0 when it refuses
    after = read_status(port)
    n_errors = after['nCommandErrors'] - before['nCommandErrors']
    unchanged = all(after[name] == before[name] for name in FACTORY_SETTINGS)
    ok = unchanged and n_errors == len(bad_commands) and refusal == b'\x00'
    report('arguments outside the limits are refused', 'PASS' if ok else 'FAIL',
           f'{n_errors} of {len(bad_commands)} bad commands counted, settings '
           + ('unchanged' if unchanged else f'changed: {after}'))
    return port


def check_read_back(port, args):
    """Op G returns what the setting ops programmed."""
    sent = {'threshold': 12345678, 'samplingInterval': 1000.0, 'rCount': 0x0123, 'settleCount': 0x0456,
            'refDivider': 2, 'driveCurrent': 17, 'activeChannel': 1, 'ledEnabled': 1}
    port.write(struct.pack('<BI', ord('T'), sent['threshold']))
    port.write(b'I' + struct.pack('<f', sent['samplingInterval']))
    port.write(struct.pack('<BH', ord('W'), sent['rCount']))
    port.write(struct.pack('<BH', ord('N'), sent['settleCount']))
    port.write(bytes([ord('D'), sent['refDivider'], ord('C'), sent['driveCurrent'], ord('!'), sent['activeChannel'],
                      ord('L'), sent['ledEnabled']]))
    time.sleep(0.1)
    status = read_status(port)
    wrong = {name: status[name] for name in sent if status[name] != sent[name]}
    set_client_defaults(port)
    report('settings read back', 'FAIL' if wrong else 'PASS',
           f'wrong: {wrong}' if wrong else 'op G returned all 8 settings as sent')
    return port


def check_debounce(port, args):
    """The lick state is held for 10 ms after a change, whatever the sampling rate. A threshold
    raised above the signal and lowered again at once makes the shortest lick there is."""
    values, _, _ = read_samples(port, 1000)
    below = struct.pack('<BI', ord('T'), int(values.mean() * 0.99))
    above = struct.pack('<BI', ord('T'), int(values.mean() * 1.01))
    port.write(below)
    failed = []
    details = []
    for rate in (2000, 500):
        port.write(b'I' + struct.pack('<f', 1e6 / rate))
        time.sleep(0.1)
        port.write(struct.pack('<BI', ord('R'), rate))  # 1 s of samples
        time.sleep(0.3)
        port.write(above)
        wait_until = time.perf_counter() + 0.004  # At least one sample must see the raised threshold, and
        while time.perf_counter() < wait_until:   # time.sleep() can take 15 ms on Windows
            pass
        port.write(below)
        _, lick = split(read_exact(port, 4 * rate, 4))
        lick_ms = lick.sum() / rate * 1000
        details.append(f'{lick_ms:.1f} ms at {rate} Hz')
        # The change is seen at one sample, held for the debounce, and seen ended at the next sample
        if not DEBOUNCE_MS <= lick_ms <= DEBOUNCE_MS + 2 * 1000 / rate:
            failed.append(rate)
    port.write(b'I' + struct.pack('<f', 1e6 / SAMPLING_RATE))
    time.sleep(0.1)
    port.reset_input_buffer()
    report('debounce time', 'FAIL' if failed else 'PASS', 'shortest lick: ' + ', '.join(details))
    return port


def check_stream_stops_with_client(port, args):
    """The device must stop streaming when the client closes the port."""
    port_name = port.port
    port.write(b'S\x01')
    time.sleep(0.2)
    port.close()
    time.sleep(1.0)
    port = serial.Serial(port_name, 115200, timeout=0.02)
    time.sleep(0.3)
    n_waiting = port.in_waiting
    port.write(b'F')
    reply = read_exact(port, 4, 1)
    port.write(b'S\x00')
    time.sleep(0.3)
    port.reset_input_buffer()
    if n_waiting == 0 and reply == struct.pack('<I', FIRMWARE_VERSION):
        report('stream stops when the port closes', 'PASS', 'device was quiet on reconnect')
    else:
        report('stream stops when the port closes', 'FAIL',
               f'{n_waiting} bytes waiting on reconnect; the reply to F read as version '
               f'{struct.unpack("<I", reply)[0] if len(reply) == 4 else reply.hex()}')
    return port


def check_reply_during_stream(port, args):
    port.write(b'S\x01')
    data = bytearray()
    for _ in range(100):
        port.write(b'F')
        end = time.perf_counter() + 0.004
        while time.perf_counter() < end:
            data += port.read(65536)
    port.write(b'S\x00')
    rest, _ = read_until_quiet(port, 0.3)
    data += rest
    words = np.frombuffer(bytes(data[:len(data) // 4 * 4]), dtype='<u4')
    n_replies = int((words == FIRMWARE_VERSION).sum())
    ok = n_replies == 100 and len(data) % 4 == 0
    report('replies during streaming', 'PASS' if ok else 'FAIL',
           f'{n_replies} of 100 replies to F found among {len(words) - n_replies} samples')
    return port


def check_interrupt_time(port, args):
    """The sampling interrupt must leave loop() time to run: its longest run is reported by op G."""
    read_status(port)  # Restarts the measurement
    read_samples(port, 2000)
    status = read_status(port)
    ok = status['maxReadCycleMicros'] < 300 and status['nSensorErrors'] == 0
    report('sampling interrupt', 'PASS' if ok else 'FAIL',
           f'longest run {status["maxReadCycleMicros"]} us in 1 s of streaming; '
           f'{status["nSensorErrors"]} I2C errors since power-on')
    return port


def reset_device(port_name):
    """Restarts the device with tycmd, and waits for its port to come back."""
    tycmd = REPO_ROOT / 'Firmware' / 'Compiled' / ('tycmd.exe' if platform.system() == 'Windows' else 'tycmd_linux64')
    done = subprocess.run([str(tycmd), 'reset', '--board', '@' + port_name], capture_output=True, text=True)
    if done.returncode != 0:
        raise RuntimeError('tycmd reset failed: ' + done.stderr.strip())
    time.sleep(1.5)
    for _ in range(40):
        try:
            return open_port(port_name)
        except serial.SerialException:
            time.sleep(0.25)
    raise RuntimeError(f'{port_name} did not come back after the reset')


def check_power_on_state(port, args):
    """The sensor value must have the same scale before and after the first op D. Needs --reset."""
    if not args.reset:
        report('value scale unchanged by op D', 'INFO', 'skipped: needs --reset to start from the power-on state')
        return port
    port_name = port.port
    port.close()
    port = reset_device(port_name)
    before, lick, _ = read_samples(port, 2000)
    port.write(b'D\x01')
    time.sleep(0.2)
    port.reset_input_buffer()
    after, _, _ = read_samples(port, 2000)
    ratio = before.mean() / after.mean()
    report('power-on state', 'INFO',
           f'value {before.mean():.0f}; lick detected in {lick.mean() * 100:.0f}% of samples with the power-on threshold')
    report('value scale unchanged by op D', 'PASS' if abs(ratio - 1) < 0.01 else 'FAIL',
           f'{before.mean():.0f} at power-on, {after.mean():.0f} after D 1 (ratio {ratio:.3f})')
    return port


def check_stored_settings(port, args):
    """Op E: stored settings are in force after a restart, and erased ones are not. Needs --reset."""
    if not args.reset:
        report('stored settings', 'INFO', 'skipped: needs --reset, as it restarts the device twice')
        return port
    port_name = port.port
    stored = {'threshold': 17000000, 'samplingInterval': 1000.0, 'rCount': 512, 'settleCount': 20,
              'refDivider': 2, 'driveCurrent': 30, 'activeChannel': 1, 'ledEnabled': 1}
    port.write(struct.pack('<BI', ord('T'), stored['threshold']))
    port.write(b'I' + struct.pack('<f', stored['samplingInterval']))
    port.write(struct.pack('<BH', ord('W'), stored['rCount']))
    port.write(struct.pack('<BH', ord('N'), stored['settleCount']))
    port.write(bytes([ord('D'), stored['refDivider'], ord('C'), stored['driveCurrent'],
                      ord('!'), stored['activeChannel'], ord('L'), stored['ledEnabled']]))
    time.sleep(0.2)
    port.reset_input_buffer()

    # Store during an acquisition, to see for how long the flash write holds up sampling
    n_samples = 1000  # 1 s at the 1 kHz just set
    start = time.perf_counter()
    port.write(struct.pack('<BI', ord('R'), n_samples))
    time.sleep(0.3)
    port.write(b'E\x01')
    raw = read_exact(port, 4 * n_samples + 1, 5)
    late_ms = (time.perf_counter() - start - n_samples / 1000) * 1000
    acknowledged = len(raw) == 4 * n_samples + 1 and 1 in raw  # The 1-byte reply lies among the sample words
    port.close()

    port = reset_device(port_name)
    status = read_status(port)
    wrong = {name: status[name] for name in stored if status[name] != stored[name]}
    ok = acknowledged and not wrong and status['storedSettingsLoaded'] == 1
    report('stored settings are in force after a restart', 'PASS' if ok else 'FAIL',
           (f'wrong: {wrong}' if wrong else 'all 8 settings as stored')
           + f'; op E {"acknowledged" if acknowledged else "not acknowledged"}, and the 1 s acquisition around it '
             f'ended {late_ms:.0f} ms late')

    port.write(b'E\x00')
    acknowledged = read_exact(port, 1, 1) == b'\x01'
    port.close()
    port = reset_device(port_name)
    status = read_status(port)
    wrong = {name: status[name] for name in FACTORY_SETTINGS if status[name] != FACTORY_SETTINGS[name]}
    ok = acknowledged and not wrong and status['storedSettingsLoaded'] == 0
    report('erased settings give the factory settings', 'PASS' if ok else 'FAIL',
           f'wrong: {wrong}' if wrong else 'all 8 settings at their factory values')
    return port


CHECKS = {
    'handshake': check_handshake,
    'poweron': check_power_on_state,
    'signal': check_signal,
    'falselick': check_setting_ops_false_lick,
    'channel2': check_channel_2_rcount,
    'channel1': check_channel_1_rcount,
    'stall': check_host_stall,
    'indexed': check_indexed_stream,
    'overflow': check_buffer_overflow,
    'truncated': check_truncated_command,
    'limits': check_argument_limits,
    'readback': check_read_back,
    'debounce': check_debounce,
    'reconnect': check_stream_stops_with_client,
    'replies': check_reply_during_stream,
    'interrupt': check_interrupt_time,
    'stored': check_stored_settings,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('port', help='serial port of the device, e.g. COM4 or /dev/ttyACM0')
    parser.add_argument('--reset', action='store_true',
                        help='restart the device for the power-on and stored settings checks, and again at the end')
    parser.add_argument('--only', nargs='+', choices=list(CHECKS), help='run only these checks')
    parser.add_argument('--rcount-trials', type=int, default=100,
                        help='number of W ops in the channel 1 conversion time check (default 100)')
    args = parser.parse_args()

    port = open_port(args.port)
    port_name = port.port
    try:
        for name, check in CHECKS.items():
            if args.only and name not in args.only:
                continue
            if name not in ('handshake', 'poweron'):
                set_client_defaults(port)
            try:
                port = check(port, args)
            except RuntimeError as error:
                report(name, 'FAIL', str(error))
                if not port.is_open:
                    port = open_port(port_name)
        set_client_defaults(port)
        set_threshold_below_signal(port)
    finally:
        port.close()
    if args.reset:
        reset_device(args.port).close()

    n_failed = sum(1 for _, outcome, _ in results if outcome == 'FAIL')
    print(f'\n{len(results)} results, {n_failed} failed')
    return n_failed


if __name__ == '__main__':
    sys.exit(main())
