"""Test of the Python class (lickstick.py) on a connected device. Nothing needs to touch the sensor.

    python tests/lickstick_class_test.py COM4
    python tests/lickstick_class_test.py COM4 --save-plot plot.png    # also saves the live plot's figure

It connects as a user's script would, and opens the streaming window three times for a few seconds
each, pressing its buttons from the code. Each check prints PASS, FAIL or INFO. The exit code is the
number of failed checks. It works with both builds of firmware v2 (/AGENTS.md, "Two builds of v2");
what it cannot verify without op G it reports as INFO.

It does not judge the window by eye: whether the plot looks right, and whether a lick shows, needs
a person. It leaves the device with the class's defaults, and the threshold 18,200,000.

Requires pyserial, numpy and matplotlib. Keep the drink tube still while it runs.
"""
import argparse
import os
import struct
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lickstick import LickStick  # noqa: E402

# Reply to op G (/Firmware/PROTOCOL.md), up to the settings
SETTINGS_FORMAT = '<IfHHBBBB'
SETTINGS_FIELDS = ('threshold', 'samplingInterval', 'rCount', 'settleCount', 'refDivider', 'driveCurrent',
                   'activeChannel', 'ledEnabled')
STATUS_SIZE = 32
CLASS_DEFAULTS = {'threshold': 18200000, 'samplingInterval': 500.0, 'rCount': 256, 'settleCount': 10,
                  'refDivider': 1, 'driveCurrent': 31, 'activeChannel': 0, 'ledEnabled': 0}

results = []  # (name, outcome, detail)


def report(name, outcome, detail):
    results.append((name, outcome, detail))
    print(f'{outcome:4s}  {name}: {detail}', flush=True)


def read_settings(L):
    """The device's settings, read back with op G. None if the build does not answer it."""
    L.Port.write(b'G')
    timeout = L._readTimeout
    L._readTimeout = 0.5
    try:
        reply = L._read_exact(STATUS_SIZE)
    except TimeoutError:
        return None
    finally:
        L._readTimeout = timeout
    return dict(zip(SETTINGS_FIELDS, struct.unpack_from(SETTINGS_FORMAT, reply)))


def plausible(values):
    """True if every value is near the others: a reading of the resting sensor, with no byte out of place."""
    values = np.asarray(values, dtype=float)
    middle = np.median(values)
    return len(values) > 0 and middle > 1e6 and bool(np.all(np.abs(values - middle) < 0.05 * middle))


def run(L, seconds):
    """What a protocol's loop does after stream(block=False)."""
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        L.update()
        time.sleep(0.01)


def spy_on_writes(L):
    """Notes every command the class sends from here on. Returns the list they are added to."""
    written = []
    write = L.Port.write

    def spy(data):
        written.append(bytes(data))
        return write(data)
    L.Port.write = spy
    return written


def time_start_of_stream(L):
    """Makes startAcq() note when it ran. Returns the list the times are added to."""
    times = []
    startAcq = L.startAcq

    def timed():
        times.append(time.perf_counter())
        startAcq()
    L.startAcq = timed
    return times


def check_connect(port_name, args):
    """The constructor programs the device with the class's defaults (known issue 9)."""
    L = LickStick(port_name)
    L.rCount = 300
    L.driveCurrent = 20
    L.threshold = 5
    L.Port.close()
    L = LickStick(port_name)
    time.sleep(0.1)
    settings = read_settings(L)
    if settings is None:
        report('connect', 'INFO', 'connected; this firmware build does not answer op G, so the settings '
                                  'on the device could not be read back')
    elif settings == CLASS_DEFAULTS:
        report('connect', 'PASS', 'a new object left the class defaults on a device that held others')
    else:
        wrong = {k: v for k, v in settings.items() if v != CLASS_DEFAULTS[k]}
        report('connect', 'FAIL', f'after connecting, the device held {wrong}')
    return L


def check_read_timeout(L, args):
    """A read gives up when the device does not answer, instead of waiting for ever (known issue 9)."""
    timeout = L._readTimeout
    L._readTimeout = 0.3
    start = time.perf_counter()
    try:
        L._read_exact(4)
        report('timeout', 'FAIL', 'a read of bytes that were never sent returned')
    except TimeoutError:
        waited = time.perf_counter() - start
        report('timeout', 'PASS' if waited < 1 else 'FAIL', f'a read of bytes that were never sent gave up after {waited:.2f} s')
    finally:
        L._readTimeout = timeout


def check_long_read(L, args):
    """A readSensor() that takes longer than the read timeout is not cut off."""
    nSamples = int(1.3 * L._readTimeout * L.samplingRate)
    try:
        values = L.readSensor(nSamples)
    except TimeoutError as e:
        report('longread', 'FAIL', f'readSensor({nSamples}) timed out: {e}')
        return
    ok = len(values) == nSamples and plausible(values)
    report('longread', 'PASS' if ok else 'FAIL', f'readSensor({nSamples}) returned {len(values)} values')


def check_left_streaming(L, port_name, args):
    """A device that an earlier session left streaming is stopped by the handshake (known issue 6)."""
    L.Port.write(b'S\x01')
    time.sleep(0.3)
    try:
        L._handshake()
        values = L.readSensor(100)
        ok = plausible(values)
        report('handshake', 'PASS' if ok else 'FAIL',
               'the handshake stopped a running stream, and the next read was ' + ('right' if ok else 'wrong'))
    except (ValueError, TimeoutError) as e:
        report('handshake', 'FAIL', f'the handshake failed on a streaming device: {e}')
        L.Port.write(b'S\x00')
        time.sleep(0.3)
        L.Port.reset_input_buffer()
    # The same, as it happens in use: a session ends while streaming, and a new one connects
    L.Port.write(b'S\x01')
    time.sleep(0.1)
    L.Port.close()
    try:
        L = LickStick(port_name)
        ok = plausible(L.readSensor(100))
        report('reconnect', 'PASS' if ok else 'FAIL', 'connected after a session that ended while streaming')
    except (ValueError, TimeoutError) as e:
        report('reconnect', 'FAIL', f'could not connect after a session that ended while streaming: {e}')
        raise
    return L


def check_threshold_values(L, args):
    """Thresholds that cannot be sent are refused or limited, not a struct.error (known issue 9)."""
    problems = []
    if L._thresholdFromSamples([100, 1000]) != 0:
        problems.append('a negative automatic threshold is not limited to 0')
    for bad in (-1, 2 ** 32):
        try:
            L.threshold = bad
            problems.append(f'{bad} was accepted')
        except ValueError:
            pass
    L.threshold = 18200000.0  # A float is rounded
    if L.threshold != 18200000 or not isinstance(L.threshold, int):
        problems.append('a float threshold was not rounded')
    report('threshold', 'FAIL' if problems else 'PASS', '; '.join(problems) or 'out-of-range thresholds are refused, '
           'and the automatic one stops at 0')


def check_stream_alongside(L, args):
    """stream(block=False) records while the caller's code runs, and the buttons do not pause it
    (known issues 8 and 9)."""
    os.chdir(tempfile.gettempdir())  # stream() must not need blank.ico in the working directory
    L._nSamplesPerAllocation = 3000  # So that the data vectors must grow during the test
    written = spy_on_writes(L)
    started = time_start_of_stream(L)
    problems = []
    try:
        L.stream(block=False)
        run(L, 1.0)
        del written[:]
        thresholdBefore = L.threshold
        for i in range(10):
            L.gui['threshPlusBtn'].invoke()
            run(L, 0.03)
        if L.threshold != thresholdBefore + 20000:
            problems.append(f'10 presses of + moved the threshold by {L.threshold - thresholdBefore}')
        for i in range(10):
            L.gui['threshMinusBtn'].invoke()
            run(L, 0.03)
        L.gui['autoThreshButton'].invoke()
        run(L, 1.0)
        nNow = L.acquiredData['nSamples']
        resting = L.acquiredData['Sensor'][nNow - 1000:nNow]
        autoThreshold = L.threshold
        if not 0.98 * resting.min() < autoThreshold < resting.min():
            problems.append(f'AutoSet gave {autoThreshold} with the signal at {resting.min()}')
        if L.acquiredData['Params']['threshold'] != autoThreshold or L.gui['thresholdVar'].get() != str(autoThreshold):
            problems.append('AutoSet did not update the Thresh box or Params')
        # A threshold above the signal, typed into the box, makes a lick in the recording
        L.gui['thresholdVar'].set('4294967295')
        L.uiSetThreshold()
        run(L, 0.3)
        L.gui['thresholdVar'].set(str(autoThreshold))
        L.uiSetThreshold()
        run(L, 0.3)
        L.gui['thresholdVar'].set('not a number')
        L.uiSetThreshold()
        if L.threshold != autoThreshold or L.gui['thresholdVar'].get() != str(autoThreshold):
            problems.append('text that is not a number changed the threshold')
        ops = sorted({chr(w[0]) for w in written})
        if ops != ['T']:
            problems.append(f'the buttons sent ops {ops}, not only T')
        plotted = int(np.count_nonzero(~np.isnan(L.gui['OscopeDataLine'].get_ydata())))
        if args.save_plot:
            L.gui['canvas'].figure.savefig(args.save_plot)
        time.sleep(1.5)  # The protocol is busy: nothing calls update()
        try:
            L.rCount = 512
            problems.append('rCount could be set while streaming')
        except ValueError:
            pass
        stopped = time.perf_counter()
    finally:
        L.endAcq()
        L._nSamplesPerAllocation = 7200000
    data = L.acquiredData
    nSamples = data['nSamples']
    expected = (stopped - started[0]) * L.samplingRate
    if len(data['Sensor']) != nSamples or len(data['TTL']) != nSamples:
        problems.append('Sensor and TTL were not trimmed to nSamples')
    if abs(nSamples - expected) > 0.01 * expected + 100:
        problems.append(f'{nSamples} samples recorded, {expected:.0f} expected')
    if not plausible(data['Sensor']):
        problems.append('the recording holds values that are not sensor readings')
    # Before AutoSet the class's default threshold may be above the signal (a lick throughout): count from there
    nLick = int(data['TTL'][nNow:].sum())
    if not 0.2 * L.samplingRate < nLick < 1.0 * L.samplingRate:
        problems.append(f'TTL is high for {nLick} samples, where a threshold above the signal for 0.3 s was expected')
    if plotted == 0:
        problems.append('nothing was plotted')
    if not plausible(L.readSensor(50)):
        problems.append('readSensor() after the window closed returned wrong values')
    report('stream', 'FAIL' if problems else 'PASS', '; '.join(problems) or
           f'{nSamples} samples in {stopped - started[0]:.1f} s ({expected:.0f} expected) through 21 button presses '
           f'and 1.5 s without update(); the buttons sent only op T; a lick of {nLick} samples was recorded')


def check_clear(L, args):
    """clearAcquiredData() restarts the recording during a stream, and stream() can be called again."""
    problems = []
    try:
        L.stream(block=False)
        L.stream(block=False)  # A second call must not open a second window, or restart the recording
        run(L, 1.0)
        nBefore = L.acquiredData['nSamples']
        L.clearAcquiredData()
        nCleared = L.acquiredData['nSamples']
        run(L, 0.5)
        nAfter = L.acquiredData['nSamples']
    finally:
        L.endAcq()
    L.update()  # Must do nothing, now that the window is closed
    if nBefore < 0.5 * L.samplingRate:
        problems.append(f'only {nBefore} samples in the first second')
    if nCleared > 0.1 * L.samplingRate or not 0.3 * L.samplingRate < nAfter < 0.8 * L.samplingRate:
        problems.append(f'{nCleared} samples right after clearing, {nAfter} half a second later')
    if not plausible(L.acquiredData['Sensor']):
        problems.append('the recording holds values that are not sensor readings')
    report('clear', 'FAIL' if problems else 'PASS', '; '.join(problems) or
           f'{nBefore} samples before clearAcquiredData(), {nAfter} half a second after')


def check_stream_blocking(L, args):
    """stream() with no argument still returns only when the window closes."""
    startAcq = L.startAcq

    def start_then_close_later():
        startAcq()
        L.gui['Fig'].after(1500, L.endAcq)  # Stands in for the user closing the window
    L.startAcq = start_then_close_later
    start = time.perf_counter()
    try:
        L.stream()
    finally:
        L.startAcq = startAcq
    took = time.perf_counter() - start
    nSamples = L.acquiredData['nSamples']
    ok = took > 1.5 and 0.8 * 1.5 * L.samplingRate < nSamples < 1.3 * 1.5 * L.samplingRate \
        and len(L.acquiredData['Sensor']) == nSamples and plausible(L.acquiredData['Sensor'])
    report('blocking', 'PASS' if ok else 'FAIL', f'stream() returned after {took:.1f} s with {nSamples} samples, '
           'when its window was closed after 1.5 s')


def main():
    parser = argparse.ArgumentParser(description='Test of the LickStick Python class on a connected device')
    parser.add_argument('port', help='serial port of the device, e.g. COM4 or /dev/ttyACM0')
    parser.add_argument('--save-plot', metavar='FILE', help='save the figure of the live plot to this image file')
    args = parser.parse_args()
    if args.save_plot:
        args.save_plot = os.path.abspath(args.save_plot)

    L = check_connect(args.port, args)
    check_read_timeout(L, args)
    check_long_read(L, args)
    L = check_left_streaming(L, args.port, args)
    check_threshold_values(L, args)
    check_stream_alongside(L, args)
    check_clear(L, args)
    check_stream_blocking(L, args)
    L.Port.close()

    nFailed = sum(1 for r in results if r[1] == 'FAIL')
    print(f'\n{len(results)} results, {nFailed} failed')
    return nFailed


if __name__ == '__main__':
    sys.exit(main())
