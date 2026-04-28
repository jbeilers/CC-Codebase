"""
serial_logger.py

Serial terminal for Teensy hardware and simulation tests.
Automatically detects the active test from the CSV header and saves
data to a named .txt file each time a move completes.

Supported tests
---------------
  test_control_sim.cpp   — simulation (G / R / S / Z / ?)
  test_control_hw.cpp    — hardware  (H / G / U / D / B / R / C / S / ?)

Output files (written to the same directory as this script)
-----------------------------------------------------------
  sim_data.txt          — simulation run  (ref_x present in header)
  hw_gantry_data.txt    — hardware gantry-only run  ('G' command)
  hw_servo_data.txt     — hardware servo-only run   ('U' / 'D' command)
  hw_combined_data.txt  — hardware combined run     ('B' command)

SETUP
-----
    pip install pyserial

USAGE
-----
    python serial_logger.py

    Type a command and press Enter.  Send '?' to the Teensy to see
    available commands for the currently loaded test.
    The logger reconnects automatically if the Teensy resets.
"""

import serial
import serial.tools.list_ports
import threading
import time
import sys
import os
from typing import Optional

# ----------------------------------------------------------------
#  Configuration — update PORT if needed
# ----------------------------------------------------------------
PORT = 'COM4'   # Teensy USB serial port (check Device Manager)
BAUD = 115200

# Output directory — folder that contains this script
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

# ----------------------------------------------------------------
#  Shared serial handle (accessed by both main thread and reader)
# ----------------------------------------------------------------
_ser_lock = threading.Lock()
_ser: Optional[serial.Serial] = None  # Optional avoids Python <3.10 TypeError with '|' syntax


def _get_ser():
    with _ser_lock:
        return _ser


def _set_ser(s):
    global _ser
    with _ser_lock:
        _ser = s

# ----------------------------------------------------------------
#  Connection helpers
# ----------------------------------------------------------------
def _try_open(port: str) -> bool:
    """Attempt to open the port once. Returns True on success."""
    try:
        s = serial.Serial(port, BAUD, timeout=0.1)
        _set_ser(s)
        return True
    except serial.SerialException:
        return False


def _reconnect():
    """Close the stale handle and block until the port reopens."""
    old = _get_ser()
    _set_ser(None)
    try:
        if old:
            old.close()
    except Exception:
        pass

    print(f'\n[logger] Disconnected — waiting for {_port_name} to come back...')
    while not _try_open(_port_name):
        time.sleep(0.5)
    time.sleep(1.5)  # Let Teensy finish boot/setup before reading
    print(f'[logger] Reconnected to {_port_name}.\n')

# ----------------------------------------------------------------
#  CSV header → output filename mapping
# ----------------------------------------------------------------
def _output_filename(header: str) -> str:
    has_ref_x  = 'ref_x'      in header
    has_gantry = 'est_x'      in header
    has_servo  = 'target_deg' in header

    if has_ref_x:
        name = 'sim_data.txt'
    elif has_gantry and has_servo:
        name = 'hw_combined_data.txt'
    elif has_gantry:
        name = 'hw_gantry_data.txt'
    elif has_servo:
        name = 'hw_servo_data.txt'
    else:
        name = 'test_data.txt'

    return os.path.join(OUTPUT_DIR, name)

# ----------------------------------------------------------------
#  Done-marker detection
#
#  Servo-only CSV (no gantry columns): close on any servo completion.
#  Gantry or combined CSV: close on "# Gantry done." only — in
#  combined mode the servo finishes first but gantry settling data
#  follows, so we must wait for the gantry marker.
# ----------------------------------------------------------------
_SERVO_DONE  = ('# Servo PASS', '# Servo FAIL', '# Servo TIMEOUT')
_GANTRY_DONE = ('# Done.', '# Gantry done.', '# FAULT')


def _is_done(line: str, csv_has_gantry: bool) -> bool:
    if not csv_has_gantry:
        return any(m in line for m in _SERVO_DONE)
    return any(m in line for m in _GANTRY_DONE)

# ----------------------------------------------------------------
#  Serial reader — background thread
# ----------------------------------------------------------------
csv_file       = None
current_file   = ''
recording      = False
csv_has_gantry = False
data_lock      = threading.Lock()


def read_loop():
    global csv_file, current_file, recording, csv_has_gantry

    while True:
        s = _get_ser()
        if s is None:
            time.sleep(0.05)
            continue

        try:
            raw = s.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='replace').rstrip()
        except Exception as e:
            print(f'\n[logger] Read error: {e}')
            _reconnect()
            continue   # resume reading after reconnect — do NOT break

        if not line:
            continue

        # Always echo to terminal
        print(line)

        with data_lock:
            # CSV header → open a new output file
            if line.startswith('t_ms,'):
                if csv_file:
                    csv_file.close()

                current_file   = _output_filename(line)
                csv_has_gantry = 'est_x' in line

                os.makedirs(os.path.dirname(current_file), exist_ok=True)
                csv_file  = open(current_file, 'w', buffering=1)
                csv_file.write(line + '\n')
                recording = True
                print(f'[logger] Recording → {current_file}')
                continue

            if recording:
                if not line.startswith('#'):
                    csv_file.write(line + '\n')

                if _is_done(line, csv_has_gantry):
                    recording = False
                    csv_file.close()
                    csv_file  = None
                    print(f'[logger] Saved → {current_file}')

# ----------------------------------------------------------------
#  Auto-detect port
# ----------------------------------------------------------------
def find_teensy_port(preferred: str) -> str:
    ports = serial.tools.list_ports.comports()
    names = [p.device for p in ports]
    if preferred in names:
        return preferred
    if ports:
        fallback = ports[0].device
        print(f'[logger] {preferred} not found. Using {fallback}.')
        print(f'[logger] Available: {names}')
        return fallback
    raise RuntimeError('No serial ports found. Is the Teensy plugged in?')

# ----------------------------------------------------------------
#  Main
# ----------------------------------------------------------------
if __name__ == '__main__':
    # global _port_name
    _port_name = find_teensy_port(PORT)

    if not _try_open(_port_name):
        print(f'[logger] Could not open {_port_name}.')
        sys.exit(1)

    print(f'[logger] Connected to {_port_name} at {BAUD} baud.')
    print(f'[logger] Send ? to the Teensy to see available commands.')
    print(f'[logger] CSV is saved automatically when a move completes.')
    print(f'[logger] Reconnects automatically if the Teensy resets.')
    print(f'[logger] Ctrl+C to exit.\n')

    reader = threading.Thread(target=read_loop, daemon=True)
    reader.start()

    try:
        while True:
            cmd = input()
            s = _get_ser()
            if s is None:
                print('[logger] Not connected — command discarded.')
                continue
            try:
                for ch in cmd:
                    s.write(ch.encode())
                s.write(b'\n')   # Teensy ignores the newline
                if cmd:
                    print(f'[logger] >>> {cmd}')
            except Exception as e:
                print(f'[logger] Write error: {e}')
    except KeyboardInterrupt:
        print('\n[logger] Exiting.')
    finally:
        with data_lock:
            if csv_file:
                csv_file.close()
        s = _get_ser()
        if s:
            s.close()
