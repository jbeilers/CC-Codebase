"""
camera_burst.py
===============
Capture and replay tool for the TSL1401 CCD burst mode in test_demo.cpp.

WORKFLOW
--------
1. Run this script — it owns the serial port, so close any other serial
   monitor (VS Code, PlatformIO, etc.) before starting.
2. Switch the Teensy to mode 5 (Camera) by typing  5  in this console.
   All keystrokes you type are forwarded to the Teensy in real time.
3. Press  B  to start a burst (default 5 s, ~50 frames at 10 Hz).
4. The script captures the F: stream, saves to camera_burst.csv, and
   immediately starts the looping animation in the plot window.
5. Press  B  again at any time for a fresh burst (overwrites the CSV).

INTEGRATED TERMINAL
-------------------
This script acts as a serial terminal while it runs.
Click the console window and type single-character commands exactly as you
would in VS Code's serial monitor:

    ]  [       cycle demo mode forward / backward
    1–8        jump to a specific demo mode
    ?          print help for the current mode
    B          (in camera mode) start a burst capture
    W A S D    (in gantry mode) move
    X          stop gantry / coast DC motors
    ... etc.

On Windows the console must be the focused window for keystrokes to register.
The matplotlib plot window and the console window can both be open at once.

SETUP
-----
    pip install pyserial matplotlib numpy

USAGE
-----
    python camera_burst.py [--port COM4] [--baud 115200] [--fps 10]

    --port   Serial port (default COM4; auto-detected if not found)
    --baud   Baud rate   (default 115200)
    --fps    Playback speed in frames-per-second (default 10)
    --output Path to save CSV (default: camera_burst.csv next to this script)

STREAM FORMAT
-------------
    F:{ms},{clk_us},{center},{left},{right},{threshold},{detected},{p0},...,{p127}
    All pixel values are 8-bit (10-bit ADC >> 2, range 0–255).

NOTES
-----
- The CSV file is overwritten after every burst.
- The animation loops the captured frames continuously at --fps.
- Lines starting with '#' from the Teensy are echoed to the console.
- A status bar below the plot shows how many frames are loaded and the
  time elapsed in the current replay cycle.
"""

import argparse
import csv
import os
import platform
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports

# Select a GUI backend before importing pyplot.
# TkAgg is the most portable on Windows; fall back through common options.
import matplotlib
for _backend in ('TkAgg', 'Qt5Agg', 'WxAgg', 'Agg'):
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        pass

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches

# ---------------------------------------------------------------------------
#  Constants
# ---------------------------------------------------------------------------
PIXEL_COUNT   = 128
DEFAULT_PORT  = 'COM4'
DEFAULT_BAUD  = 115200
DEFAULT_FPS   = 10
SCRIPT_DIR    = Path(__file__).parent
DEFAULT_CSV   = str(SCRIPT_DIR / 'camera_burst.csv')

# ---------------------------------------------------------------------------
#  Shared state  (written by reader thread, read by animation)
# ---------------------------------------------------------------------------
_lock = threading.Lock()

# Frames loaded into the animator (list of dicts)
_play_frames: list   = []
# Flag: set by reader thread when a new burst CSV has been written
_new_burst_ready     = False
# Frames being collected in the current in-progress burst
_collecting          = False
_current_burst: list = []
# Stats for console
_total_bursts        = 0

_stop_event = threading.Event()
_ser        = None   # serial.Serial, kept alive across reconnects
_ani        = None   # FuncAnimation — held here to prevent GC-triggered instant exit


# ---------------------------------------------------------------------------
#  Frame parser
# ---------------------------------------------------------------------------
def _parse_frame(line: str) -> dict | None:
    """Parse one F: stream line.  Returns a dict or None on parse error."""
    try:
        body  = line[2:]            # strip leading "F:"
        parts = body.split(',')
        if len(parts) != 7 + PIXEL_COUNT:
            return None
        return {
            'ms':        int(parts[0]),
            'clk_us':    int(parts[1]),
            'center':    int(parts[2]),
            'left':      int(parts[3]),
            'right':     int(parts[4]),
            'threshold': int(parts[5]),
            'detected':  int(parts[6]) == 1,
            'pixels':    np.array([int(p) for p in parts[7:]], dtype=np.uint8),
        }
    except (ValueError, IndexError):
        return None


# ---------------------------------------------------------------------------
#  CSV save / load
# ---------------------------------------------------------------------------
def _save_csv(frames: list, path: str) -> None:
    header = (['ms', 'clk_us', 'center', 'left', 'right', 'threshold', 'detected']
              + [f'p{i}' for i in range(PIXEL_COUNT)])
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        for fr in frames:
            w.writerow([
                fr['ms'], fr['clk_us'], fr['center'],
                fr['left'], fr['right'], fr['threshold'],
                int(fr['detected']),
            ] + fr['pixels'].tolist())
    print(f'[capture] Saved {len(frames)} frames → {path}')


def load_csv(path: str) -> list:
    """Load a previously saved burst CSV.  Returns list of frame dicts."""
    frames = []
    try:
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                frames.append({
                    'ms':        int(row['ms']),
                    'clk_us':    int(row['clk_us']),
                    'center':    int(row['center']),
                    'left':      int(row['left']),
                    'right':     int(row['right']),
                    'threshold': int(row['threshold']),
                    'detected':  int(row['detected']) == 1,
                    'pixels':    np.array(
                        [int(row[f'p{i}']) for i in range(PIXEL_COUNT)],
                        dtype=np.uint8
                    ),
                })
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f'[load] Error reading {path}: {e}')
    return frames


# ---------------------------------------------------------------------------
#  Serial reader thread
# ---------------------------------------------------------------------------
def _reader_thread(port: str, baud: int, csv_path: str) -> None:
    global _collecting, _current_burst, _play_frames, _new_burst_ready
    global _total_bursts, _ser

    while not _stop_event.is_set():
        # --- Connect ---
        try:
            s = serial.Serial(port, baud, timeout=0.5)
            _ser = s
            print(f'[serial] Connected to {port} at {baud} baud.')
            time.sleep(1.0)
        except serial.SerialException as exc:
            print(f'[serial] Cannot open {port}: {exc}  — retrying in 2 s')
            time.sleep(2.0)
            continue

        # --- Read loop ---
        try:
            while not _stop_event.is_set():
                raw = s.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='replace').rstrip()

                if 'BURST_START' in line:
                    with _lock:
                        _collecting    = True
                        _current_burst = []
                    print('[capture] Burst started — collecting frames...')

                elif 'BURST_END' in line:
                    with _lock:
                        frames       = list(_current_burst)
                        _collecting  = False
                    if frames:
                        _save_csv(frames, csv_path)
                        with _lock:
                            _play_frames      = frames
                            _new_burst_ready  = True
                            _total_bursts    += 1
                        dur_s = (frames[-1]['ms'] - frames[0]['ms']) / 1000.0
                        print(f'[capture] Burst {_total_bursts}: '
                              f'{len(frames)} frames, {dur_s:.1f} s')
                    else:
                        print('[capture] Burst ended but no frames received.')

                elif line.startswith('F:'):
                    frame = _parse_frame(line)
                    if frame:
                        with _lock:
                            if _collecting:
                                _current_burst.append(frame)

                else:
                    # Echo Teensy comments / status to console
                    if line:
                        print(f'  {line}')

        except serial.SerialException as exc:
            print(f'[serial] Connection lost: {exc}  — reconnecting...')
            try:
                s.close()
            except Exception:
                pass
            _ser = None
            time.sleep(1.0)


# ---------------------------------------------------------------------------
#  Plot construction
# ---------------------------------------------------------------------------
def build_plot(fps: int):
    """Build and return (fig, axes_dict, artists_dict)."""
    fig = plt.figure(figsize=(12, 7))
    fig.patch.set_facecolor('#1a1a2e')

    # Layout: waveform (top, tall) + centre history (bottom-left) + status bar
    gs = fig.add_gridspec(
        3, 2,
        height_ratios=[3.5, 2, 0.4],
        width_ratios=[1.5, 1],
        hspace=0.55, wspace=0.35,
        left=0.07, right=0.97, top=0.92, bottom=0.06,
    )

    ax_wave   = fig.add_subplot(gs[0, :])   # full-width waveform
    ax_ctr    = fig.add_subplot(gs[1, 0])   # centre history
    ax_width  = fig.add_subplot(gs[1, 1])   # width history
    ax_status = fig.add_subplot(gs[2, :])   # status bar

    _style_ax(ax_wave,   'Pixel index', 'Intensity (8-bit)',
              xlim=(0, PIXEL_COUNT - 1), ylim=(0, 265))
    _style_ax(ax_ctr,    'Frame', 'Centre pixel',
              xlim=(0, 1), ylim=(0, PIXEL_COUNT - 1),
              title='Centre position history')
    _style_ax(ax_width,  'Frame', 'Width (px)',
              xlim=(0, 1), ylim=(0, PIXEL_COUNT),
              title='Object width history')

    # Status bar — hide axes decorations, just text
    ax_status.set_facecolor('#0f0f23')
    for sp in ax_status.spines.values():
        sp.set_visible(False)
    ax_status.set_xticks([])
    ax_status.set_yticks([])

    px_x  = np.arange(PIXEL_COUNT)
    zeros = np.zeros(PIXEL_COUNT)

    # ---- Waveform panel ----
    wf_fill, = ax_wave.fill(
        np.concatenate([[0], px_x, [PIXEL_COUNT - 1]]),
        np.concatenate([[0], zeros, [0]]),
        color='#4a9eff', alpha=0.3, zorder=2
    )
    wf_line, = ax_wave.plot(px_x, zeros,
                            color='#74baff', linewidth=1.4, zorder=3)
    thr_line  = ax_wave.axhline(0, color='#ff6b6b', linewidth=1.1,
                                linestyle='--', zorder=4, label='Threshold')
    ctr_line  = ax_wave.axvline(64, color='#6bffb8', linewidth=2.0,
                                linestyle='-', zorder=5, label='Centre')
    edge_span = ax_wave.axvspan(0, 1, alpha=0.18, color='#ffd700', zorder=1)

    ax_wave.legend(
        handles=[
            mpatches.Patch(color='#74baff',  label='Pixel intensity'),
            mpatches.Patch(color='#ff6b6b',  label='Threshold'),
            mpatches.Patch(color='#ffd700',  alpha=0.4, label='Object region'),
            mpatches.Patch(color='#6bffb8',  label='Centre'),
        ],
        loc='upper right', fontsize=7,
        facecolor='#2a2a4a', edgecolor='#444466', labelcolor='#ccccdd'
    )

    # ---- Centre history panel ----
    ax_ctr.axhline(PIXEL_COUNT // 2, color='#444466',
                   linewidth=0.8, linestyle=':')
    ctr_det, = ax_ctr.plot([], [], 'o', ms=3, color='#6bffb8',
                           zorder=3, label='Detected')
    ctr_los, = ax_ctr.plot([], [], 'o', ms=3, color='#555566',
                           zorder=2, label='Not detected')
    ax_ctr.legend(loc='upper right', fontsize=7,
                  facecolor='#2a2a4a', edgecolor='#444466',
                  labelcolor='#ccccdd', markerscale=1.5)

    # ---- Width history panel ----
    wid_line, = ax_width.plot([], [], color='#ffd700', linewidth=1.2)
    wid_fill  = ax_width.fill_between(
        [], [], 0, color='#ffd700', alpha=0.3
    )

    # ---- Status text ----
    status_txt = ax_status.text(
        0.01, 0.5, 'Waiting for burst… press B in camera mode (mode 5)',
        transform=ax_status.transAxes,
        color='#aaaacc', fontsize=8.5, va='center', ha='left',
        family='monospace'
    )

    artists = dict(
        wf_fill=wf_fill, wf_line=wf_line,
        thr_line=thr_line, ctr_line=ctr_line, edge_span=edge_span,
        ctr_det=ctr_det, ctr_los=ctr_los,
        wid_line=wid_line, wid_fill=wid_fill,
        status_txt=status_txt,
        ax_wave=ax_wave, ax_ctr=ax_ctr, ax_width=ax_width,
        px_x=px_x,
    )
    return fig, artists


def _style_ax(ax, xlabel='', ylabel='', xlim=None, ylim=None, title=''):
    ax.set_facecolor('#0d0d1a')
    if xlim:
        ax.set_xlim(*xlim)
    if ylim:
        ax.set_ylim(*ylim)
    ax.set_xlabel(xlabel, color='#9999bb', fontsize=8)
    ax.set_ylabel(ylabel, color='#9999bb', fontsize=8)
    if title:
        ax.set_title(title, color='#bbbbdd', fontsize=8.5)
    ax.tick_params(colors='#7777aa', labelsize=7)
    for sp in ax.spines.values():
        sp.set_color('#333355')
    ax.grid(True, color='#1e1e36', linewidth=0.5)


# ---------------------------------------------------------------------------
#  Animation update function factory
# ---------------------------------------------------------------------------
def make_update(artists: dict, fps: int):
    # Mutable state inside closure
    state = {
        'idx':        0,          # current frame index in _play_frames
        'frames':     [],         # local copy of frames being replayed
        'play_start': time.monotonic(),
    }

    def update(_tick):
        global _new_burst_ready

        # Pull latest state from shared memory
        with _lock:
            new_burst = _new_burst_ready
            if new_burst:
                _new_burst_ready   = False
                state['frames']    = list(_play_frames)
                state['idx']       = 0
                state['play_start'] = time.monotonic()

        frames = state['frames']
        if not frames:
            return []

        n   = len(frames)
        idx = state['idx'] % n
        fr  = frames[idx]
        state['idx'] += 1

        px_x      = artists['px_x']
        pixels    = fr['pixels'].astype(float)
        threshold = float(fr['threshold'])
        center    = fr['center']
        left      = fr['left']
        right     = fr['right']
        detected  = fr['detected']

        # ---- Waveform ----
        verts = np.column_stack([
            np.concatenate([[px_x[0]], px_x, [px_x[-1]]]),
            np.concatenate([[0.0],    pixels,  [0.0]])
        ])
        artists['wf_fill'].set_xy(verts)
        artists['wf_line'].set_ydata(pixels)
        artists['thr_line'].set_ydata([threshold, threshold])
        artists['ctr_line'].set_xdata([center, center])
        artists['ctr_line'].set_color('#6bffb8' if detected else '#888888')

        # Edge span
        artists['edge_span'].remove()
        artists['edge_span'] = artists['ax_wave'].axvspan(
            left, right, alpha=0.18, color='#ffd700', zorder=1
        )

        # Title
        width_px = right - left if detected else 0
        det_str  = (f'DETECTED  centre={center}  width={width_px}px  thr={threshold}'
                    if detected else f'NO TARGET  thr={threshold}')
        loop_num = state['idx'] // n + 1
        artists['ax_wave'].set_title(
            f'Frame {idx + 1}/{n}  |  {fr["ms"]} ms  |  CLK ½T={fr["clk_us"]}µs  |  {det_str}'
            f'  [loop {loop_num}]',
            color='#6bffb8' if detected else '#ff6b6b',
            fontsize=8.5
        )

        # ---- Centre + width histories (all frames in burst) ----
        xs     = np.arange(n)
        ctrs   = np.array([f['center']            for f in frames])
        dets   = np.array([f['detected']           for f in frames], dtype=bool)
        widths = np.array([f['right'] - f['left'] if f['detected'] else 0
                           for f in frames], dtype=float)

        # Highlight current frame with a vertical marker
        ax_c = artists['ax_ctr']
        ax_c.set_xlim(-0.5, n - 0.5)

        if dets.any():
            artists['ctr_det'].set_data(xs[dets], ctrs[dets])
        else:
            artists['ctr_det'].set_data([], [])

        if (~dets).any():
            artists['ctr_los'].set_data(xs[~dets], ctrs[~dets])
        else:
            artists['ctr_los'].set_data([], [])

        # Width history
        ax_w = artists['ax_width']
        ax_w.set_xlim(-0.5, n - 0.5)
        artists['wid_line'].set_data(xs, widths)

        artists['wid_fill'].remove()
        artists['wid_fill'] = ax_w.fill_between(
            xs, 0, widths, color='#ffd700', alpha=0.3
        )

        # ---- Status bar ----
        elapsed = time.monotonic() - state['play_start']
        artists['status_txt'].set_text(
            f'Burst: {n} frames  |  '
            f'Replay: frame {idx + 1}/{n}  |  '
            f'Elapsed: {elapsed:.1f} s  |  '
            f'Press B on Teensy for new burst'
        )

        return []

    return update


# ---------------------------------------------------------------------------
#  Keyboard forwarding thread
#  Reads single keypresses from the console and sends them to the Teensy.
#  Works without pressing Enter — same experience as a serial monitor.
#
#  Windows: uses msvcrt.getwch() (non-blocking console read)
#  Linux / macOS: uses tty raw mode on stdin
# ---------------------------------------------------------------------------

def _keyboard_windows() -> None:
    """Forward keypresses to the Teensy (Windows implementation)."""
    import msvcrt
    while not _stop_event.is_set():
        if not msvcrt.kbhit():
            time.sleep(0.01)
            continue
        ch = msvcrt.getwch()
        # Ctrl+C / Ctrl+D → stop everything
        if ch in ('\x03', '\x04'):
            _stop_event.set()
            break
        # Arrow / function keys arrive as two-char sequences starting with
        # \x00 or \xe0 — consume and ignore the second byte.
        if ch in ('\x00', '\xe0'):
            msvcrt.getwch()
            continue
        s = _ser
        if s and s.is_open:
            try:
                s.write(ch.encode('latin-1', errors='replace'))
            except Exception:
                pass
        # Echo so the user can see what was sent
        printable = ch if (0x20 <= ord(ch) < 0x7F) else f'<{ord(ch):#04x}>'
        print(f'→ {printable}', end='  \r', flush=True)


def _keyboard_unix() -> None:
    """Forward keypresses to the Teensy (Linux/macOS implementation)."""
    import tty
    import termios
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while not _stop_event.is_set():
            ch = sys.stdin.read(1)
            if not ch or ch in ('\x03', '\x04'):   # Ctrl+C / Ctrl+D
                _stop_event.set()
                break
            s = _ser
            if s and s.is_open:
                try:
                    s.write(ch.encode('latin-1', errors='replace'))
                except Exception:
                    pass
            printable = ch if (0x20 <= ord(ch) < 0x7F) else f'<{ord(ch):#04x}>'
            print(f'→ {printable}', end='  \r', flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _keyboard_thread() -> None:
    """Dispatch to the platform-specific keyboard forwarder."""
    try:
        if platform.system() == 'Windows':
            _keyboard_windows()
        else:
            _keyboard_unix()
    except Exception as exc:
        # Non-fatal: keyboard forwarding unavailable (e.g. no TTY in IDE)
        print(f'[keys] Keyboard forwarding unavailable: {exc}')
        print('[keys] Use a different terminal or send commands another way.')


# ---------------------------------------------------------------------------
#  Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='TSL1401 burst capture + replay for test_demo.cpp camera mode')
    parser.add_argument('--port',   default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud',   type=int, default=DEFAULT_BAUD,
                        help=f'Baud rate (default: {DEFAULT_BAUD})')
    parser.add_argument('--fps',    type=int, default=DEFAULT_FPS,
                        help=f'Playback fps (default: {DEFAULT_FPS})')
    parser.add_argument('--output', default=DEFAULT_CSV,
                        help=f'Output CSV path (default: {DEFAULT_CSV})')
    args = parser.parse_args()

    # Port auto-detection
    available = [p.device for p in serial.tools.list_ports.comports()]
    port = args.port
    if port not in available:
        if available:
            port = available[0]
            print(f'[serial] {args.port} not found — using {port}.')
            print(f'[serial] Available: {available}')
        else:
            print('[serial] No serial ports found. Plug in the Teensy.')
            sys.exit(1)

    # Pre-load any existing CSV so animation shows something immediately
    existing = load_csv(args.output)
    if existing:
        with _lock:
            _play_frames[:] = existing
        print(f'[startup] Loaded {len(existing)} frames from {args.output}')
    else:
        print(f'[startup] No existing {args.output} — waiting for first burst.')

    # Start serial reader thread
    reader = threading.Thread(
        target=_reader_thread,
        args=(port, args.baud, args.output),
        daemon=True
    )
    reader.start()

    # Start keyboard forwarding thread
    kb = threading.Thread(target=_keyboard_thread, daemon=True)
    kb.start()

    # Build plot
    interval_ms = max(1, int(1000 / args.fps))
    fig, artists = build_plot(args.fps)
    update_fn    = make_update(artists, args.fps)

    # Pre-load existing frames into animation state
    if existing:
        with _lock:
            global _new_burst_ready
            _new_burst_ready = True

    # Keep ani in a module-level variable so Python's GC never collects it.
    # If FuncAnimation is GC'd the animation silently stops and plt.show()
    # may return immediately — this is a common "instant exit" cause.
    global _ani
    _ani = animation.FuncAnimation(
        fig, update_fn,
        interval=interval_ms,
        blit=False,
        cache_frame_data=False
    )

    print()
    print('=' * 60)
    print('  INTEGRATED TERMINAL — keystrokes go to the Teensy')
    print('  Make sure this console window is focused when typing.')
    print('  Common commands:')
    print('    5        switch to Camera mode')
    print('    B        start burst capture (5 s)')
    print('    ] [      cycle modes    1-8  jump to mode')
    print('    ?        help for current mode')
    print('    Ctrl+C   exit')
    print('=' * 60)
    print(f'[plot] Animation at {args.fps} fps — plot window will open now.')
    print()

    try:
        plt.show(block=True)   # block=True forces the call to wait for window close
    except KeyboardInterrupt:
        pass
    finally:
        _stop_event.set()
        print('[plot] Exited.')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        import traceback
        print('\n--- UNHANDLED ERROR ---')
        traceback.print_exc()
        print('\nDependencies needed:  pip install pyserial matplotlib numpy')
    finally:
        # Keep the terminal open so you can read any error messages
        # before the window closes.
        input('\nPress Enter to close...')
