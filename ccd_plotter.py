"""
ccd_plotter.py
==============
Real-time plotter for the TSL1401 128-pixel CCD sensor test.

Connects to the Teensy running test_tsl1401.cpp, sends 'S' to start
the full-frame stream, and plots three live panels:

  1. Pixel waveform  — 128-pixel intensity + threshold + edge markers
  2. Centre history  — scrolling centre-pixel position over time
  3. Width history   — scrolling object width in pixels over time

SETUP
-----
    pip install pyserial matplotlib

USAGE
-----
    python ccd_plotter.py [--port COM4] [--baud 115200]

    Close the window or press Ctrl+C to stop.  The Teensy stream is
    stopped cleanly before the script exits.

STREAM FORMAT (produced by test_tsl1401.cpp 'S' command)
---------------------------------------------------------
    F:{ms},{clk_us},{center},{left},{right},{threshold},{detected},{p0},...,{p127}

    All pixel values are 8-bit (10-bit ADC >> 2, range 0-255).
    Lines starting with '#' are comments and are ignored by this plotter.
"""

import argparse
import threading
import time
import sys
from collections import deque

import serial
import serial.tools.list_ports
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
import numpy as np

# ----------------------------------------------------------------
#  Configuration defaults (overridden by command-line args)
# ----------------------------------------------------------------
DEFAULT_PORT = 'COM4'
DEFAULT_BAUD = 115200
HISTORY_LEN  = 300   # Frames to keep in scrolling history plots
PIXEL_COUNT  = 128
UPDATE_MS    = 100   # Animation refresh interval (matches Teensy 10 Hz stream)

# ----------------------------------------------------------------
#  Parsed frame dataclass (plain dict for simplicity)
# ----------------------------------------------------------------
# Keys: ms, clk_us, center, left, right, threshold, detected, pixels (np.array)

# ----------------------------------------------------------------
#  Shared state — written by reader thread, read by animation
# ----------------------------------------------------------------
_lock         = threading.Lock()
_latest_frame = None          # Most recent parsed frame dict
_frame_count  = 0             # Total frames received (for FPS calc)
_fps_ts       = time.monotonic()
_fps_value    = 0.0

# Scrolling history buffers
_hist_ms       = deque(maxlen=HISTORY_LEN)
_hist_center   = deque(maxlen=HISTORY_LEN)
_hist_width    = deque(maxlen=HISTORY_LEN)
_hist_detected = deque(maxlen=HISTORY_LEN)


def _parse_frame(line: str):
    """Parse one F: stream line.  Returns a dict or None on error."""
    try:
        body = line[2:]   # strip "F:"
        parts = body.split(',')
        if len(parts) != 7 + PIXEL_COUNT:
            return None
        ms        = int(parts[0])
        clk_us    = int(parts[1])
        center    = int(parts[2])
        left      = int(parts[3])
        right     = int(parts[4])
        threshold = int(parts[5])
        detected  = int(parts[6]) == 1
        pixels    = np.array([int(p) for p in parts[7:]], dtype=np.uint8)
        return dict(ms=ms, clk_us=clk_us, center=center, left=left,
                    right=right, threshold=threshold, detected=detected,
                    pixels=pixels)
    except (ValueError, IndexError):
        return None


# ----------------------------------------------------------------
#  Serial reader — background thread
# ----------------------------------------------------------------
_ser = None
_stop_event = threading.Event()


def _reader_thread(port: str, baud: int):
    """Opens the serial port, sends 'S', reads F: lines indefinitely."""
    global _ser, _latest_frame, _frame_count, _fps_value, _fps_ts

    while not _stop_event.is_set():
        try:
            s = serial.Serial(port, baud, timeout=0.5)
            _ser = s
            print(f'[plotter] Connected to {port} at {baud} baud.')
            time.sleep(1.5)   # Wait for Teensy boot
            s.write(b'S')     # Start stream
            print('[plotter] Stream started (S sent).')
        except serial.SerialException as e:
            print(f'[plotter] Could not open {port}: {e}. Retrying...')
            time.sleep(1.0)
            continue

        try:
            while not _stop_event.is_set():
                raw = s.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='replace').rstrip()

                if not line.startswith('F:'):
                    # Echo non-frame lines (comments, status) to console
                    if line:
                        print(f'  {line}')
                    continue

                frame = _parse_frame(line)
                if frame is None:
                    continue

                now = time.monotonic()
                with _lock:
                    _latest_frame = frame
                    _frame_count += 1
                    _hist_ms.append(frame['ms'])
                    _hist_center.append(frame['center'])
                    _hist_width.append(
                        frame['right'] - frame['left'] if frame['detected'] else 0
                    )
                    _hist_detected.append(frame['detected'])

                    # Update FPS every second
                    elapsed = now - _fps_ts
                    if elapsed >= 1.0:
                        _fps_value = _frame_count / elapsed
                        _frame_count = 0
                        _fps_ts = now

        except serial.SerialException as e:
            print(f'[plotter] Connection lost: {e}. Reconnecting...')
            try:
                s.close()
            except Exception:
                pass
            _ser = None
            time.sleep(1.0)
            # Loop back to reconnect


def _stop_stream():
    """Send 'S' to stop streaming before closing."""
    s = _ser
    if s and s.is_open:
        try:
            s.write(b'S')
            time.sleep(0.1)
            print('[plotter] Stream stopped (S sent).')
        except Exception:
            pass


# ----------------------------------------------------------------
#  Plot setup
# ----------------------------------------------------------------
def build_plot():
    fig = plt.figure(figsize=(11, 8))
    fig.patch.set_facecolor('#1e1e1e')

    # Three vertically stacked subplots with height ratios 3:2:1
    gs = fig.add_gridspec(3, 1, height_ratios=[3, 2, 1],
                          hspace=0.45, left=0.08, right=0.97,
                          top=0.93, bottom=0.07)

    ax_wave  = fig.add_subplot(gs[0])  # Pixel waveform
    ax_ctr   = fig.add_subplot(gs[1])  # Centre history
    ax_width = fig.add_subplot(gs[2])  # Width history

    # ---- Waveform axes ----
    ax_wave.set_facecolor('#121212')
    ax_wave.set_xlim(0, PIXEL_COUNT - 1)
    ax_wave.set_ylim(0, 260)
    ax_wave.set_xlabel('Pixel index', color='#aaaaaa', fontsize=9)
    ax_wave.set_ylabel('Intensity (8-bit)', color='#aaaaaa', fontsize=9)
    ax_wave.set_title('TSL1401 CCD — waiting for data...', color='white', fontsize=10)
    ax_wave.tick_params(colors='#aaaaaa', labelsize=8)
    for sp in ax_wave.spines.values():
        sp.set_color('#444444')
    ax_wave.grid(True, color='#2a2a2a', linewidth=0.5)
    ax_wave.axhline(0, color='#444444', linewidth=0.5)

    px_x = np.arange(PIXEL_COUNT)
    zeros = np.zeros(PIXEL_COUNT)

    # Pixel waveform — fill under line
    wf_fill, = ax_wave.fill(
        np.concatenate([[0], px_x, [PIXEL_COUNT - 1]]),
        np.concatenate([[0], zeros, [0]]),
        color='#3a7abf', alpha=0.35
    )
    wf_line, = ax_wave.plot(px_x, zeros, color='#5ba3e0', linewidth=1.2)

    # Threshold horizontal line
    thr_line = ax_wave.axhline(0, color='#e05b5b', linewidth=1.0,
                               linestyle='--', label='Threshold')

    # Shadow region between left/right edges
    edge_span = ax_wave.axvspan(0, 1, alpha=0.15, color='#e0d45b', label='Object region')

    # Centre marker
    ctr_line = ax_wave.axvline(64, color='#5be07a', linewidth=1.5,
                               linestyle='-', label='Centre')

    # Legend
    ax_wave.legend(
        handles=[
            mpatches.Patch(color='#5ba3e0', label='Pixel intensity'),
            mpatches.Patch(color='#e05b5b', label='Threshold'),
            mpatches.Patch(color='#e0d45b', alpha=0.4, label='Object region'),
            mpatches.Patch(color='#5be07a', label='Centre pixel'),
        ],
        loc='upper right', fontsize=7,
        facecolor='#2a2a2a', edgecolor='#444444', labelcolor='#cccccc'
    )

    # ---- Centre history axes ----
    ax_ctr.set_facecolor('#121212')
    ax_ctr.set_xlim(0, HISTORY_LEN - 1)
    ax_ctr.set_ylim(0, PIXEL_COUNT - 1)
    ax_ctr.set_ylabel('Centre pixel', color='#aaaaaa', fontsize=9)
    ax_ctr.set_xlabel('Frame (newest → right)', color='#aaaaaa', fontsize=9)
    ax_ctr.set_title('Centre position history', color='#cccccc', fontsize=9)
    ax_ctr.tick_params(colors='#aaaaaa', labelsize=8)
    for sp in ax_ctr.spines.values():
        sp.set_color('#444444')
    ax_ctr.grid(True, color='#2a2a2a', linewidth=0.5)
    # Midpoint reference
    ax_ctr.axhline(64, color='#444444', linewidth=0.8, linestyle=':')
    ax_ctr.text(2, 66, 'mid', color='#555555', fontsize=7)

    ctr_detected_scatter = ax_ctr.scatter([], [], s=4, c='#5be07a',
                                          zorder=3, label='Detected')
    ctr_lost_scatter     = ax_ctr.scatter([], [], s=4, c='#555555',
                                          zorder=2, label='Not detected')
    ax_ctr.legend(loc='upper right', fontsize=7,
                  facecolor='#2a2a2a', edgecolor='#444444',
                  labelcolor='#cccccc', markerscale=2)

    # ---- Width history axes ----
    ax_width.set_facecolor('#121212')
    ax_width.set_xlim(0, HISTORY_LEN - 1)
    ax_width.set_ylim(0, PIXEL_COUNT)
    ax_width.set_ylabel('Width (px)', color='#aaaaaa', fontsize=9)
    ax_width.set_title('Object width history', color='#cccccc', fontsize=9)
    ax_width.tick_params(colors='#aaaaaa', labelsize=8)
    for sp in ax_width.spines.values():
        sp.set_color('#444444')
    ax_width.grid(True, color='#2a2a2a', linewidth=0.5)

    width_fill = ax_width.fill_between(
        range(HISTORY_LEN), 0, [0] * HISTORY_LEN,
        color='#e0d45b', alpha=0.5
    )
    width_line, = ax_width.plot([], [], color='#c9be4a', linewidth=1.0)

    artists = dict(
        wf_fill=wf_fill, wf_line=wf_line,
        thr_line=thr_line, edge_span=edge_span, ctr_line=ctr_line,
        ctr_detected=ctr_detected_scatter, ctr_lost=ctr_lost_scatter,
        width_fill=width_fill, width_line=width_line,
        ax_wave=ax_wave, ax_ctr=ax_ctr, ax_width=ax_width,
        px_x=px_x,
    )
    return fig, artists


# ----------------------------------------------------------------
#  Animation update callback
# ----------------------------------------------------------------
def make_update(artists):
    def update(_frame_num):
        with _lock:
            frame   = _latest_frame
            hist_c  = list(_hist_center)
            hist_w  = list(_hist_width)
            hist_d  = list(_hist_detected)
            fps     = _fps_value

        if frame is None:
            return []

        px_x      = artists['px_x']
        pixels    = frame['pixels'].astype(float)
        threshold = float(frame['threshold'])
        center    = frame['center']
        left      = frame['left']
        right     = frame['right']
        detected  = frame['detected']
        clk_us    = frame['clk_us']
        ms        = frame['ms']

        # ---- Waveform panel ----
        # Update fill polygon vertices
        verts = np.column_stack([
            np.concatenate([[px_x[0]], px_x, [px_x[-1]]]),
            np.concatenate([[0], pixels, [0]])
        ])
        artists['wf_fill'].set_xy(verts)
        artists['wf_line'].set_ydata(pixels)
        artists['thr_line'].set_ydata([threshold, threshold])

        # Edge span (object region)
        artists['edge_span'].remove()
        color = '#5be07a' if detected else '#e05b5b'
        artists['edge_span'] = artists['ax_wave'].axvspan(
            left, right, alpha=0.15, color='#e0d45b'
        )

        # Centre marker
        artists['ctr_line'].set_xdata([center, center])
        artists['ctr_line'].set_color('#5be07a' if detected else '#888888')

        # Width label on waveform
        width_px = right - left if detected else 0
        status = (f"DETECTED  centre={center}px  "
                  f"width={width_px}px  thr={threshold}"
                  if detected else
                  f"NO TARGET  thr={threshold}")
        artists['ax_wave'].set_title(
            f'TSL1401 CCD  |  {fps:.1f} fps  |  CLK ½T={clk_us}µs  |  {status}',
            color='#5be07a' if detected else '#e05b5b',
            fontsize=9
        )

        # ---- Centre history panel ----
        n = len(hist_c)
        if n > 0:
            xs = np.arange(HISTORY_LEN - n, HISTORY_LEN)
            ys = np.array(hist_c)
            det = np.array(hist_d, dtype=bool)

            if det.any():
                artists['ctr_detected'].set_offsets(
                    np.c_[xs[det], ys[det]]
                )
            else:
                artists['ctr_detected'].set_offsets(np.empty((0, 2)))

            if (~det).any():
                artists['ctr_lost'].set_offsets(
                    np.c_[xs[~det], ys[~det]]
                )
            else:
                artists['ctr_lost'].set_offsets(np.empty((0, 2)))

        # ---- Width history panel ----
        n_w = len(hist_w)
        if n_w > 0:
            xs_w = np.arange(HISTORY_LEN - n_w, HISTORY_LEN)
            ys_w = np.array(hist_w, dtype=float)
            artists['width_line'].set_data(xs_w, ys_w)

            # Rebuild fill_between
            artists['width_fill'].remove()
            artists['width_fill'] = artists['ax_width'].fill_between(
                xs_w, 0, ys_w, color='#e0d45b', alpha=0.45
            )

        return []

    return update


# ----------------------------------------------------------------
#  Entry point
# ----------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='Real-time TSL1401 CCD plotter for Teensy test_tsl1401.cpp')
    parser.add_argument('--port', default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                        help=f'Baud rate (default: {DEFAULT_BAUD})')
    args = parser.parse_args()

    # Auto-detect port
    available = [p.device for p in serial.tools.list_ports.comports()]
    port = args.port
    if port not in available:
        if available:
            port = available[0]
            print(f'[plotter] {args.port} not found — using {port}.')
            print(f'[plotter] Available ports: {available}')
        else:
            print('[plotter] No serial ports found. Is the Teensy plugged in?')
            sys.exit(1)

    # Start background reader thread
    reader = threading.Thread(
        target=_reader_thread, args=(port, args.baud), daemon=True
    )
    reader.start()

    print('[plotter] Starting live plot. Close window or Ctrl+C to stop.')
    print('[plotter] Teensy commands: +/- adjust CLK, L toggle laser, ? help.')

    fig, artists = build_plot()
    update_fn = make_update(artists)

    ani = animation.FuncAnimation(
        fig, update_fn,
        interval=UPDATE_MS,
        blit=False,   # False because axvspan needs full redraw
        cache_frame_data=False
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        _stop_event.set()
        _stop_stream()
        print('[plotter] Exited.')


if __name__ == '__main__':
    main()
