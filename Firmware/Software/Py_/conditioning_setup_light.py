"""
conditioning_setup.py
===========
PROJECT : Conditioning Setup
VERSION : 2.0
AUTHOR  : Flavio Afonso Goncalves Mourao
          mourao.fg@gmail.com

DESCRIPTION:
    PyQt5 graphical interface for the Conditioning Setup Arduino DUE.
    The DUE runs Stimuli_PY_DUE.ino and controls five independent stimuli:
    - SOUND (DAC1, AM sine), LIGHT (pin 45, square wave), 
    - LIGHT (pin 45, square wave 50% duty cycle),
    - SHOCK (8 bar pins,random sequence per trial), 
    - TRIGGER 1 (pin 10, digital pulse), 
    - TRIGGER 2 (pin 11, digital pulse). 
    
    This application programs trial parameters, triggers execution, and monitors status in real time.

SERIAL PROTOCOL (newline-terminated JSON, USB CDC speed):
    All commands are sent as compact JSON objects followed by newline.
    All responses from the DUE are also newline-terminated JSON objects.

    Commands sent by Python:
        {"cmd":"ping"}
            -> {"ok":true,"msg":"pong","version":"2.0"}
               Verifies that the DUE is alive and responsive.

        {"cmd":"program","n":N,"data":"f0;f1;..."}
            -> {"ok":true,"msg":"N trials programmed"}
               Sends N trials. 'data' is a flat semicolon-separated string of
               N x 20 float values in row-major order, matching FIELDS exactly.

        {"cmd":"start"}
            -> {"ok":true,"msg":"started"}
               Starts the experiment. RunExperiment() on the DUE is blocking;
               the DUE handles all stimulus timing autonomously after this.

        {"cmd":"abort"}
            -> {"ok":true,"msg":"aborted"}
               Immediately stops all stimuli and resets the DUE state machine.

        {"cmd":"status"}
            -> {"ok":true,"status":N,"trial":N,"total":N,
                "running":bool,"ready":bool,"fault":bool}
               Returns the current experiment state. Called automatically by
               the poll timer every 1500 ms while connected.

    DUE status codes:
        0 = IDLE     -- waiting for parameters
        1 = READY    -- parameters loaded, ready to start
        2 = RUNNING  -- experiment in progress
        3 = DONE     -- all trials completed normally
        4 = FAULT    -- watchdog fault, experiment aborted
        5 = ABORTED  -- aborted by command or hardware button

TRIAL DATA FORMAT (20 fields per trial, matching DUE struct Trial):
    baseline       -- quiet period at the start of the session (s)
    silence        -- inter-trial interval between each trial (s)
    onset_sound    -- sound onset within trial (s)
    sound_duration -- sound duration (s)
    carrier_freq   -- DAC carrier frequency (Hz); 0 = no sound
    modulator_freq -- AM modulator frequency (Hz); 0 = pure sine
    volume         -- amplitude (%), 0-100
    waveform_type  -- 0=SINE_AM, 1=SINE, 2=SQUARE
                      NOTE: reserved. The firmware supports all three modes,
                      but this UI always sends 0 (SINE_AM, which reduces to
                      pure sine when modulator_freq=0). A waveform selector
                      may be added in a future release.
    onset_shock    -- shock onset within trial (s)
    shock_duration -- shock duration (s); 0 = no shock
    pulse_high     -- shock bar ON time per pulse (ms)
    pulse_low      -- shock bar OFF time between pulses (ms)
    onset_light    -- light onset within trial (s)
    light_duration -- light duration (s); 0 = no light
    light_freq     -- light square wave frequency (Hz); 9999 = DC HIGH (constant ON)
    bar_select     -- shock bar: 0 = random sequence per trial, 1-8 = fixed single bar
    onset_trig1    -- Trigger 1 onset within trial (s)
    trig1_duration -- Trigger 1 pulse duration (ms); 0 = disabled
    onset_trig2    -- Trigger 2 onset within trial (s)
    trig2_duration -- Trigger 2 pulse duration (ms); 0 = disabled

REQUIREMENTS:
    pip install pyserial PyQt5

USAGE:
    python conditioning_setup_light.py
    Compatible with Spyder IDE (uses QApplication.instance() to avoid
    duplicate QApplication errors on re-run).
    
    
    
AUTHOR:
    Flavio Mourao  (mourao.fg@gmail.com)
    Federal University of Minas Gerais (UFMG) - Brazil

    Started:     12/2023
    Last update: 06/2026
    
"""

import sys
import os
import json
import time
import queue
import math
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QFrame, QComboBox, QLineEdit, QTextEdit,
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QAction, QFileDialog, QMessageBox, QDialog
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QTextCursor, QPainter, QColor, QPen, QFont, QPixmap

# =============================================================================
# THEME
# UI colour palette. All widgets reference these constants; change here to
# restyle the entire application.
# =============================================================================

BG        = "#f5f5f5"   # Main window background
BG_PANEL  = "#ffffff"   # Panel / section frame background
BG_INPUT  = "#ffffff"   # Input field background
BORDER    = "#d0d0d0"   # Inactive border colour
TEXT      = "#1a1a1a"   # Primary text (dark)
DIM       = "#888888"   # Secondary / label text
ACC_BLUE  = "#1a5fa8"   # Sound stimulus; RUNNING status; info highlights
ACC_RED   = "#9b1c1c"   # Shock stimulus; ABORT button; error messages
ACC_YELL  = "#b35f00"   # LED stimulus; warning messages; CONNECTING state
ACC_TRG1  = "#aaaaaa"   # Trigger 1 (light grey)
ACC_TRG2  = "#555555"   # Trigger 2 (dark grey)

# =============================================================================
# SENTINEL VALUES
# Used as "infinite duration" flags recognised by the DUE firmware.
# =============================================================================
CALIB_DURATION_S = 9999.0  # Calibration runs until ABORT (sound, shock, light)
LIGHT_DC_HIGH    = 9999.0  # light_freq sentinel: Timer7 not started, pin stays HIGH

# =============================================================================
# TOOLTIP STYLE
# Applied globally via app.setStyleSheet(TOOLTIP_STYLE) in __main__.
# =============================================================================
TOOLTIP_STYLE = f"""
    QToolTip {{
        background: {BG_PANEL};
        color: {TEXT};
        border: 1px solid {BORDER};
        font-size: 11px;
        padding: 4px 8px;
    }}
"""

# =============================================================================
# FIELDS
# Ordered list of trial parameter names. This list governs:
#   - The order in which values are serialised into the "data" string sent to
#     the DUE (must match the field assignment order in handleSerialUSB()).
#   - The columns written to / read from protocol .txt files.
#   - The keys expected in every trial dict throughout the application.
# =============================================================================

FIELDS = [
    "baseline",       # Quiet period before first trial (s)
    "silence",        # Inter-trial interval (s)
    "onset_sound",    # Sound onset within trial (s)
    "sound_duration", # Sound duration (s)
    "carrier_freq",   # DAC carrier frequency (Hz)
    "modulator_freq", # AM modulator frequency (Hz); 0 = pure sine
    "volume",         # Sound amplitude (%)
    "waveform_type",  # 0=SINE_AM  1=SINE  2=SQUARE -- reserved; UI always sends 0
    "onset_shock",    # Shock onset within trial (s)
    "shock_duration", # Shock duration (s)
    "pulse_high",     # Shock bar ON time per pulse (ms)
    "pulse_low",      # Shock bar OFF time between pulses (ms)
    "onset_light",    # LIGHT onset within trial (s)
    "light_duration", # LIGHT duration (s)
    "light_freq",     # LIGHT frequency (Hz); 9999 = DC HIGH (constant ON)
    "bar_select",     # Shock bar selection: 0 = random sequence per trial, 1-8 = fixed single bar
    "onset_trig1",    # Trigger 1 onset within trial (s)
    "trig1_duration", # Trigger 1 pulse duration (ms); 0 = disabled
    "onset_trig2",    # Trigger 2 onset within trial (s)
    "trig2_duration", # Trigger 2 pulse duration (ms); 0 = disabled
]

# =============================================================================
# RESOURCE PATH
# Resolves file paths for both development and PyInstaller packaged builds.
# =============================================================================
def resource_path(relative):
    base = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base, relative)

# =============================================================================
# SerialThread
# Runs the serial port in a dedicated QThread so the UI never blocks.
#
# Architecture:
#   - CageApp creates a SerialThread and connects its signals to UI slots.
#   - Commands (JSON dicts) are pushed into _tx_queue from the main thread.
#   - run() drains _tx_queue and writes to the serial port, then reads any
#     incoming bytes and emits data_received for each complete JSON line.
#   - An initial 5-second window after opening the port drains any DUE boot
#     message before emitting ready(), which switches the UI to DISCONNECT.
#
# Thread safety:
#   - _tx_queue is a thread-safe queue.Queue; no mutex is needed.
#   - All UI updates happen in the main thread via Qt signals/slots.
# =============================================================================

class SerialThread(QThread):
    data_received    = pyqtSignal(dict)  # Emitted for every valid JSON line received
    connection_error = pyqtSignal(str)   # Emitted on serial exception
    ready            = pyqtSignal()      # Emitted after the 5 s init window completes

    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port      = port
        self.baudrate  = baudrate
        self.ser       = None
        self.running   = True          # Set to False by stop() to exit run()
        self._tx_queue = queue.Queue() # Thread-safe outgoing command queue

    def stop(self):
        """Signal the run() loop to exit on the next iteration."""
        self.running = False

    def run(self):
        """
        Main thread body. Opens the serial port, waits for the DUE to
        stabilise, then enters the read/write loop until stop() is called.
        """
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)

            # PORT SELECTION NOTE:
            #
            # Arduino DUE has two USB connectors:
            #
            # PROGRAMMING PORT (same connector used for firmware upload):
            #   Opening this port asserts DTR, causing a hardware reset of the
            #   SAM3X8E. The bootloader occupies ~8 s before setup() runs.
            #   Firmware must use: Serial.begin(115200)
            #   To use this port, replace time.sleep(1) with time.sleep(9).
            #
            # NATIVE PORT (second connector, labelled "Native" on the board):
            #   No DTR reset. Direct USB connection to the SAM3X8E.
            #   Firmware must use: SerialUSB.begin(115200)
            #   This is the recommended port for this application.

            time.sleep(1)  # Native port: 1 s stabilisation after open

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            # Init window: spend 5 s flushing any pending TX and reading the
            # DUE boot message ("Conditioning Setup v2.0 ready").
            init_deadline = time.time() + 5.0
            while time.time() < init_deadline:
                self._drain_tx()
                self._read_lines()
                time.sleep(0.05)

            # Signal the UI that the port is open and the DUE is ready.
            self.ready.emit()

            # Main read/write loop: runs until stop() sets self.running = False.
            while self.running:
                self._drain_tx()
                self._read_lines()
                time.sleep(0.01)

        except Exception as e:
            self.connection_error.emit(str(e))
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def _drain_tx(self):
        """Write all queued commands to the serial port."""
        while not self._tx_queue.empty():
            try:
                line = self._tx_queue.get_nowait()
                self.ser.write(line.encode('utf-8'))
                self.ser.flush()
            except queue.Empty:
                break

    def _read_lines(self):
        """Read all available lines and emit data_received for valid JSON."""
        while self.ser.in_waiting > 0:
            raw = self.ser.readline()
            if not raw:
                continue
            text = raw.decode('utf-8', errors='ignore').strip()
            if text.startswith('{') and text.endswith('}'):
                try:
                    self.data_received.emit(json.loads(text))
                except json.JSONDecodeError:
                    pass

    def send(self, cmd_dict):
        """
        Queue a command for transmission. Safe to call from any thread.
        cmd_dict is serialised to compact JSON and terminated with newline.
        Example: send({"cmd": "start"})
        """
        self._tx_queue.put(json.dumps(cmd_dict, separators=(',', ':')) + '\n')


# =============================================================================
# TimingWidget
# Custom QWidget that paints a block timeline of the full trial session.
#
# Layout (top to bottom):
#   SOUND row  -- blue blocks at (onset_sound, sound_duration) per trial
#   LIGHT row    -- yellow blocks at (onset_light, light_duration) per trial
#   SHOCK row  -- red blocks at (onset_shock, shock_duration) per trial
#   Time axis  -- labelled tick marks in seconds
#
# Time model:
#   The horizontal axis represents cumulative session time starting at t=0.
#   For each trial the cursor advances by (baseline + silence) before placing
#   stimulus blocks, then by the trial duration (max of all stimulus end times).
#   Inter-trial intervals appear as empty space between block groups.
#
# Progress line:
#   A semi-transparent white vertical line tracks elapsed experiment time.
#   Updated every 200 ms by CageApp._progress_timer during an experiment.
#   Hidden (progress_time = None) when the experiment is not running.
# =============================================================================

class TimingWidget(QWidget):

    ROW_H   = 28  # Height of each stimulus row (px)
    ROW_GAP =  8  # Vertical gap between rows (px)
    PAD_L   = 52  # Left padding reserved for row labels (px)
    PAD_R   = 16  # Right padding (px)
    PAD_T   = 10  # Top padding (px)
    PAD_B   = 26  # Bottom padding for time axis labels (px)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.trials        = []   # List of trial dicts, set by set_trials()
        self.progress_time = None # Elapsed seconds; None = progress line hidden
        # Height accommodates up to 5 rows (SOUND, LIGHT, SHOCK, TRIG1, TRIG2).
        # Unused rows are hidden dynamically in paintEvent but space is reserved
        # so the widget does not jump when rows appear/disappear.
        self.setMinimumHeight(
            self.PAD_T + 5 * self.ROW_H + 4 * self.ROW_GAP + self.PAD_B + 10
        )
        self.setStyleSheet(f"background:{BG_PANEL};")

    def set_trials(self, trials):
        """Replace the trial list and trigger a repaint."""
        self.trials = trials
        self.update()

    def set_progress(self, elapsed_s):
        """
        Set the progress line position (seconds from experiment start).
        Pass None to hide the line (experiment not running).
        """
        self.progress_time = elapsed_s
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        if not self.trials:
            p.setPen(QColor(DIM))
            p.setFont(QFont("Courier New", 11))
            p.drawText(self.rect(), Qt.AlignCenter, "Add a trial to preview")
            p.end()
            return

        w      = self.width()
        draw_w = w - self.PAD_L - self.PAD_R

        # Build absolute time position of each trial's stimulus start.
        # cursor advances by (baseline + silence), then by the trial duration.
        events = []   # List of (abs_t0, trial_dict)
        cursor = 0.0
        for t in self.trials:
            cursor += t.get('baseline', 0) + t.get('silence', 0)
            events.append((cursor, t))
            s_end  = t.get('onset_sound', 0) + t.get('sound_duration', 0)
            k_end  = t.get('onset_shock', 0) + t.get('shock_duration', 0)
            l_end  = t.get('onset_light',  0) + t.get('light_duration',  0)
            t1_end = t.get('onset_trig1', 0) + t.get('trig1_duration', 0) / 1000.0
            t2_end = t.get('onset_trig2', 0) + t.get('trig2_duration', 0) / 1000.0
            cursor += max(s_end, k_end, l_end, t1_end, t2_end, 0.0)

        total = max(cursor, 0.1)  # Avoid division by zero

        def s2x(sec):
            """Convert a time value (s) to a pixel x-coordinate."""
            return self.PAD_L + sec / total * draw_w

        # Row vertical positions
        y_sound  = self.PAD_T
        y_light  = self.PAD_T + self.ROW_H + self.ROW_GAP
        y_shock  = self.PAD_T + 2 * (self.ROW_H + self.ROW_GAP)
        y_trig1  = self.PAD_T + 3 * (self.ROW_H + self.ROW_GAP)
        y_trig2  = self.PAD_T + 4 * (self.ROW_H + self.ROW_GAP)

        # Determine which rows have active stimuli across all trials
        has_sound  = any(t.get('sound_duration', 0) > 0 for t in self.trials)
        has_light  = any(t.get('light_duration', 0) > 0 for t in self.trials)
        has_shock  = any(t.get('shock_duration', 0) > 0 for t in self.trials)
        has_trig1  = any(t.get('trig1_duration', 0) > 0 for t in self.trials)
        has_trig2  = any(t.get('trig2_duration', 0) > 0 for t in self.trials)

        # Build active rows list -- only rows with at least one active trial are shown
        active_rows = []
        if has_sound: active_rows.append((y_sound, ACC_BLUE,  "SOUND"))
        if has_light: active_rows.append((y_light, ACC_YELL,  "LIGHT"))
        if has_shock: active_rows.append((y_shock, ACC_RED,   "SHOCK"))
        if has_trig1: active_rows.append((y_trig1, ACC_TRG1,  "TRIG 1"))
        if has_trig2: active_rows.append((y_trig2, ACC_TRG2,  "TRIG 2"))

        # Recompute row positions based on active rows only
        row_positions = {}
        for row_idx, (_, color_hex, label) in enumerate(active_rows):
            row_positions[label] = self.PAD_T + row_idx * (self.ROW_H + self.ROW_GAP)

        # Remap y variables to compact positions
        y_sound = row_positions.get("SOUND",  y_sound)
        y_light = row_positions.get("LIGHT",  y_light)
        y_shock = row_positions.get("SHOCK",  y_shock)
        y_trig1 = row_positions.get("TRIG 1", y_trig1)
        y_trig2 = row_positions.get("TRIG 2", y_trig2)
        # Bottom of last active row
        last_y  = self.PAD_T + max(len(active_rows) - 1, 0) * (self.ROW_H + self.ROW_GAP)

        # Faint background track for each active row
        for y, color_hex, _ in active_rows:
            y = row_positions[_]
            c = QColor(color_hex)
            c.setAlpha(25)
            p.setPen(Qt.NoPen)
            p.setBrush(c)
            p.drawRect(int(self.PAD_L), y, int(draw_w), self.ROW_H)

        # Stimulus blocks for each trial.
        # _t0 default captures abs_t0 at definition time (avoids Python closure gotcha).
        for abs_t0, t in events:
            def draw_block(y, color_hex, onset, duration, _t0=abs_t0):
                if duration <= 0:
                    return
                x1 = int(s2x(_t0 + onset))
                x2 = max(int(s2x(_t0 + onset + duration)), x1 + 3)
                c = QColor(color_hex)
                c.setAlpha(210)
                p.setPen(Qt.NoPen)
                p.setBrush(c)
                p.drawRoundedRect(x1, y + 2, x2 - x1, self.ROW_H - 4, 2, 2)

            draw_block(y_sound, ACC_BLUE, t.get('onset_sound', 0), t.get('sound_duration', 0))
            draw_block(y_light,  ACC_YELL, t.get('onset_light',  0), t.get('light_duration', 0))
            draw_block(y_shock,  ACC_RED,  t.get('onset_shock',  0), t.get('shock_duration', 0))
            # Triggers: trig1_duration/trig2_duration are in ms -- convert to s for display.
            draw_block(y_trig1, ACC_TRG1, t.get('onset_trig1', 0), t.get('trig1_duration', 0) / 1000.0)
            draw_block(y_trig2, ACC_TRG2, t.get('onset_trig2', 0), t.get('trig2_duration', 0) / 1000.0)

            # Separator at the start of each trial (skip the first)
            if abs_t0 > 0:
                sx = int(s2x(abs_t0))
                p.setPen(QPen(QColor(BORDER), 1))
                p.drawLine(sx, self.PAD_T, sx, last_y + self.ROW_H)

        # Row labels -- only for active rows
        for label, (_, color_hex, lbl) in [(r[2], r) for r in active_rows]:
            y = row_positions[label]
            p.setPen(QColor(color_hex))
            p.setFont(QFont("Courier New", 11, QFont.Bold))
            p.drawText(0, y, self.PAD_L - 4, self.ROW_H,
                       Qt.AlignRight | Qt.AlignVCenter, label)

        # Time axis
        axis_y = last_y + self.ROW_H + 4
        p.setPen(QPen(QColor(DIM), 1))
        p.drawLine(int(self.PAD_L), axis_y, w - self.PAD_R, axis_y)

        # Tick marks: ~5 ticks with a "nice" interval
        raw_step = total / 5
        mag      = 10 ** math.floor(math.log10(raw_step)) if raw_step > 0 else 1
        step     = mag * round(raw_step / mag) if raw_step > 0 else 1
        step     = max(step, 0.5)

        p.setFont(QFont("Courier New", 11))
        t_tick = 0.0
        while t_tick <= total + step * 0.01:
            x = int(s2x(t_tick))
            p.setPen(QPen(QColor(DIM), 1))
            p.drawLine(x, axis_y, x, axis_y + 4)
            lbl = f"{t_tick:.0f}s" if step >= 1 else f"{t_tick:.1f}s"
            p.drawText(x - 18, axis_y + 6, 36, 14, Qt.AlignHCenter, lbl)
            t_tick += step

        # Progress line: semi-transparent white bar showing elapsed time.
        # Clamped to [0, total] so it never overruns the right edge.
        if self.progress_time is not None:
            OFFSET = -0.1  # seconds -- compensates DUE handshake + firmware delay(50ms)
            adjusted = max(self.progress_time - OFFSET, 0.0)
            px = int(s2x(min(adjusted, total)))
            p.setPen(QPen(QColor(255, 255, 255, 140), 2))
            p.drawLine(px, self.PAD_T, px, last_y + self.ROW_H)

        p.end()


# =============================================================================
# UI FACTORY HELPERS
# =============================================================================

def make_input_row(label_text, default_val, unit_text="", color=TEXT):
    """
    Build a horizontal input row: [label | QLineEdit | unit label].
    Returns (QWidget, QLineEdit) so the caller can read the entered value.
    """
    row = QWidget()
    lay = QHBoxLayout(row)
    lay.setContentsMargins(0, 2, 0, 2)
    lay.setSpacing(10)

    lbl = QLabel(label_text)
    lbl.setStyleSheet(f"color:{color}; font-size:12px;")
    lbl.setFixedWidth(110)
    lay.addWidget(lbl)

    edit = QLineEdit(str(default_val))
    edit.setStyleSheet(
        f"background:{BG_INPUT}; color:{TEXT}; border:1px solid {BORDER}; "
        f"font-size:11px; padding:4px;"
    )
    edit.setFixedWidth(60)
    edit.setAlignment(Qt.AlignCenter)
    lay.addWidget(edit)

    u = QLabel(unit_text)
    u.setStyleSheet(f"color:{DIM}; font-size:11px;")
    u.setFixedWidth(40)
    lay.addWidget(u)

    lay.addStretch()
    return row, edit


def make_button(text, color=TEXT, border_color=BORDER):
    """
    Build a flat push button with hover/press highlight effects.
    color sets both the text and border colour by convention.
    """
    b = QPushButton(text)
    b.setStyleSheet(
        f"QPushButton {{ background:transparent; color:{color}; "
        f"border:1px solid {border_color}; "
        f"font-size:11px; font-weight:bold; padding:8px 12px; }}"
        f"QPushButton:hover {{ background:rgba(255,255,255,0.05); }}"
        f"QPushButton:pressed {{ background:rgba(255,255,255,0.1); }}"
    )
    b.setCursor(Qt.PointingHandCursor)
    return b


def make_section_frame(title):
    """
    Build a titled section frame with a content layout.
    Returns (QFrame, QVBoxLayout) so the caller can add child widgets.
    """
    frame = QFrame()
    frame.setStyleSheet(f"QFrame {{ background:{BG_PANEL}; border:1px solid {BORDER}; }}")
    lay = QVBoxLayout(frame)
    lay.setContentsMargins(12, 12, 12, 12)
    lay.setSpacing(8)

    lbl = QLabel(title)
    lbl.setStyleSheet(
        f"color:{DIM}; font-size:11px; font-weight:bold; letter-spacing:1px; border:none;"
    )
    lay.addWidget(lbl)

    content = QVBoxLayout()
    content.setContentsMargins(0, 0, 0, 0)
    content.setSpacing(4)
    lay.addLayout(content)

    return frame, content


# =============================================================================
# Calibration dialogs
# Each dialog sends a single calibration trial to the DUE with an effectively
# infinite duration (9999 s) so the stimulus runs until ABORT is pressed.
# The trial dict is serialised using FIELDS to guarantee field order matches
# the DUE firmware exactly -- the same mechanism used by _send_params().
#
# _CalibBase provides shared helpers; the three subclasses each implement
# __init__ (UI) and _start() (trial dict).
# =============================================================================

class _CalibBase(QDialog):
    """Shared base for calibration dialogs."""

    def __init__(self, serial_thread, parent=None):
        super().__init__(parent)
        self.serial_thread = serial_thread

    def _send_trial(self, trial_dict):
        """Serialise trial_dict via FIELDS and send program + start to DUE."""
        if not self.serial_thread:
            QMessageBox.warning(self, "Not Connected", "Connect first.")
            return
        # Stop poll BEFORE sending commands to avoid USB glitch during handshake
        if self.parent():
            self.parent()._poll_timer.stop()
        values = ";".join(str(float(trial_dict[f])) for f in FIELDS)
        self.serial_thread.send({"cmd": "program", "n": 1, "data": values})
        self.serial_thread.send({"cmd": "start"})
            
    def _abort(self):
        """Send abort to stop the calibration stimulus.
        If the parent window has programmed trials, re-sends them to the DUE
        immediately so the DUE memory is restored to the experiment state.
        This prevents the calibration trial (e.g. shock_duration=9999) from
        remaining in DUE RAM and being accidentally executed on next START."""
        if self.serial_thread:
            self.serial_thread.send({"cmd": "abort"})
            # Restore experiment trials to DUE memory if available.
            # Without this, the DUE would retain the calibration trial in RAM
            # and execute it if the user clicks START without re-sending params.
            parent = self.parent()
            if parent and parent.trials:
                values = ";".join(
                    str(float(t[f])) for t in parent.trials for f in FIELDS
                )
                self.serial_thread.send({
                    "cmd": "program",
                    "n": len(parent.trials),
                    "data": values
                })
        # Resume polling after calibration stops
        if self.parent():
            self.parent()._poll_timer.start()
    

    def _field(self, lay, label, default, unit):
        """Add an inline label/input/unit row to lay; return the QLineEdit."""
        row = QWidget()
        rl  = QHBoxLayout(row)
        rl.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel(label)
        lbl.setStyleSheet(f"color:{TEXT}; font-size:11px;")
        lbl.setFixedWidth(110)
        edit = QLineEdit(str(default))
        edit.setStyleSheet(
            f"background:{BG_INPUT}; color:{TEXT}; border:1px solid {BORDER}; "
            f"font-size:11px; padding:4px;"
        )
        edit.setFixedWidth(70)
        edit.setAlignment(Qt.AlignCenter)
        u = QLabel(unit)
        u.setStyleSheet(f"color:{DIM}; font-size:10px;")
        rl.addWidget(lbl); rl.addWidget(edit); rl.addWidget(u); rl.addStretch()
        lay.addWidget(row)
        return edit

    def _get(self, edit, default=0.0):
        try:
            return float(edit.text())
        except ValueError:
            return default

    def closeEvent(self, event):
        """Abort any active calibration stimulus when dialog is closed.
        Without this, closing with X leaves stimulus running for 9999 s.
        Delegates to _abort() so DUE memory is also restored if trials exist."""
        self._abort()
        event.accept()

    def _build_btn_row(self, lay):
        """Add START / ABORT buttons to lay."""
        btn_row   = QHBoxLayout()
        btn_start = make_button("START", TEXT, TEXT)
        btn_start.clicked.connect(self._start)
        btn_abort = make_button("ABORT", ACC_RED, ACC_RED)
        btn_abort.clicked.connect(self._abort)
        btn_row.addWidget(btn_start)
        btn_row.addWidget(btn_abort)
        lay.addLayout(btn_row)


class CalibSoundDialog(_CalibBase):
    """
    Continuous tone until ABORT.
    sound_duration = 9999 s acts as an infinite sentinel in the firmware.
    """
    def __init__(self, serial_thread, parent=None):
        super().__init__(serial_thread, parent)
        self.setWindowTitle("Calibration - Sound")
        self.setFixedWidth(320)
        self.setStyleSheet(f"background:{BG_PANEL}; color:{TEXT};")
        lay = QVBoxLayout(self)
        lay.setSpacing(12)
        lay.setContentsMargins(20, 20, 20, 20)
        title = QLabel("SOUND CALIBRATION")
        title.setStyleSheet(f"color:{ACC_BLUE}; font-size:13px; font-weight:bold;")
        title.setTextFormat(Qt.RichText)
        lay.addWidget(title)
        self.e_carrier = self._field(lay, "Carrier freq", 3000, "Hz")
        self.e_mod     = self._field(lay, "Modulator",       0, "Hz")
        self.e_volume  = self._field(lay, "Volume",        100, "%")
        info = QLabel("Sound plays continuously until ABORT.")
        info.setStyleSheet(f"color:{DIM}; font-size:10px;")
        info.setAlignment(Qt.AlignCenter)
        lay.addWidget(info)
        self._build_btn_row(lay)

    def _start(self):
        self._send_trial({
            "baseline": 0.0, "silence": 0.0,
            "onset_sound": 0.0, "sound_duration": CALIB_DURATION_S,
            "carrier_freq":   self._get(self.e_carrier),
            "modulator_freq": self._get(self.e_mod),
            "volume":         self._get(self.e_volume),
            "waveform_type": 0.0, # Reserved; calibration always uses SINE_AM
            "onset_shock": 0.0, "shock_duration": 0.0,
            "pulse_high": 0.0, "pulse_low": 0.0,
            "onset_light": 0.0, "light_duration": 0.0, "light_freq": 0.0,
            "bar_select": 0.0,
            "onset_trig1": 0.0, "trig1_duration": 0.0,
            "onset_trig2": 0.0, "trig2_duration": 0.0,
        })


class CalibLightDialog(_CalibBase):
    """
    LIGHT held continuously HIGH until ABORT.
    light_freq = 9999 is the firmware sentinel that skips Timer7 and drives
    the LIGHT pin permanently HIGH. light_duration = 9999 s runs until ABORT.
    """
    def __init__(self, serial_thread, parent=None):
        super().__init__(serial_thread, parent)
        self.setWindowTitle("Calibration - Light")
        self.setFixedWidth(320)
        self.setStyleSheet(f"background:{BG_PANEL}; color:{TEXT};")
        lay = QVBoxLayout(self)
        lay.setSpacing(12)
        lay.setContentsMargins(20, 20, 20, 20)
        title = QLabel("LIGHT CALIBRATION")
        title.setStyleSheet(f"color:{ACC_YELL}; font-size:13px; font-weight:bold;")
        title.setTextFormat(Qt.RichText)
        lay.addWidget(title)
        info = QLabel("LIGHT held ON (HIGH) continuously\nPress ABORT to turn off.")
        info.setStyleSheet(f"color:{DIM}; font-size:11px;")
        info.setAlignment(Qt.AlignCenter)
        lay.addWidget(info)
        self._build_btn_row(lay)

    def _start(self):
        self._send_trial({
            "baseline": 0.0, "silence": 0.0,
            "onset_sound": 0.0, "sound_duration": 0.0,
            "carrier_freq": 0.0, "modulator_freq": 0.0,
            "volume": 0.0, "waveform_type": 0.0,
            "onset_shock": 0.0, "shock_duration": 0.0,
            "pulse_high": 0.0, "pulse_low": 0.0,
            "onset_light": 0.0, "light_duration": CALIB_DURATION_S, "light_freq": CALIB_DURATION_S,
            "bar_select": 0.0,
            "onset_trig1": 0.0, "trig1_duration": 0.0,
            "onset_trig2": 0.0, "trig2_duration": 0.0,
        })


class CalibShockDialog(_CalibBase):
    """
    Shock bar calibration.
    Allows selection of a specific bar (1-8) or random sequence (all bars).
    bar_select = All (0)  -> random sequence per trial (default behaviour).
    bar_select = 1-8      -> fixed single bar; stays active for entire shock_duration.
    pulse_high and pulse_low are user-configurable (ms).
    shock_duration = 9999 s runs until ABORT.
    """
    def __init__(self, serial_thread, parent=None):
        super().__init__(serial_thread, parent)
        self.setWindowTitle("Calibration - Shock")
        self.setFixedWidth(380)
        self.setStyleSheet(f"background:{BG_PANEL}; color:{TEXT};")
        lay = QVBoxLayout(self)
        lay.setSpacing(12)
        lay.setContentsMargins(20, 20, 20, 20)

        title = QLabel("SHOCK CALIBRATION")
        title.setStyleSheet(f"color:{ACC_RED}; font-size:13px; font-weight:bold;")
        title.setTextFormat(Qt.RichText)
        lay.addWidget(title)

        # Pulse timing inputs
        self.e_pulse_high = self._field(lay, "Pulse HIGH", 60000, "ms")
        self.e_pulse_low  = self._field(lay, "Pulse LOW",  10000, "ms")

        # Bar selection
        bar_row = QWidget()
        bar_lay = QHBoxLayout(bar_row)
        bar_lay.setContentsMargins(0, 0, 0, 0)
        bar_lay.setSpacing(10)
        bar_lbl = QLabel("Bar select")
        bar_lbl.setStyleSheet(f"color:{TEXT}; font-size:11px;")
        bar_lbl.setFixedWidth(110)
        bar_lay.addWidget(bar_lbl)

        self.cmb_bar = QComboBox()
        self.cmb_bar.addItem("All", 0)
        for i in range(1, 9):
            self.cmb_bar.addItem(f"Bar {i}", i)
        self.cmb_bar.setStyleSheet(
            f"background:{BG_INPUT}; color:{TEXT}; border:1px solid {BORDER}; "
            f"font-size:11px; padding:2px;"
        )
        self.cmb_bar.setFixedWidth(130)
        bar_lay.addWidget(self.cmb_bar)
        bar_lay.addStretch()
        lay.addWidget(bar_row)

        info = QLabel("For individual bars shock runs continuously until ABORT.")
        info.setStyleSheet(f"color:{DIM}; font-size:10px;")
        info.setAlignment(Qt.AlignCenter)
        lay.addWidget(info)

        self._build_btn_row(lay)

    def _start(self):
        bar_sel = float(self.cmb_bar.currentData())
        self._send_trial({
            "baseline": 0.0, "silence": 0.0,
            "onset_sound": 0.0, "sound_duration": 0.0,
            "carrier_freq": 0.0, "modulator_freq": 0.0,
            "volume": 0.0, "waveform_type": 0.0,
            "onset_shock": 0.0, "shock_duration": CALIB_DURATION_S,
            "pulse_high": self._get(self.e_pulse_high, 60000.0),
            "pulse_low":  self._get(self.e_pulse_low,  10000.0),
            "onset_light": 0.0, "light_duration": 0.0, "light_freq": 0.0,
            "bar_select": bar_sel,
            "onset_trig1": 0.0, "trig1_duration": 0.0,
            "onset_trig2": 0.0, "trig2_duration": 0.0,
        })


# =============================================================================
# CageApp -- Main window
# =============================================================================

class CageApp(QMainWindow):
    def __init__(self):
        super().__init__()
        #self.setWindowTitle("Conditioning Setup v2.0")
        self.setWindowTitle("Conditioning Setup")
        #self.resize(1800, 850)
        
        # Set window icon for taskbar and title bar
        from PyQt5.QtGui import QIcon
        icon_path = resource_path("icon.ico")
        if os.path.exists(icon_path):
            self.setWindowIcon(QIcon(icon_path))
      
        # Centre window on screen at startup
        #screen = QApplication.primaryScreen().geometry()
        #x = (screen.width()  - 1800) // 2
        #y = (screen.height() -  850) // 4
        #self.move(x, y)

        # Adaptive window size -- fits any screen resolution
        screen = QApplication.primaryScreen().availableGeometry()
        w = min(1600, screen.width()  - 40)
        h = min(1110,  screen.height() - 40)
        self.resize(w, h)
        self.setMinimumSize(900, 600)
        #self.setMaximumSize(2560, 1600)
        #self.setMaximumSize(1540, 1100)
        self.move(screen.x() + (screen.width()  - w) // 2,
                  screen.y() + (screen.height() - h) // 4)

        self.setStyleSheet(f"background:{BG}; color:{TEXT};")

        # Application state
        self.trials                   = []            # List of trial dicts built by _add_trial()
        self._connected               = False         # True while a SerialThread is active
        self.serial_thread            = None          # Active SerialThread instance, or None
        self._ready_confirmed         = False         # Set True when any data arrives from DUE
        self._programmed_total        = 0             # Number of trials last acknowledged by DUE
        self._experiment_start        = None          # time.time() snapshot when START was sent
        self._experiment_done_logged  = False         # True after "Experiment completed" logged -- prevents loop
        self._poll_generation         = 0             # Incremented each START; orphaned singleShots are suppressed
        self._params_sent             = False         # True after SEND PARAMETERS acknowledged by DUE
        
        self.init_ui()
        self._build_menu()

        # Poll timer: sends {"cmd":"status"} every 1500 ms while connected.
        # The DUE response updates the status pill and trial counter.
        self._poll_timer = QTimer()
        self._poll_timer.timeout.connect(self._poll_status)
        self._poll_timer.setInterval(1500)

        # Progress timer: repaints the timeline progress line every 200 ms
        # during a running experiment. Stopped by _stop_progress().
        self._progress_timer = QTimer()
        self._progress_timer.timeout.connect(self._update_progress)
        self._progress_timer.setInterval(200)

    # -------------------------------------------------------------------------
    # Menu bar
    # -------------------------------------------------------------------------

    def _build_menu(self):
        menubar = self.menuBar()
        menubar.setNativeMenuBar(False)
        menubar.setStyleSheet(
            f"QMenuBar {{ background:{BG_PANEL}; color:{TEXT}; border-bottom:1px solid {BORDER}; }}"
            f"QMenuBar::item:selected {{ background:{BORDER}; }}"
            f"QMenu {{ background:{BG_PANEL}; color:{TEXT}; border:1px solid {BORDER}; }}"
            f"QMenu::item:selected {{ background:{BORDER}; }}"
        )

        # Protocol menu: save / load / clear trial lists
        proto_menu = menubar.addMenu("Protocol")
        act_save  = QAction("Save Protocol...", self); act_save.setShortcut("Ctrl+S")
        act_load  = QAction("Load Protocol...", self); act_load.setShortcut("Ctrl+O")
        act_save_log = QAction("Save Log...", self)
        act_clear = QAction("Clear All Trials",  self)

        act_save.triggered.connect(self._save_protocol)
        act_load.triggered.connect(self._load_protocol)
        act_save_log.triggered.connect(self._save_log)
        act_clear.triggered.connect(self._clear_trials)
        
        proto_menu.addAction(act_save)
        proto_menu.addAction(act_load)
        proto_menu.addSeparator()
        proto_menu.addAction(act_save_log)
        proto_menu.addSeparator()
        proto_menu.addAction(act_clear)

        # Calibration menu: individual stimulus calibration dialogs
        calib_menu  = menubar.addMenu("Calibration")
        act_cal_snd = QAction("Sound", self)
        act_cal_light = QAction("Light",   self)
        act_cal_shk = QAction("Shock", self)
        act_cal_snd.triggered.connect(lambda: CalibSoundDialog(self.serial_thread, self).exec_())
        act_cal_light.triggered.connect(lambda: CalibLightDialog(self.serial_thread,   self).exec_())
        act_cal_shk.triggered.connect(lambda: CalibShockDialog(self.serial_thread, self).exec_())
        calib_menu.addAction(act_cal_snd)
        calib_menu.addAction(act_cal_light)
        calib_menu.addAction(act_cal_shk)
        
        # HELP menu
        help_menu = menubar.addMenu("Help")
        act_info  = QAction("Info", self);
        act_info.triggered.connect(self._show_info)
        help_menu.addAction(act_info)

    # -------------------------------------------------------------------------
    # Protocol save / load / clear
    # -------------------------------------------------------------------------

    def _save_protocol(self):
        """Save self.trials to a semicolon-delimited .txt file with a header row."""
        if not self.trials:
            QMessageBox.warning(self, "No Trials", "No trials to save.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Protocol", "", "Protocol files (*.txt);;All files (*)"
        )
        if not path:
            return
        if not path.endswith('.txt'):
            path += '.txt'
        try:
            with open(path, 'w') as f:
                f.write(';'.join(FIELDS) + '\n')
                for t in self.trials:
                    f.write(';'.join(str(float(t[field])) for field in FIELDS) + '\n')
            self._log(f"Protocol saved: {os.path.basename(path)}", "ok")
        except Exception as e:
            QMessageBox.critical(self, "Save Error", str(e))

    def _load_protocol(self):
        """Load trials from a .txt file previously saved by _save_protocol()."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Protocol", "", "Protocol files (*.txt);;All files (*)"
        )
        if not path:
            return
        try:
            added = 0
            with open(path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith(FIELDS[0]):
                        continue  # skip blank lines and the header row
                    parts = [p.strip() for p in line.split(';') if p.strip()]
                    if len(parts) >= len(FIELDS):
                        try:
                            self.trials.append(
                                {field: float(parts[i]) for i, field in enumerate(FIELDS)}
                            )
                            added += 1
                        except ValueError:
                            self._log(f"Skipped malformed line: {line[:40]}", "warn")
                            continue
                    else:
                        self._log(f"Skipped short line ({len(parts)} fields): {line[:40]}", "warn")    
                            
            if added:
                self._refresh_table()
                self._update_timeline()
                self._log(f"Loaded {added} trial(s) from {os.path.basename(path)}", "ok")
            else:
                QMessageBox.warning(self, "Load Error", "No valid trials found in file.")
        except Exception as e:
            QMessageBox.critical(self, "Load Error", str(e))

    def _save_log(self):
        """Save the event log to a plain text file."""
        text = self.log.toPlainText()
        if not text.strip():
            QMessageBox.warning(self, "Empty Log", "Nothing to save.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Log", "", "Text files (*.txt);;All files (*)"
        )
        if not path:
            return
        if not path.endswith('.txt'):
            path += '.txt'
        try:
            with open(path, 'w') as f:
                f.write(text)
            self._log(f"Log saved: {os.path.basename(path)}", "ok")
        except Exception as e:
            QMessageBox.critical(self, "Save Error", str(e))



    def _clear_trials(self):
        if self.trials:
            self.trials.clear()
            self._refresh_table()
            self._update_timeline()
            self._log("All trials cleared.", "warn")
            self._params_sent = False
            
   # -------------------------------------------------------------------------
   # Help/Info
   # -------------------------------------------------------------------------       
            
    def _show_info(self):
        msg = QMessageBox(self)
        msg.setWindowTitle("Info")
        msg.setStyleSheet(f"background:{BG_PANEL}; color:{TEXT};")
        msg.setText(
            "<p><b>Conditioning Setup v2.0</b></p>"
            "<p>Documentation licensed under the<br>"
            "<b>Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)</b></p>"
            "<p><a href='https://github.com/fgmourao' style='color:#44aaff;'>"
            "https://github.com/fgmourao</a></p>"
        )
        msg.setTextFormat(Qt.RichText)
        msg.setTextInteractionFlags(Qt.TextBrowserInteraction)
        msg.exec_()        

    # -------------------------------------------------------------------------
    # UI construction
    # -------------------------------------------------------------------------

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)
        main_layout.addWidget(self._build_header())

        body_lay = QHBoxLayout()
        body_lay.setContentsMargins(0, 0, 0, 0)
        body_lay.setSpacing(15)
        body_lay.addWidget(self._build_left_panel(),  stretch=0)
        body_lay.addWidget(self._build_right_panel(), stretch=1)
        main_layout.addLayout(body_lay)

    def _build_header(self):
        """Top bar: title, port selector, connect button, trial counter, status pill."""
        hdr = QFrame()
        hdr.setFixedHeight(40)
        hdr.setStyleSheet("border: none;")
        lay = QHBoxLayout(hdr)
        lay.setContentsMargins(0, 0, 0, 0)

        title = QLabel("CONDITIONING SETUP")
        title.setStyleSheet(f"color:{TEXT}; font-size:16px; font-weight:bold; letter-spacing:2px;")
        #ver = QLabel("V2.0")
        #ver.setStyleSheet(f"color:{TEXT}; font-size:16px;")
        lay.addWidget(title)
        #lay.addWidget(ver)

        img_path = resource_path("image.png")

        lbl_img  = QLabel()
        pixmap   = QPixmap(img_path)
        pixmap   = pixmap.scaledToHeight(40, Qt.SmoothTransformation)
        lbl_img.setPixmap(pixmap)
        lbl_img.setStyleSheet("border:none;")
        lay.addWidget(lbl_img)
        
        lay.addStretch()
        
        
        lbl_port = QLabel("Port:")
        lbl_port.setStyleSheet(f"color:{DIM}; font-size:11px;")

        self.cmb_port = QComboBox()
        self.cmb_port.setFixedWidth(150)
        self.cmb_port.setStyleSheet(
            f"background:{BG_INPUT}; border:1px solid {BORDER}; padding:2px;"
        )
        self._refresh_ports()

        btn_refresh = QPushButton("⟳")
        btn_refresh.setFixedSize(26, 26)
        btn_refresh.setToolTip("Refresh serial ports")
        btn_refresh.setStyleSheet(
            f"QPushButton {{ background:transparent; color:{DIM}; border:1px solid {BORDER}; "
            f"font-size:14px; padding:0px; }}"
            f"QPushButton:hover {{ color:{TEXT}; border-color:{TEXT}; }}"
        )
        btn_refresh.setCursor(Qt.PointingHandCursor)
        btn_refresh.clicked.connect(self._refresh_ports)

        self.btn_connect = make_button("CONNECT", TEXT, BORDER)
        self.btn_connect.setFixedHeight(26)
        self.btn_connect.clicked.connect(self._toggle_connect)

        lay.addWidget(lbl_port)
        lay.addWidget(self.cmb_port)
        lay.addWidget(btn_refresh)
        lay.addWidget(self.btn_connect)
        lay.addSpacing(30)

        self.lbl_trial = QLabel("TRIAL 0/0")
        self.lbl_trial.setStyleSheet(
            f"color:{DIM}; font-size:11px; font-family:'Courier New';"
        )
        self.lbl_status = QLabel("IDLE")
        self.lbl_status.setStyleSheet(f"color:{DIM}; border:none; font-weight:bold; padding:4px 6px;")

        lay.addWidget(self.lbl_trial)
        lay.addWidget(self.lbl_status)
        return hdr

    def _build_left_panel(self):
        """Left column: trial configuration inputs and control buttons."""

        # Outer container with fixed width
        outer = QWidget()
        outer.setMinimumWidth(300)
        outer.setMaximumWidth(380)
        outer_lay = QVBoxLayout(outer)
        outer_lay.setContentsMargins(0, 0, 0, 0)
        outer_lay.setSpacing(0)
    
        # Scrollable inner panel -- allows vertical scroll on small screens
        from PyQt5.QtWidgets import QScrollArea
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setStyleSheet(
            f"QScrollArea {{ border:none; background:{BG}; }}"
            f"QScrollBar:vertical {{ background:{BG_PANEL}; width:6px; border:none; }}"
            f"QScrollBar::handle:vertical {{ background:{BORDER}; border-radius:3px; }}"
            f"QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height:0px; }}"
        )
    
        panel = QWidget()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(15)

        cfg_frm, cfg_lay = make_section_frame("TRIAL CONFIGURATION")

        # Timing
        r, self.e_baseline = make_input_row("Baseline",             0,  "s"); r.setToolTip("Quiet period before the first trial begins (seconds)."); cfg_lay.addWidget(r)
        r, self.e_silence  = make_input_row("Inter-Trial Interval", 0,  "s"); r.setToolTip("Silence between consecutive trials (seconds)."); cfg_lay.addWidget(r)
        cfg_lay.addSpacing(10)

        # Sound
        cfg_lay.addWidget(QLabel(f"<span style='color:{ACC_BLUE}; font-size:13px; font-weight:bold;'>SOUND</span>"))
        r, self.e_snd_onset = make_input_row("Onset",        0,    "s");  r.setToolTip("Time from trial start to sound onset (seconds)."); cfg_lay.addWidget(r)
        r, self.e_snd_dur   = make_input_row("Duration",     10,   "s");  r.setToolTip("Duration of the sound stimulus (seconds)."); cfg_lay.addWidget(r)
        r, self.e_carrier   = make_input_row("Carrier freq", 3000, "Hz"); r.setToolTip("Carrier tone frequency (Hz). Recommended range: 1000 – 20000 Hz."); cfg_lay.addWidget(r)
        r, self.e_modulator = make_input_row("Modulator",    0,   "Hz"); r.setToolTip("AM modulator frequency (Hz). Set to 0 for a pure sine tone."); cfg_lay.addWidget(r)
        r, self.e_volume    = make_input_row("Volume",       100,  "%");  r.setToolTip("Sound amplitude as a percentage of DAC full scale (0 – 100 %)."); cfg_lay.addWidget(r)
        cfg_lay.addSpacing(10)

        # LIGHT
        cfg_lay.addWidget(QLabel(f"<span style='color:{ACC_YELL}; font-size:13px; font-weight:bold;'>LIGHT</span>"))
        r, self.e_light_onset = make_input_row("Onset",     0,  "s");  r.setToolTip("Time from trial start to light onset (seconds)."); cfg_lay.addWidget(r)
        r, self.e_light_dur   = make_input_row("Duration",  10, "s");  r.setToolTip("Duration of the light stimulus (seconds)."); cfg_lay.addWidget(r)
        r, self.e_light_freq  = make_input_row("Frequency", 10, "Hz"); r.setToolTip("Light square wave frequency (Hz) at 50 % duty cycle.\nSet to 9999 for constant DC HIGH — light stays ON continuously."); cfg_lay.addWidget(r)
        cfg_lay.addSpacing(10)

        # Shock
        cfg_lay.addWidget(QLabel(f"<span style='color:{ACC_RED}; font-size:13px; font-weight:bold;'>SHOCK</span>"))
        r, self.e_shk_onset = make_input_row("Onset",      8,  "s");  r.setToolTip("Time from trial start to shock onset (seconds)."); cfg_lay.addWidget(r)
        r, self.e_shk_dur   = make_input_row("Duration",   2,  "s");  r.setToolTip("Total shock duration (seconds). Bars cycle in random sequence for this period."); cfg_lay.addWidget(r)
        r, self.e_pulse_hi  = make_input_row("Pulse HIGH", 20, "ms"); r.setToolTip("Duration each shock bar stays ON per pulse (ms).\nResolution: 0.1 ms. Default: 20 ms."); cfg_lay.addWidget(r)
        r, self.e_pulse_lo  = make_input_row("Pulse LOW",  20, "ms"); r.setToolTip("Inter-pulse interval between consecutive shock pulses (ms).\nResolution: 0.1 ms. Default: 20 ms."); cfg_lay.addWidget(r)
        cfg_lay.addSpacing(10)

        # Trigger 1
        cfg_lay.addWidget(QLabel(f"<span style='color:{ACC_TRG1}; font-size:13px; font-weight:bold;'>TRIGGER 1</span>"))
        r, self.e_trig1_onset = make_input_row("Onset",    0,  "s");  r.setToolTip("Time from trial start to Trigger 1 pulse onset (seconds)."); cfg_lay.addWidget(r)
        r, self.e_trig1_dur   = make_input_row("Duration", 10, "ms"); r.setToolTip("Trigger 1 digital pulse duration (ms). Set to 0 to disable.\nPin 10 goes HIGH for this duration — use to synchronise cameras or recording systems."); cfg_lay.addWidget(r)
        cfg_lay.addSpacing(10)

        # Trigger 2
        cfg_lay.addWidget(QLabel(f"<span style='color:{ACC_TRG2}; font-size:13px; font-weight:bold;'>TRIGGER 2</span>"))
        r, self.e_trig2_onset = make_input_row("Onset",    0,  "s");  r.setToolTip("Time from trial start to Trigger 2 pulse onset (seconds)."); cfg_lay.addWidget(r)
        r, self.e_trig2_dur   = make_input_row("Duration", 10, "ms"); r.setToolTip("Trigger 2 digital pulse duration (ms). Set to 0 to disable.\nPin 11 goes HIGH for this duration — use to synchronise external stimulators."); cfg_lay.addWidget(r)
        cfg_lay.addSpacing(10)

        btn_add = make_button("+ ADD TRIAL", TEXT, BORDER)
        btn_add.clicked.connect(self._add_trial)
        cfg_lay.addWidget(btn_add)
        lay.addWidget(cfg_frm)

        # Controls
        ctrl_frm, ctrl_lay = make_section_frame("CONTROLS")
        btn_send = make_button("SEND PARAMETERS", TEXT, BORDER)
        btn_send.setStyleSheet(btn_send.styleSheet().replace("font-size:11px", "font-size:13px"))
        btn_send.clicked.connect(self._send_params)
        ctrl_lay.addWidget(btn_send)

        btn_row   = QHBoxLayout()
        btn_start = make_button("START", TEXT, BORDER)
        btn_start.setStyleSheet(btn_start.styleSheet().replace("font-size:11px", "font-size:13px"))
        btn_start.clicked.connect(self._send_start)
        btn_abort = make_button("ABORT", ACC_RED, ACC_RED)
        btn_abort.setStyleSheet(btn_abort.styleSheet().replace("font-size:11px", "font-size:13px"))
        btn_abort.clicked.connect(self._send_abort)
        btn_row.addWidget(btn_start)
        btn_row.addWidget(btn_abort)
        ctrl_lay.addLayout(btn_row)
        lay.addWidget(ctrl_frm)

        lay.addStretch()
        scroll.setWidget(panel)
        outer_lay.addWidget(scroll)
        return outer

    def _build_right_panel(self):
        from PyQt5.QtWidgets import QScrollArea
    
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setStyleSheet(
            f"QScrollArea {{ border:none; background:{BG}; }}"
            f"QScrollBar:vertical {{ background:{BG_PANEL}; width:6px; border:none; }}"
            f"QScrollBar::handle:vertical {{ background:{BORDER}; border-radius:3px; }}"
            f"QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height:0px; }}"
            f"QScrollBar:horizontal {{ background:{BG_PANEL}; height:6px; border:none; }}"
            f"QScrollBar::handle:horizontal {{ background:{BORDER}; border-radius:3px; }}"
            f"QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{ width:0px; }}"
        )
    
        panel = QWidget()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)
    
        # Recorded trials table
        tbl_frm, tbl_lay = make_section_frame("RECORDED TRIALS")
        cols = ["#", "BASELINE", "ITI",
                "SND ON", "SND DUR", "CARRIER", "MOD", "VOL",
                "SHK ON", "SHK DUR", "PLS HI", "PLS LO",
                "LIGHT ON", "LIGHT DUR", "LIGHT HZ",
                "TRG1 ON", "TRG1 MS", "TRG2 ON", "TRG2 MS", "DEL"]
        self.tbl = QTableWidget(0, len(cols))
        self.tbl.setHorizontalHeaderLabels(cols)
        self.tbl.setStyleSheet(
            f"QTableWidget {{ background:{BG_PANEL}; color:{TEXT}; "
            f"gridline-color:{BG}; border:none; font-size:11px; }}"
            f"QHeaderView::section {{ background:{BG}; color:{TEXT}; "
            f"border:none; padding:4px; font-size:10px; }}"
        )
        # Stretch columns when space is available; scroll when window is too narrow
        header = self.tbl.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        header.setMinimumSectionSize(50)
        self.tbl.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tbl.setFixedHeight(240)
        self.tbl.clicked.connect(self._on_table_click)
        tbl_lay.addWidget(self.tbl)
        tbl_lay.addStretch()
        lay.addWidget(tbl_frm)
    
        # Timeline preview
        timing_frm, timing_lay = make_section_frame("TRIAL TIMELINE PREVIEW")
        self.timing_widget = TimingWidget()
        timing_lay.addWidget(self.timing_widget)
        lay.addWidget(timing_frm)
    
        # Event log
        log_frm, log_lay = make_section_frame("EVENT LOG")
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(120)
        self.log.setStyleSheet(
            f"background:{BG_PANEL}; color:{TEXT}; border:none; "
            f"font-family:'Courier New'; font-size:11px;"
        )
        log_lay.addWidget(self.log)
        lay.addWidget(log_frm)
    
        scroll.setWidget(panel)
        return scroll

    # -------------------------------------------------------------------------
    # Timeline
    # -------------------------------------------------------------------------

    def _on_table_click(self, index):
        """Clicking any cell refreshes the full session timeline."""
        self.timing_widget.set_trials(self.trials)

    def _update_timeline(self):
        """Push the current trial list to the timeline widget."""
        self.timing_widget.set_trials(self.trials)

    # -------------------------------------------------------------------------
    # Connection management
    # -------------------------------------------------------------------------

    def _set_connect_style(self):
        """Restore the CONNECT button to its default grey/boxed appearance."""
        self.btn_connect.setStyleSheet(
            f"QPushButton {{ background:transparent; color:{TEXT}; "
            f"border:1px solid {BORDER}; "
            f"font-size:11px; font-weight:bold; padding:8px 12px; }}"
            f"QPushButton:hover {{ background:rgba(255,255,255,0.05); }}"
            f"QPushButton:pressed {{ background:rgba(255,255,255,0.1); }}"
        )

    def _refresh_ports(self):
        """Populate the port combo box with currently available serial ports."""
        self.cmb_port.clear()
        for p in serial.tools.list_ports.comports():
            self.cmb_port.addItem(p.device)

    def _toggle_connect(self):
        """Connect if disconnected; disconnect if connected."""
        if self._connected:
            self._disconnect()
        else:
            port = self.cmb_port.currentText().strip()
            if not port:
                return
            # Show CONNECTING... immediately; SerialThread emits ready() after
            # the 1 s stabilisation delay and 5 s init window.
            self.btn_connect.setText("CONNECTING...")
            self.btn_connect.setStyleSheet(
                f"background:transparent; color:{ACC_YELL}; border:none; "
                f"font-size:11px; font-weight:bold; padding:8px 12px;"
            )
            self.btn_connect.setEnabled(False)

            self.serial_thread = SerialThread(port)
            self.serial_thread.data_received.connect(self._on_data)
            self.serial_thread.connection_error.connect(self._on_serial_error)
            self.serial_thread.ready.connect(self._on_serial_ready)
            self.serial_thread.start()

    def _on_serial_ready(self):
        """
        Called when SerialThread.ready is emitted (init window complete).
        Switches the button to DISCONNECT, starts the poll timer, and
        arms a 4 s watchdog that warns the user if no DUE response arrives.
        """
        self._connected = True
        self.btn_connect.setText("DISCONNECT")
        self.btn_connect.setStyleSheet(
            f"QPushButton {{ background:transparent; color:{ACC_RED}; "
            f"border:1px solid {ACC_RED}; "
            f"font-size:11px; font-weight:bold; padding:8px 12px; }}"
            f"QPushButton:hover {{ background:rgba(255,255,255,0.05); }}"
            f"QPushButton:pressed {{ background:rgba(255,255,255,0.1); }}"
        )
        self.btn_connect.setEnabled(True)
        self._log("Connected.", "ok")

        self.serial_thread.send({"cmd": "ping"})
        QTimer.singleShot(500, self._poll_timer.start)

        # Warn the user if the DUE has not responded within 4 s (wrong port).
        self._ready_confirmed = False
        QTimer.singleShot(4000, self._check_due_response)
        self._experiment_done_logged = True  # Suppress spurious "Experiment completed" on first poll:
        # DUE may report status=DONE (3) from a previous session immediately after connect.
        
    def _check_due_response(self):
        """Show a warning if the DUE did not respond within 4 s of connecting."""
        if self._connected and not self._ready_confirmed:
            QMessageBox.warning(
                self, "No Response",
                "No response from Conditioning Setup.\n\nPossible causes:\n"
                "- Wrong port selected\n"
                "- Firmware not uploaded\n"
                "- Wrong USB port on the DUE board"
            )

    def _disconnect(self):
        """Stop the serial thread, reset the UI, and stop all timers."""
        self._poll_timer.stop()
        self._stop_progress()
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread = None
        self._connected = False
        self.btn_connect.setText("CONNECT")
        self._set_connect_style()
        self.btn_connect.setEnabled(True)
        self._set_status_pill(0)
        self._log("Disconnected.", "warn")

    def _on_serial_error(self, err):
        """Handle a serial exception: show popup and reset connection state.
        NOTE: if the USB cable is disconnected during an experiment, the DUE
        continues autonomously. The only way to stop it is the hardware ABORT
        button (pin 48) on the DUE board."""
        self._poll_timer.stop()
        self._stop_progress()
        self._connected = False
        self.btn_connect.setText("CONNECT")
        self._set_connect_style()
        self.btn_connect.setEnabled(True)
        self._set_status_pill(0)
        QMessageBox.critical(
            self, "Connection Error",
            f"Lost connection to DUE:\n\n{err}\n\nCheck the USB cable and port selection."
        )
        self._log(f"Connection error: {err}", "err")

    # -------------------------------------------------------------------------
    # Incoming data
    # -------------------------------------------------------------------------

    def _on_data(self, d):
        """
        Process one JSON object received from the DUE. Three cases:
          "status" key  -- experiment state update (response to poll)
          ok = false    -- error message from DUE firmware
          "msg" key     -- informational string (e.g. "5 trials programmed")
        """
        self._ready_confirmed = True  # Any response confirms DUE is alive

        if "status" in d:
            self._set_status_pill(d.get("status", 0))
            run   = d.get("running", False)
            t_cur = d.get("trial",   0)
            t_tot = d.get("total",   0)
            if run:
                # DUE trial index is 0-based; display as 1-based
                self.lbl_trial.setText(f"TRIAL {t_cur+1}/{t_tot}")
            elif self._programmed_total > 0:
                self.lbl_trial.setText(f"TRIAL 1/{self._programmed_total}")
            else:
                self.lbl_trial.setText("TRIAL 0/0")
            if d.get("fault"):
                self._log("WATCHDOG FAULT - experiment aborted.", "err")
                QMessageBox.critical(self, "Hardware Fault",
                    "Watchdog fault detected.\n\nAll stimuli have been stopped.\n"
                    "Check hardware connections before continuing.")
            if not run:
                self._stop_progress()
                self._poll_timer.start()  # Resume polling after experiment ends
                if d.get("status", 0) == 3 and not self._experiment_done_logged:
                    self._experiment_done_logged = True
                    self._log("Experiment completed.", "ok")
                    
        elif not d.get("ok", True):
            self._log(d.get("msg", "Error from DUE"), "err")

        elif "msg" in d:
            if d["msg"] == "pong":
                version = d.get("version", "unknown")
                if version != "2.0":
                    QMessageBox.warning(self, "Firmware Version Mismatch",
                        f"Expected firmware v2.0, connected to v{version}.\n\n"
                        "Update firmware before running experiments.")
            else:
                self._log(d["msg"], "ok")
                # Update trial counter immediately when program is acknowledged
                if "trials programmed" in d["msg"]:
                    try:
                        n = int(d["msg"].split()[0])
                        self._programmed_total = n
                        self.lbl_trial.setText(f"TRIAL 1/{n}")
                        self._params_sent = True  # DUE confirmed receipt
                    except ValueError:
                        pass

    def _set_status_pill(self, st):
        """Update the status label for DUE status code st."""
        states = {
            0: (DIM,      "IDLE"),
            1: (ACC_BLUE, "READY"),
            2: (ACC_BLUE, "RUNNING"),
            3: (ACC_BLUE, "DONE"),
            4: (ACC_RED,  "FAULT"),
            5: (ACC_RED,  "ABORTED"),
        }
        color, text = states.get(st, (DIM, "IDLE"))
        self.lbl_status.setText(text)
        self.lbl_status.setStyleSheet(
            f"color:{color}; border:none; font-weight:bold; padding:4px 6px;"
        )

    # -------------------------------------------------------------------------
    # Trial management
    # -------------------------------------------------------------------------

    def _get_float(self, edit, default=0.0):
        """Read a float from a QLineEdit; return default on parse failure."""
        try:
            return float(edit.text())
        except ValueError:
            return default

    def _add_trial(self):
        """Read all input fields, append a trial dict, and refresh the UI."""
        t = {
            "baseline":         self._get_float(self.e_baseline),
            "silence":          self._get_float(self.e_silence),
            "onset_sound":      self._get_float(self.e_snd_onset),
            "sound_duration":   self._get_float(self.e_snd_dur),
            "carrier_freq":     self._get_float(self.e_carrier),
            "modulator_freq":   self._get_float(self.e_modulator),
            "volume":           self._get_float(self.e_volume),
            "waveform_type":    0.0,  # SINE_AM; no selector in this version
            "onset_shock":      self._get_float(self.e_shk_onset),
            "shock_duration":   self._get_float(self.e_shk_dur),
            "pulse_high":       self._get_float(self.e_pulse_hi),
            "pulse_low":        self._get_float(self.e_pulse_lo),
            "onset_light":      self._get_float(self.e_light_onset),
            "light_duration":   self._get_float(self.e_light_dur),
            "light_freq":       self._get_float(self.e_light_freq),
            "bar_select":       0.0,  # Always random sequence for experiment trials
            "onset_trig1":      self._get_float(self.e_trig1_onset),
            "trig1_duration":   self._get_float(self.e_trig1_dur),
            "onset_trig2":      self._get_float(self.e_trig2_onset),
            "trig2_duration":   self._get_float(self.e_trig2_dur),
        }
        self.trials.append(t)
        self._refresh_table()
        self._update_timeline()
        self._log(f"Trial {len(self.trials)} added.", "info")

    def _refresh_table(self):
        """Rebuild the recorded trials table from self.trials."""
        self.tbl.setUpdatesEnabled(False)
        self.tbl.setRowCount(0)
        for i, t in enumerate(self.trials):
            vals = [
                str(i + 1),
                f"{t['baseline']}",     f"{t['silence']}",
                f"{t['onset_sound']}",  f"{t['sound_duration']}",
                f"{t['carrier_freq']}", f"{t['modulator_freq']}", f"{t['volume']}",
                f"{t['onset_shock']}",  f"{t['shock_duration']}",
                f"{t['pulse_high']}",   f"{t['pulse_low']}",
                f"{t['onset_light']}",    f"{t['light_duration']}",   f"{t['light_freq']}",
                f"{t.get('onset_trig1',0)}", f"{t.get('trig1_duration',0)}",
                f"{t.get('onset_trig2',0)}", f"{t.get('trig2_duration',0)}",
            ]
            self.tbl.insertRow(i)
            for j, val in enumerate(vals):
                item = QTableWidgetItem(val)
                item.setTextAlignment(Qt.AlignCenter)
                self.tbl.setItem(i, j, item)

            btn_del = QPushButton("del")
            btn_del.setStyleSheet(
                f"background:transparent; color:{ACC_RED}; "
                f"border:1px solid {ACC_RED}; font-size:10px; padding:2px;"
            )
            btn_del.clicked.connect(lambda _, idx=i: self._del_trial(idx))
            self.tbl.setCellWidget(i, len(vals), btn_del)

        self.tbl.setUpdatesEnabled(True)

    def _del_trial(self, idx):
        """Remove trial at idx and refresh the UI."""
        if 0 <= idx < len(self.trials):
            self.trials.pop(idx)
            self._refresh_table()
            self._update_timeline()

    # -------------------------------------------------------------------------
    # DUE commands
    # -------------------------------------------------------------------------

    def _send_params(self):
        """
        Serialise self.trials and send the "program" command to the DUE.
        Data format: N * 20 floats in FIELDS order, semicolon-separated,
        row-major (all fields of trial 0, then trial 1, etc.).
        """
        if not self._connected:
            self._log("Not connected.", "err"); return
        if not self.trials:
            self._log("No trials to send.", "warn"); return
        values = []
        for t in self.trials:
            values.extend(str(float(t[f])) for f in FIELDS)
        self.serial_thread.send({
            "cmd": "program",
            "n":   len(self.trials),
            "data": ";".join(values),
        })
        self._log(f"Sending {len(self.trials)} trial(s) to device...", "info")
        
        self._params_sent = False  # Reset until DUE acknowledges
        
    def _send_start(self):
        """Send the start command and begin the timeline progress line."""
        if not self._connected: return
        if self._progress_timer.isActive(): return  # Already running -- ignore double-click
        if not self.trials:
            self._log("No trials programmed. Use SEND PARAMETERS first.", "warn")
            return
        if not self._params_sent:
            self._log("Parameters not yet sent to device. Use SEND PARAMETERS first.", "warn")
            return
        
        # --- Confirmation dialog before sending start ---
        dlg = QDialog(self)
        dlg.setWindowTitle("Confirm Start")
        dlg.setFixedWidth(320)
        dlg.setStyleSheet(f"QDialog {{ background:{BG}; border:1px solid {BORDER}; }}")
        dlg_lay = QVBoxLayout(dlg)
        dlg_lay.setContentsMargins(28, 28, 28, 24)
        dlg_lay.setSpacing(20)

        lbl_main = QLabel("Were the trials sent correctly?")
        lbl_main.setAlignment(Qt.AlignCenter)
        lbl_main.setStyleSheet(
            f"color:{TEXT}; font-size:13px; font-weight:bold; border:none;"
        )
        dlg_lay.addWidget(lbl_main)

        lbl_sub = QLabel("Can we begin?")
        lbl_sub.setAlignment(Qt.AlignCenter)
        lbl_sub.setStyleSheet(f"color:{DIM}; font-size:11px; border:none;")
        dlg_lay.addWidget(lbl_sub)

        btn_row_dlg = QHBoxLayout()
        btn_row_dlg.setSpacing(12)
        btn_yes = make_button("YES — START", ACC_BLUE, ACC_BLUE)
        btn_no  = make_button("NO — CANCEL", ACC_RED,  ACC_RED)
        btn_yes.clicked.connect(dlg.accept)
        btn_no.clicked.connect(dlg.reject)
        btn_row_dlg.addWidget(btn_yes)
        btn_row_dlg.addWidget(btn_no)
        dlg_lay.addLayout(btn_row_dlg)

        if dlg.exec_() != QDialog.Accepted:
            self._log("Start cancelled by user.", "warn")
            return
        # ------------------------------------------------
    
        self.serial_thread.send({"cmd": "start"})
        self._log("Start command sent.", "ok")
        self._experiment_start = time.time()
        self._progress_timer.start()
        self._poll_timer.stop()
        self._poll_generation += 1
        self._experiment_done_logged = False
        self._schedule_trial_polls()  # Schedule status polls at each trial onset

    def _send_abort(self):
        """Send the abort command and stop the progress line."""
        if not self._connected: return
        self.serial_thread.send({"cmd": "abort"})
        self._log("ABORT sent.", "err")
        self._poll_generation += 1  # Invalidate all pending singleShot polls
        self._stop_progress()
        self._poll_timer.start()  # Resume polling after abort
        
    def _schedule_trial_polls(self):
        """Schedule one status poll at the start of each trial.
        Uses _poll_generation to suppress orphaned singleShots after ABORT.
        NOTE: polls are scheduled relative to _send_start() time. USB latency
        and firmware delay(50ms) introduce ~100ms initial offset; in long
        experiments cumulative drift may reach several hundred ms.
        Timing reference for stimuli is always the DUE sync pins, not these polls."""
        gen = self._poll_generation  # capture current generation
        cursor = 0.0
        for t in self.trials:
            cursor += t.get('baseline', 0) + t.get('silence', 0)
            delay_ms = int(cursor * 1000) + 100
            QTimer.singleShot(delay_ms, lambda g=gen: self._poll_if_current(g))
            s_end  = t.get('onset_sound', 0) + t.get('sound_duration', 0)
            k_end  = t.get('onset_shock',  0) + t.get('shock_duration',  0)
            l_end  = t.get('onset_light',  0) + t.get('light_duration',  0)
            t1_end = t.get('onset_trig1',  0) + t.get('trig1_duration',  0) / 1000.0
            t2_end = t.get('onset_trig2',  0) + t.get('trig2_duration',  0) / 1000.0
            cursor += max(s_end, k_end, l_end, t1_end, t2_end, 0.0)
            
    def _poll_status(self):
        """Request a status update from the DUE (called by _poll_timer)."""
        if self._connected:
            self.serial_thread.send({"cmd": "status"})

    def _poll_if_current(self, generation):
        """Poll only if generation matches and experiment is not actively running.
    Suppresses polls during active sound playback to avoid USB/Timer4 glitch:
    USB transactions during Timer4 ISR execution cause phase discontinuities
    in the DAC output, audible as clicks approximately 1 second before trial end.
    The trial counter is updated by spontaneous sendStatus() from the firmware
    at each trial onset instead."""
        if self._connected and generation == self._poll_generation:
            if not self._progress_timer.isActive():
                self.serial_thread.send({"cmd": "status"})

    # -------------------------------------------------------------------------
    # Progress line
    # -------------------------------------------------------------------------

    def _update_progress(self):
        """Called every 200 ms to advance the progress line on the timeline."""
        if self._experiment_start is not None:
            self.timing_widget.set_progress(time.time() - self._experiment_start)

    def _stop_progress(self):
        """Stop the progress timer and hide the progress line."""
        self._progress_timer.stop() 
        self._experiment_start = None
        self.timing_widget.set_progress(None)

    # -------------------------------------------------------------------------
    # Event log
    # -------------------------------------------------------------------------

    def _log(self, msg, level="info"):
        """
        Append a timestamped entry to the event log.
        level: "ok" (white), "info" (white), "warn" (yellow-orange), "err" (red).
        """
        colors = {"ok": TEXT, "err": ACC_RED, "warn": ACC_RED, "info": TEXT}
        color  = colors.get(level, TEXT)
        ts     = time.strftime("%Y-%m-%d %H:%M:%S")
        self.log.append(
            f'<span style="color:{DIM}">{ts}</span> &nbsp; '
            f'<span style="color:{color}">{msg}</span>'
        )
        self.log.moveCursor(QTextCursor.End)

    # -------------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------------

    def closeEvent(self, event):
        """Stop all timers and the serial thread cleanly on window close.
        If an experiment is in progress, sends ABORT before closing so the DUE
        does not continue delivering stimuli unattended (shock/sound/light)."""
        if self._progress_timer.isActive() and self.serial_thread is not None:
            self.serial_thread.send({"cmd": "abort"})
            time.sleep(0.1)  # Brief wait for abort command to transmit
        self._poll_timer.stop()
        self._progress_timer.stop()
        if self.serial_thread is not None:
            self.serial_thread.stop()
            self.serial_thread.wait(2000)  # Bounded wait -- avoids hang if thread stalls
        event.accept()


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == '__main__':
    
    # High-DPI support for Windows -- prevents font scaling issues on
    # high-resolution displays and fixes clipped text in buttons and labels.
    if hasattr(Qt, 'AA_EnableHighDpiScaling'):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
        
    # QApplication.instance() returns the existing app when running inside
    # Spyder or Jupyter, preventing a crash on re-run in the same kernel.
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    app.setStyleSheet(TOOLTIP_STYLE)

    window = CageApp()
    window.show()
    app.exec_()