<p align="left">
  <img src="image.png" width="80" />
</p>


# Conditioning Setup v2.0
** Under development

Hardware, firmware and desktop interface for classical fear conditioning.
Arduino DUE stimulus generator controlled by a Python/PyQt5 application over USB.

---

## Origin

This project is a full rewrite of the system described in:

> Amaral Junior PA, Mourao FAG, Moraes MFD (2019).
> *A Custom Microcontrolled and Wireless-Operated Chamber for Auditory Fear Conditioning.*
> Frontiers in Neuroscience.
> https://doi.org/10.3389/fnins.2019.01193

The original v1.0 architecture used an ESP8266 as a Wi-Fi master communicating with the Arduino DUE via SPI, with a browser-based HTML interface. Version 2.0 removes the ESP8266 entirely — a PyQt5 desktop application communicates directly with the DUE over its native USB port.

Beyond the architectural change, v2.0 introduces several new capabilities: adaptive DAC lookup tables for lower harmonic distortion across the full frequency range, an AM modulation channel with configurable modulator frequency, two independent programmable digital trigger outputs (pins 10 and 11) for synchronising external equipment, an OLED status display, a hardware watchdog with fault detection, and a calibration interface with per-bar shock selection.


---

## Changes from v1.0 to v2.0

### Architecture
The ESP8266 Wi-Fi master and SPI bus are removed entirely. Python communicates directly with the DUE over its native USB port using newline-terminated JSON. All stimulus timing is handled autonomously by the DUE after a single `start` command — no real-time control from the host is required during execution.

### Bug fixes and stability improvements

**Shock clock decoupled from DAC** — in v1.0 the shock interval counter (`iCountShockIntervals`) was incremented inside the `carrier()` ISR, which ran at the DAC rate. Changing `carrier_freq` silently altered shock pulse timing. In v2.0 the shock has its own dedicated Timer6 running at a fixed 10 kHz, completely independent of the DAC.

**Hardware ABORT pin pullup** — in v1.0 `pinABORT` was configured as `INPUT` with no pullup resistor, leaving the pin floating when the button was not pressed and susceptible to spurious aborts from electrical noise. In v2.0 it uses `INPUT_PULLUP`.

**DAC disabled between sounds** — in v1.0 `analogWrite(DAC1, 0)` at sound offset wrote zero but left the DAC channel enabled, injecting DC noise into the amplifier between trials. In v2.0 `dacc_disable_channel` is called at every sound offset and the coupling capacitor is pre-charged to midscale before each onset to eliminate transients.

**Shock pulse timing precision** — in v1.0 `floor()` was used to convert pulse times to tick counts, introducing a systematic downward truncation error. In v2.0 `roundf()` gives the nearest tick (0.1 ms resolution at 10 kHz).

**Watchdog** — v1.0 had no hardware fault detection. If Timer4 or Timer6 stalled, the experiment continued silently with no stimuli. In v2.0 a software watchdog monitors both timers; if either stops incrementing for 5 seconds a fault is declared, all outputs are driven LOW, and the Python interface displays a critical alert.

**DAC buffer type** — in v1.0 the DAC buffer was `uint32_t` (4 bytes per sample), wasting twice the RAM needed. In v2.0 it is `uint16_t` (2 bytes per sample), halving the buffer footprint to 2 KB.

### Sound generation

In v1.0 the carrier lookup table had a fixed 8 samples regardless of frequency. At low carrier frequencies (1–3 kHz) this produced visible harmonic distortion — only 8 points approximated each sine cycle. In v2.0 adaptive tables select 4, 8, 16, or 32 samples based on the carrier frequency, keeping the DAC update rate within the optimal range across the full 0–20 kHz span.

### New stimuli and features

- **LIGHT** — independent light stimulus (pin 45, square wave or DC HIGH) with its own onset, duration, and frequency per trial
- **Trigger 1 / Trigger 2** — two independent digital pulse outputs (pins 10 and 11) for synchronising external equipment, with configurable onset (s) and duration (ms) per trial
- **Baseline period** — quiet time before the first trial, separate from the inter-trial interval
- **Per-trial volume control** — sound amplitude programmable per trial (0–100%)
- **AM modulation** — configurable modulator frequency per trial; `modulator_freq = 0` gives a pure sine
- **OLED status display** — real-time trial state on an SSD1306 128×32 display
- **Bar selection** — calibration mode allows fixing a single shock bar for individual verification
- **Python desktop interface** — replaces the ESP8266 HTML page with a PyQt5 application featuring a trial timeline preview, protocol save/load, event log, and calibration dialogs

---

## Architecture

```
Python / PyQt5  (PC)
      │
      │  SerialUSB — native USB port (USB CDC)
      │
Arduino DUE
      ├─ DAC1         →  Sound  (AM sine, 0–20 kHz)
      ├─ Timer4/5     →  DAC clock / AM modulator ISR
      ├─ Timer6       →  Shock clock (10 kHz)
      ├─ Timer7       →  Light square wave
      ├─ Pins 23–37   →  Shock bars (8 outputs, round-robin)
      ├─ Pin 45       →  Light output
      ├─ Pin 46       →  Watchdog fault output
      ├─ Pin 48       →  Hardware ABORT input
      ├─ Pin 10        →  Trigger 1 output (digital pulse)
      ├─ Pin 11        →  Trigger 2 output (digital pulse)
      ├─ Pins 50–53   →  Sync outputs (SOUND, LIGHT, SHOCK, MOD)
      └─ Pins 20–21   →  I²C — OLED display (SDA / SCL)
```

The DUE stores the complete trial list in RAM and executes the experiment autonomously after a single `start` command. Python is responsible only for programming parameters, triggering execution, and monitoring status.

---

## Stimuli

Five independent stimuli with individual onset and duration per trial:

### Sound
- Output: DAC1 (12-bit, 0–3.3 V)
- Waveform: AM-modulated sine (`SINE_AM`) or pure sine
- Carrier: 0–20 kHz; modulator: 0–500 Hz
- Adaptive lookup tables (see [Sound generation](#sound-generation))
- AC coupling capacitor (10 µF) recommended in series on DAC1 output

### Light
- Output: pin 45, square wave 50% duty cycle
- Frequency: configurable; `light_freq = 9999` = DC HIGH (constant ON)

### Shock
- Outputs: pins 23, 25, 27, 29, 31, 33, 35, 37 (8 bars)
- Round-robin activation or fixed single bar (calibration)
- Pulse HIGH and LOW times configurable in ms (0.1 ms resolution at 10 kHz clock)

### Trigger 1 / Trigger 2
- Outputs: pin 10 (Trigger 1), pin 11 (Trigger 2)
- Simple digital HIGH pulse: onset in seconds, duration in milliseconds
- Independent of each other and of all other stimuli
- Intended for synchronising external equipment (cameras, stimulators, recording systems)

---

## Trial structure

Each trial stores 20 float parameters, transmitted semicolon-separated over serial:

| Field | Description | Unit |
|---|---|---|
| `baseline` | Quiet period before first trial | s |
| `silence` | Inter-trial interval | s |
| `onset_sound` | Sound onset within trial | s |
| `sound_duration` | Sound duration | s |
| `carrier_freq` | DAC carrier frequency; 0 = no sound | Hz |
| `modulator_freq` | AM modulator frequency; 0 = pure sine | Hz |
| `volume` | Sound amplitude | % |
| `waveform_type` | 0 = SINE_AM, 1 = SINE, 2 = SQUARE | — |
| `onset_shock` | Shock onset within trial | s |
| `shock_duration` | Shock duration; 0 = no shock | s |
| `pulse_high` | Shock bar ON time per pulse | ms |
| `pulse_low` | Shock bar OFF time between pulses | ms |
| `onset_light` | Light onset within trial | s |
| `light_duration` | Light duration; 0 = no light | s |
| `light_freq` | Light frequency; 9999 = DC HIGH | Hz |
| `bar_select` | 0 = round-robin, 1–8 = fixed bar | — |
| `onset_trig1` | Trigger 1 onset within trial; 0 = start of trial | s |
| `trig1_duration` | Trigger 1 pulse duration; 0 = disabled | ms |
| `onset_trig2` | Trigger 2 onset within trial; 0 = start of trial | s |
| `trig2_duration` | Trigger 2 pulse duration; 0 = disabled | ms |

---

## Serial protocol

All communication uses newline-terminated JSON objects:

```
{"cmd":"ping"}
    -> {"ok":true,"msg":"pong","version":"2.0"}

{"cmd":"program","n":N,"data":"f0;f1;..."}
    -> {"ok":true,"msg":"N trials programmed"}

{"cmd":"start"}
    -> {"ok":true,"msg":"started"}

{"cmd":"abort"}
    -> {"ok":true,"msg":"aborted"}

{"cmd":"status"}
    -> {"ok":true,"status":N,"trial":N,"total":N,
        "running":bool,"ready":bool,"fault":bool}
```

Status codes: `0` IDLE · `1` READY · `2` RUNNING · `3` DONE · `4` FAULT · `5` ABORTED

---

## Sync outputs

All sync pins are active HIGH while the corresponding stimulus is active.
**Use these pins as the authoritative timing reference for external equipment** (electrophysiology, cameras). The `onset_*` fields in seconds carry a worst-case jitter of ~500 µs (one loop iteration); the sync pins do not.

| Pin | Signal | Description |
|---|---|---|
| 50 | SOUND_SYN | HIGH while sound is playing |
| 51 | LIGHT_SYN | HIGH while light is active |
| 52 | SHOCK_SYN | HIGH while shock is being delivered |
| 53 | MOD_SYN | Square wave at `modulator_freq`, phase-locked to AM envelope |

---

## OLED status display

Optional SSD1306 128×32 OLED on I²C (SDA = pin 20, SCL = pin 21, address 0x3C).
Shows experiment state at each transition:

| State | Display |
|---|---|
| Startup | `Conditioning Setup / Version 2.0` |
| Trial onset | `TRIAL X/N` / `CS` · `CS + US` · `US` |
| Done | `Conditioning Setup / Done` |
| Aborted | `Conditioning Setup / Aborted` |

The firmware runs normally if the display is not connected.

---

## Sound generation

v2.0 uses adaptive lookup tables (`Waveforms.h`) to minimise harmonic distortion across the full frequency range:

| Carrier frequency | Table size | DAC update rate |
|---|---|---|
| > 10 000 Hz | 4 samples | up to 80 kHz |
| > 3 000 Hz | 8 samples | up to 80 kHz |
| > 1 000 Hz | 16 samples | up to 48 kHz |
| ≤ 1 000 Hz | 32 samples | up to 32 kHz |

All rates are within the SAM3X8E DAC hardware limit of 1 MHz.
Buffer RAM cost: 32 × 32 × 2 bytes = 2 KB.

AM modulation uses a 32-sample unipolar envelope table. When `modulator_freq = 0` the carrier is output as a pure sine with no modulation.

---

## DAC signal conditioning

The DAC1 output is 0–3.3 V (unipolar, midscale = 1.65 V DC). A signal conditioning chain is required before the amplifier or speaker.

### Signal chain

```
DAC1 ── 10 µF (AC coupling) ── [ON-OFF-ON switch] ─┬─ Path A: direct output (external amplifier / measurement)
                                                  └─ Path B: Rs (12 kΩ) ── Rp (1 kΩ to GND)
                                                                     │
                                                                  POT (50 kΩ linear)
                                                                     │ cursor
                                                                  TDA8932 IN+
                                                                     │
                                                               OUT1 ── tweeter ── OUT2
```

### Components

| Component | Value | Purpose |
|---|---|---|
| C1 electrolytic | 10 µF, + toward DAC1 | Blocks 1.65 V DC offset; passes AC signal |
| Rs fixed resistor | 12 kΩ | Attenuates DAC signal to prevent amplifier saturation |
| Rp fixed resistor | 1 kΩ to GND | Anchors amplifier input when DAC is disabled; eliminates idle noise |
| Potentiometer | 50 kΩ linear | Volume control across full rotation range without clipping |
| TDA8932 module | Class-D, 12 V, BTL | Gain 30 dB (31.6×); differential output OUT1/OUT2 |
| Tweeter | 4 Ω, 40 W, 6–20 kHz | Connected differentially between OUT1 and OUT2 |

### ON-OFF-ON switch

A three-position toggle switch after the coupling capacitor selects the signal path:

| Position | Path | Use |
|---|---|---|
| ON (left) | Direct output after C1 | External amplifier, oscilloscope, audio interface |
| OFF (centre) | Mute | No signal at any output |
| ON (right) | Rs → Rp → POT → TDA8932 → tweeter | Normal experimental operation |

---

## Python application

### Requirements

```
pip install pyserial PyQt5
```

Python 3.8 or later.

### Features

- Dark/Light-theme PyQt5 desktop application
- Trial configuration panel: per-stimulus onset, duration, frequency, volume, pulse timing, trigger durations
- Recorded trials table with per-row delete
- Timeline preview: block diagram of all trials on a shared time axis with a real-time progress line during execution; rows are shown only for stimuli with non-zero duration
- Event log with timestamps (`YYYY-MM-DD HH:MM:SS`) — exportable via Save Log
- **Protocol menu:** Save Protocol, Load Protocol, Clear All Trials, Save Log
- **Calibration menu:** Sound, Light, Shock — continuous single-trial stimulus stopped by ABORT; shock calibration allows bar selection and configurable pulse timing
- **Help menu:** Info dialog
- Port selector with refresh button
- 4-second timeout warning if DUE does not respond after connection
- Firmware version check on connect (warns if not v2.0)
- Watchdog fault popup with critical alert
- Hardware ABORT button (pin 48) is read inside the trial timing loop and during inter-trial silences — responsive at any point during experiment execution

### Poll strategy

The poll timer (`{"cmd":"status"}` every 1500 ms) is stopped when the experiment starts to prevent USB interrupt collisions with Timer4 on the Native Port. One poll is scheduled at the calculated onset of each trial using `QTimer.singleShot`. A generation counter invalidates scheduled polls that remain pending after an ABORT. The poll timer resumes when the experiment ends or is aborted.

---

## Hardware requirements

| Component | Specification |
|---|---|
| Microcontroller | Arduino DUE |
| USB connection | Native port (second USB connector) |
| Shock bars | 8 outputs — pins 23, 25, 27, 29, 31, 33, 35, 37 |
| Light output | Pin 45 |
| Trigger 1 output | Pin 10 — digital pulse |
| Trigger 2 output | Pin 11 — digital pulse |
| ABORT button | Momentary switch — pin 48 to GND |
| DAC coupling | 10 µF electrolytic capacitor in series on DAC1 |
| Amplifier | Class-D module (e.g. TDA8932); 10–50 kΩ potentiometer at input |
| Speaker | Tweeter (2–20 kHz); 4 Ω or 8 Ω |
| OLED (optional) | SSD1306 128×32, I²C, address 0x3C |

---

## Files

| File | Description |
|---|---|
| `conditioning_setup_dark.py` | Main application — dark theme |
| `Stimuli_PY_DUE.ino` | Arduino DUE firmware |
| `Waveforms.h` | Adaptive DAC lookup tables |
| `image.png` | Logo displayed in the application header |

Arduino library dependencies (install via Arduino IDE Library Manager):
- **DueTimer** (Ivan Seidel)
- **Adafruit GFX Library**
- **Adafruit SSD1306**

---

## Desktop Executables

Pre-built executables are available for macOS and Windows. No Python installation is required — all dependencies are bundled inside the application package.

### macOS

**File:** `Conditioning Setup.app`

**Compatibility:** macOS 10.15 (Catalina) or later. Built on Apple Silicon (M-series); runs natively on Apple Silicon Macs and via Rosetta 2 on Intel Macs.

**Installation:**
1. Download `Conditioning Setup.app`
2. Move it to your Applications folder or any preferred location
3. On first launch, macOS Gatekeeper may block the app because it is not signed by an Apple-registered developer. To allow it, open Terminal and run:

```bash
xattr -cr "/Applications/Conditioning Setup.app"
```

Then double-click the app to open normally.

**Build environment:**
- macOS 15.7.5 (Sequoia), Apple Silicon (M2)
- Python 3.12.2
- PyInstaller 6.x
- PyQt5 5.15.x, pyserial 3.x

**Build command:**
```bash
pyinstaller --windowed \
    --name "Conditioning Setup" \
    --icon "icon.icns" \
    --add-data "image.png:." \
    conditioning_setup_dark.py
```

---

### Windows

**File:** `Conditioning Setup.exe`

**Compatibility:** Windows 10 and Windows 11 (64-bit). Built on Windows 11; compatible with Windows 10 version 1903 or later.

**Installation:**
1. Download `Conditioning Setup.exe`
2. Place it in any folder
3. On first launch, Windows SmartScreen may display a warning ("Windows protected your PC"). Click **More info** → **Run anyway**. This warning appears because the executable is not digitally signed.

**USB driver:** the Arduino DUE native port uses a standard USB CDC driver. Windows 10 and 11 include this driver natively — no additional installation is required in most cases. If the COM port is not recognised, install the SAM-BA driver from the Arduino IDE or from the Microchip website.

**Build environment:**
- Windows 11, 64-bit
- Python 3.14.5
- PyInstaller 6.20.0
- PyQt5 5.15.x, pyserial 3.x

**Build command:**
```
py -m PyInstaller --onefile --windowed --name "Conditioning Setup" --icon "icon.ico" --add-data "image.png;." --add-data "icon.ico;." conditioning_setup_dark.py
```

---

## Known limitations

- Noise waveforms (white, pink) are not implemented. The lookup-table architecture is periodic; a possible approach is to generate noise buffers in Python (NumPy/SciPy) and send them as custom tables over serial.
- If the USB cable is disconnected during an experiment, the DUE continues executing autonomously. The only way to stop it is the hardware ABORT button (pin 48).

---

## License

Documentation licensed under Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0).

---

## Authors

**v1.0 (2019)**
Paulo Aparecido Amaral Junior, Flavio Afonso Goncalves Mourao, Marcio Flavio Dutra Moraes
*Núcleo de Neurociências, Federal University of Minas Gerais, Brazil*

**v2.0 (2026)**
Flavio Afonso Goncalves Mourao — [mourao.fg@gmail.com](mailto:mourao.fg@gmail.com)
*CNPq/MCTI/FNDCT Nº 21/2024 — Processo 446467/2024-3*
*Federal University of Minas Gerais, Brazil*

---

*Last update: May 2026*
