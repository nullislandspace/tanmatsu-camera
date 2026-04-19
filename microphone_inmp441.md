# INMP441 I2S Microphone — Tanmatsu Wiring

Connects an INMP441 MEMS microphone (or a stereo pair) to the Tanmatsu
internal add-on port using a **dedicated second I2S controller** on the
ESP32-P4. This keeps the existing speaker I2S chain (ES8156 on port 0,
GPIO 28/29/30/31) completely untouched.

## Pin selection rationale

Chosen GPIOs: **E8 (GPIO 54), E9 (GPIO 49), E10 (GPIO 53)** — pins 19/20/21
on the internal add-on connector. These three are physically adjacent,
free, unshared, and carry no strapping-pin or pull-up gotchas. The
ESP32-P4 routes I2S signals through the GPIO matrix so any of these can
host BCLK, WS, or DIN.

GPIO 54 multiplexes ANA_CMPR_CH1 and GPIO 53 multiplexes ADC2_CH4 — both
are opt-in analog functions that stay disabled unless explicitly
configured, so they do not interfere with I2S use.

## Mono wiring (single microphone)

| Mic pin | Function             | Connector pin # | Connector name | ESP32-P4 GPIO |
|---------|----------------------|-----------------|----------------|---------------|
| SCK     | bit clock (input)    | 19              | E8             | GPIO 54       |
| WS      | word select (input)  | 20              | E9             | GPIO 49       |
| SD      | serial data (output) | 21              | E10            | GPIO 53       |
| L/R     | channel strap        | 7               | GND            | —             |
| VDD     | 3.3 V supply         | 8               | +3.3V          | —             |
| GND     | ground               | 6               | GND            | —             |

L/R tied to GND makes the microphone drive SD during the **left** half of
the WS frame. In firmware, configure the I2S slot as `MONO_LEFT` (or read
a stereo stream and take the left channel).

## Stereo wiring (two microphones sharing SD) - NOT SUPPORTED BY CAMERA APP

Both microphones share the clock and data lines — no extra GPIOs are
needed beyond the mono wiring. Only the L/R strap differs between the
two mics, which causes each to tri-state its SD output during the other's
half of the WS frame.

| Mic pin | Mic A (left)              | Mic B (right)             |
|---------|---------------------------|---------------------------|
| SCK     | pin 19 (E8, GPIO 54)      | pin 19 (E8, GPIO 54) — shared |
| WS      | pin 20 (E9, GPIO 49)      | pin 20 (E9, GPIO 49) — shared |
| SD      | pin 21 (E10, GPIO 53)     | pin 21 (E10, GPIO 53) — shared |
| L/R     | pin 7 (GND)               | pin 8 (+3.3V)             |
| VDD     | pin 8 (+3.3V)             | pin 33 (+3.3V)            |
| GND     | pin 6 (GND)               | pin 35 (GND)              |

Configure the I2S slot as `STEREO`; the ESP32-P4 receives left samples
from Mic A and right samples from Mic B, automatically interleaved by
the hardware.

## Board / layout notes

- Place a **100 nF ceramic** decoupling capacitor between VDD and GND as
  close to each INMP441 as possible (required by the datasheet).
- Keep the SD trace short; avoid running it parallel to SCK over any
  significant length to limit crosstalk.
- None of the chosen GPIOs collide with the existing speaker I2S
  (port 0, GPIO 28/29/30/31) or with other exposed connector functions.

## Firmware summary

Implemented in `main/microphone.c` / `main/microphone.h`.

- Allocated on `I2S_NUM_1` (explicitly, not `I2S_NUM_AUTO`) so the ES8156
  codec path — which uses `I2S_NUM_0` in the BSP — is reserved for future
  audio playback without conflict.
- RX-only channel, `I2S_ROLE_MASTER`, Philips standard mode.
- `I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO)`.
  The default `slot_mask` for mono is `I2S_STD_SLOT_LEFT`, which matches the
  L/R → GND strap.
- `gpio_cfg`: `.bclk = GPIO_NUM_54`, `.ws = GPIO_NUM_49`,
  `.din = GPIO_NUM_53`, `.mclk = I2S_GPIO_UNUSED`,
  `.dout = I2S_GPIO_UNUSED`.
- **I2S runs at 32 kHz** internally so BCLK = 2.048 MHz, landing exactly on
  the INMP441's spec minimum. A background reader task decimates 2:1 into a
  stream buffer delivering 16 kHz mono int16 PCM to the recorder — which
  keeps the existing AVI/Shine path (also 16 kHz mono) unchanged.
- Peak detection runs over the full 32 kHz input for best level-meter
  response and is exposed as `microphone_peak_level()` (0..32767).
- Lifecycle is driven by the main UI loop: the mic starts on
  MODE_VIDEO entry (when `config.mic_enabled` is set) and stops on exit.
  The HUD shows a green/yellow/red level bar whenever the mic is running.
