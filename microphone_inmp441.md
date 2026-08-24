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

## Stereo wiring (two microphones sharing SD)

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
the hardware. **This is what the camera app expects.** It averages the
two slots into the single mono channel the AVI/Shine pipeline encodes —
worth ~3 dB of uncorrelated-noise rejection, and it means the recording
doesn't depend on which mic the user happens to be talking into. The
mono wiring above still works: the unused slot reads as a floating pin
(all-`0xFFFFFFFF` ≈ -1), which averages to a harmless half-LSB offset.

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
- `I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO)`.
  STEREO is **deliberate** even though only one mic is wired — see the
  "Pitfalls" section below for why MONO mode doesn't work as expected on
  the P4 RX path.
- `gpio_cfg`: `.bclk = GPIO_NUM_54`, `.ws = GPIO_NUM_49`,
  `.din = GPIO_NUM_53`, `.mclk = I2S_GPIO_UNUSED`,
  `.dout = I2S_GPIO_UNUSED`.
- **I2S runs at 44.1 kHz** so BCLK = 2.8224 MHz, comfortably inside the
  INMP441's 2.048–4.096 MHz BCLK range. The reader task drains DMA, picks
  both slots, averages them to mono, gain-scales, applies a 2-stage IIR
  low-pass (~6 kHz cutoff — also serves as the
  decimation anti-alias), then keeps every other sample to halve the
  rate to **22.05 kHz mono** — the exact rate Shine encodes and the AVI
  file advertises. No fractional resampler, no rate drift.
- DMA: 16 descriptors × 256 frames = ~93 ms of buffered audio. Each
  `i2s_channel_read` consumes exactly one descriptor (2048 bytes); see
  "Pitfalls" for why this is non-negotiable.
- Reader task `mic_rx` runs at FreeRTOS priority **7**, above the video
  encoder (`vid_rec`, prio 6) and audio encoder (`aud_rec`, prio 6) so
  the I2S DMA reader is never preempted by the heavy encode work.
- Telemetry: 5-second log line `rate: raw=N Hz emitted=N Hz (cfg=N Hz)
  drops=N` lets you spot rate-mismatch or DMA-overflow regressions
  without instrumentation. Healthy operation reads `raw≈88200`,
  `emitted≈22050`, `drops=0`.
- An `on_recv_q_ovf` ISR callback increments the `drops` counter when
  the IDF's RX queue overflows (otherwise the IDF silently discards
  the oldest descriptor without warning).
- Peak detection runs over the full 44.1 kHz input (before decimation)
  for best level-meter response and is exposed as
  `microphone_peak_level()` (0..32767).
- Gain staging: the slots are 24-bit samples **left-justified** in the
  32-bit word, so `raw >> 8` recovers the true signed 24-bit value
  (±2²³ at full scale) and a further `>> 8` would be 0 dB into int16.
  The `mic_gain` setting is a *step* on a ~5.7 dB ladder (1× = unity at
  step 1, up to 100× at step 8), applied as `(sample24 * mult) >> 8`.
  The ladder is calibrated by replaying a `microphone_debug_raw_dump()`
  capture through the real chain, **not** from datasheet sensitivity —
  the two disagree by ~30 dB in near-field use (a quiet-speech dump
  peaked at -19.4 dBFS, which the datasheet's -26 dBFS @ 94 dB SPL
  would put at ~91 dB SPL). Re-measure if the mic hardware changes.
- A **60 Hz DC-blocking high-pass** (2 cascaded one-pole stages, Q30)
  runs on the 24-bit sample *before* the gain stage. See the
  "Sub-20 Hz rumble" pitfall below for why this is load-bearing.
- Lifecycle is driven by the main UI loop: the mic starts on
  MODE_VIDEO entry (when `config.mic_enabled` is set) and stops on exit.
  The HUD shows a green/yellow/red level bar whenever the mic is running.

## Pitfalls discovered the hard way

These all caused real bugs that wasted hours of debugging. Document them
so the next person doesn't fall in.

### MONO slot mask doesn't filter on P4 RX

The IDF's `I2S_STD_SLOT_LEFT` mask is documented to deliver only the
LEFT slot's data, but on the ESP32-P4 RX path it does **not** filter —
both slots arrive interleaved in DMA, with the RIGHT slot carrying
whatever the floating DIN pin produced (typically all-`0xFFFFFFFF`
because the INMP441 tri-states during the unused half-frame).

Workaround: configure STEREO explicitly and handle the slots in
software. The downstream cost is negligible — the IIR + decimator
already iterate the buffer. With the stereo mic pair this stopped being
a workaround and became the correct configuration anyway.

### Left-justified slots: the +48 dB that hid inside a shift

The INMP441 clocks 24 data bits MSB-first into a 32-bit slot and then
stops driving, so the I2S peripheral hands you the sample
**left-justified**: bits [31:8] are the signed 24-bit value, bits [7:0]
are trailing garbage. Converting that to int16 is `>> 16`.

The driver originally did `(raw >> 8) * gain` and clamped the result to
int16 — treating a ±2²³ value as if it were already int16, i.e. baking
in a fixed **256× (+48 dB)** before the user's gain multiplier even
applied. This went unnoticed for a long time because the prototype PCB's
microphone read roughly 17 dB low, so the two errors partially cancelled
and the level meter looked plausible. Swapping in healthy mics made it
obvious: ordinary speech clipped by ~11 dB even at the lowest gain
setting, and the recording sounded badly overdriven.

The tell is in the raw dump — normal speech showing peaks around ±1M
means a 24-bit sample of only ±3900 (-66 dBFS), which is far too quiet
for a part rated -26 dBFS @ 94 dB SPL. If the extraction shift looks
right but the levels are wildly off, check which justification you are
actually getting before reaching for the gain knob.

### Sub-20 Hz rumble is louder than the audio

A raw capture from the stereo pair had **99.6 % of its total power below
20 Hz**, at -22 dBFS — roughly **19 dB above** the voice band (-41 dBFS).
Amplifying that ahead of the int16 clamp means clipping on rumble long
before speech ever gets loud.

It is mechanical, not acoustic. The evidence:

| band | L/R correlation | reading |
|------|-----------------|---------|
| < 20 Hz    | **-0.41**, flat across lags | anti-phase, no propagation delay → not a sound field |
| 300 Hz-3 kHz | **+0.986** at -2 samples    | same source, ~15 mm path difference → both mics healthy |
| 3-8 kHz    | +0.85 at -2 samples         | same |

Two bottom-port mics sharing a PCB see board flex in *opposite* phase,
which is exactly this signature. A genuine sub-bass acoustic wave (17 m
wavelength) would arrive at both essentially in phase.

The INMP441's own high-pass does not help — the datasheet puts its
corner at -3 dB / 3.7 Hz at 48 kHz (≈3.4 Hz at 44.1 kHz) and only
-0.5 dB at 10.4 Hz. Hence the driver's own 60 Hz DC blocker. 60 Hz is
free acoustically: it is the -3 dB corner of the INMP441's MEMS
transducer itself, so nothing the part reproduces faithfully is lost.

The L+R mono average is worth ~5 dB against this on its own (it partly
cancels anti-correlated content while leaving the correlated voice
intact) — useful, but nowhere near sufficient by itself.

### Missing 100 kΩ pull-down on SD

The datasheet is explicit: *"SD immediately tri-states after the LSB is
output so that another microphone can drive the common data line"* and
*"The SD trace should have a pull-down resistor to discharge the line
during the time that all microphones on the bus have tri-stated their
outputs. A 100 kΩ resistor is sufficient."*

Without it the 8 undriven bit-times at the end of each slot read back as
junk. On this board they read `0x06/0x07/0x0e/0x0f` instead of zero —
the documented symptom of the missing resistor. `microphone_start()`
enables the ESP32's internal pull-down on DIN as a software stand-in.

This is hygiene only: `raw >> 8` discards those bits regardless. Do
**not** substitute a pull-*up* here — an earlier debugging session added
one, which is backwards, fights the mics, and makes raw dumps harder to
read.

Note that the data LSB (bit 8 of the 32-bit word) also reads a constant
zero, most likely because the P4 latches it just after the mic releases
the line — the datasheet allows no margin there. It costs one bit of a
24-bit word (-144 dB) and is not worth chasing.

### `i2s_channel_read` returns ESP_ERR_TIMEOUT with valid partial bytes

`i2s_channel_read` doesn't atomically grab N descriptors. Internally it
loops, calling `xQueueReceive` once per descriptor, copying data out,
until the requested size is reached or a per-descriptor timeout fires.
If the timeout fires partway through a multi-descriptor request, the
function returns `ESP_ERR_TIMEOUT` with `bytes_read = (already-fetched
bytes)` — i.e., **valid data is left in your buffer that you must
process or you lose it**.

The previous version of this driver requested 2 descriptors per call
(4096 bytes) and did `if (err != ESP_OK) continue;`, which silently
discarded ~2048 bytes on every timed-out read. The result looked exactly
like a clock-divider bug: a rate-shaped phantom shortfall (~16 % at
44.1 kHz, ~7 % at 48 kHz) with no other symptoms.

Workaround: read **exactly one descriptor's worth per call**
(`MIC_DMA_FRAMES = dma_frame_num × slots_per_frame`). Each call needs
only one `xQueueReceive`, so it either succeeds atomically or returns
0 bytes — no partial state to mishandle.

### IDF RX ISR silently drops descriptors on queue overflow

`components/esp_driver_i2s/i2s_common.c` (the `i2s_dma_rx_callback`) ::
when the message queue is full and **no `on_recv_q_ovf` callback is
registered**, the ISR discards the oldest descriptor without logging,
without setting an error flag, and without notifying the reader. The
loss appears downstream as a slow producer rate.

Workaround: always register `on_recv_q_ovf` and surface the count in
periodic telemetry. We do this via `mic_on_q_ovf` and the `drops=N`
field in the 5-second rate log.

### ESP32-P4 I2S clock source is XTAL by default, not PLL

Unlike older ESP32 family chips, on the P4 `I2S_CLK_SRC_DEFAULT`
resolves to `SOC_MOD_CLK_XTAL` (40 MHz), not a PLL. The 9-bit
fractional divider can synthesize any standard audio rate to within
parts-per-million accuracy from XTAL — clock setup is rarely the
problem when audio rate looks wrong on the P4 (despite that being the
first-instinct hypothesis). See the `i2s_channel_read` partial-read
trap above for what's actually likely going wrong.
