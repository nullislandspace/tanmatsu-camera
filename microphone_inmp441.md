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
- `I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO)`.
  STEREO is **deliberate** even though only one mic is wired — see the
  "Pitfalls" section below for why MONO mode doesn't work as expected on
  the P4 RX path.
- `gpio_cfg`: `.bclk = GPIO_NUM_54`, `.ws = GPIO_NUM_49`,
  `.din = GPIO_NUM_53`, `.mclk = I2S_GPIO_UNUSED`,
  `.dout = I2S_GPIO_UNUSED`.
- **I2S runs at 44.1 kHz** so BCLK = 2.8224 MHz, comfortably inside the
  INMP441's 2.048–4.096 MHz BCLK range. The reader task drains DMA, picks
  the LEFT slot (RIGHT carries the floating DIN pin), gain-scales,
  applies a 2-stage IIR low-pass (~4 kHz cutoff — also serves as the
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

Workaround: configure STEREO explicitly and stride-2 through the
buffer to pick the LEFT samples in software. The downstream cost is
negligible — the IIR + decimator already iterate the buffer.

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
