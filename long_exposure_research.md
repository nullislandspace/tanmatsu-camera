# Long-Exposure Capability of OV5647, OV5640, OV5645, OV9281

Research notes for using these Raspberry Pi-compatible CMOS sensors for
multi-second night-sky photography on the Tanmatsu / ESP32-P4 platform,
where we have direct register-level control over the sensor.

## TL;DR

- None of these four sensors are *ideal* for night-sky work — they're
  small, uncooled, commercial CMOS with significant dark current at
  multi-second exposures. They're fine for "experimenting" (Moon, bright
  planets, lightning, light-painting, bright stars).
- The Pi-software 6 s limit on the OV5647 is a *driver* limit, not
  silicon. With direct register access you can push all four sensors
  past it.
- Maximum exposure on all four is gated by the **VTS (frame length
  lines)** register × **row time (T_row)**. VTS is 16-bit. To reach
  5–30 s you almost always have to slow the line clock, not just bump
  VTS.
- The OV9281 has the most flexibility for arbitrary "bulb" exposures via
  its FSIN trigger pin, but **only in pulse-width-controlled trigger
  mode**, not the more commonly documented snapshot mode (which uses the
  exposure register, not the pulse width). On the Innomaker V2 module
  the FSIN pin is exposed via an opto-isolated TRIG+/TRIG− header.

---

## ESP32-P4 / Tanmatsu support

Espressif's `esp_cam_sensor` component (the standard ESP32-P4 MIPI-CSI
sensor framework) ships drivers for:

- OV5647 — official
- OV5645 — official
- OV5640 — official (commonly listed)
- OV9281 — **not** in the official component; you would write the driver
  yourself (well-documented sensor, not difficult)

The Tanmatsu's MIPI-CSI block is the standard ESP32-P4 2-lane DPHY, so
all four sensors are electrically compatible. OV9281 typically runs as
2-lane MIPI at 1 MP, which fits.

For multi-second integrations the cleanest approach is **snapshot mode**
+ waking the CSI receiver only for the readout frame, otherwise the host
blocks waiting for a frame that takes 10 s to arrive. ESP-IDF DMA and
application timeouts must be raised accordingly.

---

## Sensor-by-sensor

### OV5647 (Raspberry Pi Camera Module v1, 5 MP)

- 1/4″ Bayer rolling-shutter CMOS, 2592×1944
- Pi software limit: 6 s (driver-imposed; hardware can do more)
- **No on-chip AEC** to fight — clean manual control via I²C
- Long-exposure path on ESP32-P4: bump VTS, slow line clock, write
  exposure register
- Best "easy first-light" choice on Tanmatsu — already in
  `esp_cam_sensor`

### OV5640 (5 MP autofocus, common Arducam/Waveshare modules)

- 1/4″ Bayer rolling-shutter CMOS with **on-chip ISP and AEC/AGC**
- The on-chip AEC will fight you. Disable it via register `0x3503`
  (set bit 0 = manual AEC) before manual exposure values stick. This is
  the #1 thing people miss.
- Exposure registers `0x3500` / `0x3501` / `0x3502` (value in
  *line × 16* units; low 4 bits of `0x3502` are fractional)
- Frame length / VTS at `0x380E` / `0x380F`; AEC-PK-VTS dummy lines at
  `0x350C` / `0x350D`
- Rule from datasheet: exposure value in `0x3500–0x3502` must be less
  than `{0x380E,0x380F} + {0x350C,0x350D}`. To go beyond the default
  frame period you set VTS first, then exposure.
- Also supports **FREX (frame-exposure / snapshot) mode** with external
  XVS pulse — same idea as OV9281 trigger
- Practical multi-second exposures: doable, but requires AEC override +
  PLL/VTS work

### OV5645 (5 MP, MIPI low-power variant)

- Architecturally a sibling of the OV5640; same on-chip ISP/AEC family,
  same line × 16 exposure register layout, same VTS-bound max
  integration
- Mostly used in mobile/embedded reference designs (NXP i.MX, Qualcomm)
- Officially in `esp_cam_sensor`, but rare in Pi/embedded
  astrophotography write-ups
- No documented "long-exposure mode" specifically intended for
  multi-second astro. Same workarounds as OV5640.
- Skip unless there's a specific reason to prefer it over OV5640

### OV9281 (1 MP monochrome global shutter)

- 1/4″ **monochrome** CMOS with **global shutter** — both are genuine
  pluses for night sky (no Bayer demosaic loss, no rolling-shutter
  star streaks during long readout)
- Designed for high-speed/short exposures (10 µs increments, useful from
  ~30 µs upward)
- Stock streaming-mode max exposure ≈ frame period (sub-second at
  default settings; 16-bit exposure register at 10 µs/step ≈ 0.65 s)
- Has **two distinct external-trigger sub-modes** (see below). One uses
  the FSIN edge to *start* a register-defined exposure (no help for
  long exposures); the other uses the FSIN *pulse width itself* as the
  exposure window. The latter is what enables the 60-minute exposures
  Arducam users have reported.
- Only 1 MP and not in official `esp_cam_sensor` — driver port required

#### Global-shutter caveats specific to OV9281

A global shutter doesn't change the timing register model — exposure
and frame period are still gated by VTS × T_row — but it does add
some considerations:

- **Storage-node leakage during readout.** After integration ends, charge
  sits on a per-pixel storage node until that row is read out. That node
  has its own dark-current contribution. Slowing the pixel clock to
  extend exposure also slows readout, so the storage node holds charge
  longer → image-position-dependent noise gradient (top of frame less
  affected than bottom, since top is read out first).
- **MIPI DPHY minimum data rate.** This is the hard floor on the
  "I²C-only, slow PCLK" approach. MIPI CSI-2 D-PHY has a minimum lane
  bit rate (~80 Mbps in many implementations), and the ESP32-P4 receiver
  has its own floor. Push the PCLK too low and the receiver desyncs.
  Practical I²C-only ceiling on OV9281: ~5–10 s, not the minutes you can
  get with FSIN pulse-width mode.
- **PLL lock range.** Pre-divider/multiplier values outside the
  documented PLL range will silently fail to lock — no clock, no MIPI,
  no frame.
- **Exposure register range.** 16-bit, in T_row units. At default
  ~10 µs T_row → ~0.65 s. Slow T_row to ~100 µs → ~6.5 s. Cannot exceed
  65 535 × T_row in any case; you'd hit the DPHY floor first.

---

## Three ways to trigger the OV9281

### 1. Via the MIPI data bus

**No.** MIPI-CSI is strictly sensor → host. There is no command channel
back over the data lanes.

### 2. Via I²C alone (no FSIN pin)

Yes — but capped at the VTS ceiling, same as the other three.

```
write 0x0100 = 0          # stop streaming
set 0x3500/01/02          # exposure
set 0x380E/0F             # VTS (frame length)
write 0x0100 = 1          # start streaming → one frame at chosen exposure
read frame via MIPI
write 0x0100 = 0          # stop streaming
```

Max integration = VTS_lines × T_row. VTS is 16-bit. At default
~30 µs T_row that caps at ~2 s. To reach 5–30 s, slow the pixel clock
(smaller PLL multiplier or longer HTS) so T_row grows. Doable, just
register work.

### 3. Via the FSIN / XVS trigger pin

There are **two distinct external-trigger sub-modes** on the OV9281,
and they handle exposure timing differently. Picking the wrong one is a
common gotcha.

#### Mode A — External-trigger *snapshot* mode (register-defined exposure)

This is what the OV9281 datasheet primarily documents.

- Enable: `0x3030` bit[2] = 1
- Frames per trigger: `0x303F` (= 1 for single-shot)
- FSIN minimum pulse width: 2 µs (`tFSIN_High ≥ 2 µs`)
- Behaviour: rising edge on FSIN wakes the sensor (~61 396 input clock
  cycles), array reset, integrate for `0x3501/0x3502` register-defined
  time, read out N frames, sleep until next FSIN edge.
- **Exposure time is set by the exposure register, NOT by the pulse
  width.** Holding FSIN high longer does nothing useful.
- Useful for: hardware-deterministic *start timing* (e.g. multi-camera
  sync), but capped at the same VTS × T_row ceiling as I²C-only.

#### Mode B — Pulse-width-controlled exposure ("bulb" mode)

This is the mode that gives unbounded exposure time. Less officially
documented; Arducam's wiki is the most public reference.

- Enable: different register sequence, typically involving
  `0x3823` = 0x30 (`ext_vs_en` + `r_init_man`), plus disabling the
  internal exposure counter / AEC
- Behaviour: **rising edge on FSIN starts integration**; **falling edge
  ends integration**; sensor reads out one frame at the normal pixel
  clock; back to idle.
- Exposure time = FSIN high duration (minus small fixed wake-up / reset
  offsets, ~tens of µs).
- The exact register sequence varies by OV9281 firmware revision;
  bring-up is partly empirical. Reference: Arducam's I²C init script for
  trigger mode, and the Rockchip / Antmicro kernel drivers.

**For night-sky use, Mode B is what we want.** Mode A only buys you
deterministic capture *start*, not a long exposure.

#### Why global shutter matters here

Mode B specifically benefits from the global shutter: all pixels start
and stop integrating at *exactly* the trigger edges, so the exposure
window is well-defined and identical for every pixel. The readout that
follows is rolling (pixel array → MIPI lane), but by then integration
is done and the charge is held on the per-pixel storage node — so the
global-shutter property is preserved through readout.

A rolling-shutter sensor in the same setup would smear: rows would see
different integration windows depending on where the readout pointer
was when the trigger arrived.

#### Driving FSIN from an ESP32-P4 GPIO

The FSIN edge does **not** need to come from external hardware. An
ESP32-P4 GPIO can drive it directly (with level-shifting to the
sensor/module's logic level if required). See the Innomaker V2 section
below for the practical wiring.

---

## Innomaker CAM-MIPIOV9281-V2 module — TRIG+ / TRIG− pads

This module exposes the OV9281 trigger via **two unsoldered pads marked
TRIG+ and TRIG−**, wired through an on-board **TLP281 optocoupler**
(galvanic isolation between the trigger source and the sensor).

- TRIG+ = LED anode, TRIG− = LED cathode (single-ended through the
  opto, despite the +/− naming)
- Drive it like an LED: TLP281 wants ~5–15 mA forward current
- On-board series resistor is typically sized for ~5 V drive ≈ 11 mA. At
  3.3 V from an ESP32-P4 GPIO that drops to ~5–6 mA — still above the
  TLP281's typical trigger threshold, but at the low end. If triggering
  is unreliable: drive from 5 V via an external transistor, or replace
  the on-board resistor.
- Polarity: TRIG+ → GPIO, TRIG− → GND. GPIO high energises the LED and
  asserts the trigger.
- Inversion: depending on how the opto's output side is wired to the
  OV9281 FSIN pin, the *actual* edge seen by the sensor may be inverted
  relative to the GPIO pulse. Verify on a scope or empirically.
- Rise/fall: TLP281 ≈ 3–10 µs at low drive currents. Irrelevant for
  multi-second exposures; would matter for fast sync work.

### Wiring on Tanmatsu

1. Solder a 2-pin header (or wires) into the TRIG+ / TRIG− pads
2. Pick any free ESP32-P4 GPIO that survives boot strapping
3. TRIG+ → GPIO, TRIG− → GND
4. Configure OV9281 over I²C in **Mode B** (pulse-width-controlled
   exposure) — see "Three ways to trigger the OV9281" above. Mode A
   (register-defined snapshot) will *not* give you long exposures.
5. `gpio_set_level(pin, 1)` → trigger asserted (integration starts);
   `gpio_set_level(pin, 0)` → trigger released (integration ends,
   readout begins). Exposure ≈ time GPIO held asserted (modulo
   inversion through the opto).
6. After the falling edge, wait long enough to read out one frame over
   MIPI before the next trigger.

---

## The real limiting factors at multi-second exposures

These will hurt more than the register limits:

- **Dark current / hot pixels.** All four are tiny, uncooled,
  ~1.4 µm-pixel commercial CMOS. At Tanmatsu board temperature the
  ESP32-P4 runs warm; a 10 s exposure looks like salt-and-pepper static.
  **Dark-frame subtraction** (capture identical exposure with cap on,
  subtract) is essential. Easy to do in software on the P4.
- **Read noise / no cooling.** Don't expect to see anything dim. Bright
  stars, Moon, Jupiter, lightning — yes. Milky Way detail — no.
- **Stacking beats single long exposures** on uncooled sensors.
  Median-stacking many short exposures (~1–2 s) typically beats a single
  10 s shot. This is how the Pi all-sky-camera community works around
  the same physics.

---

## Recommendations for "simple experimenting" on Tanmatsu

| Use case                  | Best of the four      | Why                                          |
|---------------------------|-----------------------|----------------------------------------------|
| Easiest first light       | OV5647                | Already in `esp_cam_sensor`, no on-chip AEC  |
| Most flexibility / colour | OV5640                | Official driver, FREX mode, but fight AEC    |
| Best optics for stars     | OV9281 (trigger mode) | Mono + global shutter; needs driver port     |
| Skip                      | OV5645                | OV5640 with fewer references; no benefit     |

For night-sky specifically: the OV9281 + Innomaker V2 module + a
GPIO-driven trigger is the most interesting build. Mono is a real
optical advantage, the trigger path gives unbounded exposure time, and
the hardware (TRIG header) is already exposed — no flex-cable surgery.

---

## Sources

### Sensor datasheets / product briefs

- [OV5647 full datasheet (Sparkfun mirror)](https://cdn.sparkfun.com/datasheets/Dev/RaspberryPi/ov5647_full.pdf)
- [OV5640 datasheet (Sparkfun mirror)](https://cdn.sparkfun.com/datasheets/Sensors/LightImaging/OV5640_datasheet.pdf)
- [OV5640 datasheet (Imaging Source mirror)](https://s1-dl.theimagingsource.com/api/2.5/packages/publication/sensor-omnivision/ov5640/98284436-e865-5de5-a517-7a514df571b6/ov5640_1.0.en_US.pdf)
- [OV9281 datasheet (Sinotimes mirror)](http://www.sinotimes-tech.com/product/20220217221034589.pdf)
- [OmniVision OV9281/OV9282 product brief v1.3](https://www.ovt.com/wp-content/uploads/2022/01/OV9281-OV9282-PB-v1.3-WEB.pdf)
- [OmniVision OV9281 product brief v1.4](https://www.ovt.com/wp-content/uploads/2024/05/OV9281-PB-v1.4-WEB.pdf)
- [5MP OV5647 datasheet (UCTronics mirror)](https://www.uctronics.com/5mp-ov5647-download-full-datasheet-pdf)

### Raspberry Pi forum / community references

- [OV5647 and 6 s exposure — RPi Forums](https://forums.raspberrypi.com/viewtopic.php?t=355760)
- [Long exposure with old camera module — RPi Forums](https://forums.raspberrypi.com/viewtopic.php?t=273871)
- [Need help with OV5640 exposure timing — RPi Forums](https://forums.raspberrypi.com/viewtopic.php?t=350473)
- [Raspberry Pi HQ Camera long exposures — RPi Forums](https://forums.raspberrypi.com/viewtopic.php?t=273358&start=50)
- [OV9281 with external trigger — RPi Forums](https://forums.raspberrypi.com/viewtopic.php?t=293959)
- [OV9281 — RPi Forums](https://forums.raspberrypi.com/viewtopic.php?t=291818)
- [Trigger mode OV9281 on Pi 5 — RPi Forums](https://forums.raspberrypi.com/viewtopic.php?t=396655)
- [InnoMaker OV7251/OV9281 driver — RPi Forums](https://forums.raspberrypi.com/viewtopic.php?t=339382)
- [Pi 5 and OV9281-v2 from Innomaker — RPi Forums](https://forums.raspberrypi.com/viewtopic.php?t=371860)
- [New Raspberry Pi Global Shutter camera (IMX296, for context) — RPi Forums](https://forums.raspberrypi.com/viewtopic.php?t=348642)

### Arducam documentation / community

- [Arducam — Set Long Exposure (wiki)](https://docs.arducam.com/Raspberry-Pi-Camera/Low-Light/set-long-exposure/)
- [Arducam OV9281 external-trigger application note](https://docs.arducam.com/UVC-Camera/Appilcation-Note/External-Trigger-Mode/OV9281-Global-Shutter/)
- [Arducam — How to access OV9281 in external-trigger snapshot mode](https://docs.arducam.com/uvc-camera-module/access-camera-demo/how-to-access-arducam-ov9281-global-shutter-uvc-camera-using-external-trigger-snapshot-mode/)
- [Arducam — Hardware Trigger with External Signal (multi-camera sync, Mode A)](https://docs.arducam.com/USB-Industrial-Camera/Connect-and-Sync-Multi-Stereo-Cameras/Hardware-Trigger-with-External-Signal/)
- [Arducam — External Trigger Mode (RPi Global Shutter Camera)](https://docs.arducam.com/Raspberry-Pi-Camera/Global-Shutter-Camera/external-trigger/)
- [Arducam OV9281 Mini mono GS module](https://www.arducam.com/product/mini-ov9281-mono-global-shutter-for-pi/)
- [Set exposure of OV9281 camera — Arducam Forum](https://forum.arducam.com/t/set-exposure-of-ov9281-camera/2889)
- [Use external trigger on OV9281 — Arducam Forum](https://forum.arducam.com/t/use-external-trigger-on-ov9281/5403)
- [OV9281 external trigger — Arducam Forum (MIPI module)](https://forum.arducam.com/t/ov9281-external-trigger/1426)
- [Exposure time vs FPS in external trigger mode — Arducam Forum](https://forum.arducam.com/t/when-using-external-trigger-the-correlation-between-exposure-time-and-capture-fps-and-the-capture-time-with-the-frame/3150)
- [Change exposure time and frame rate OV9281 — Arducam Forum](https://forum.arducam.com/t/change-exposure-time-and-frame-rate-ov9281/6451)
- [Using FSIN on OV9281 from Arduino — Arduino Forum](https://forum.arduino.cc/t/using-fsin-pin-on-camera-sensor-ov9281-with-input-signal-from-arduino-mega-2560/680338)
- [OV9281 datasheet (PDFCoffee mirror — register details)](https://pdfcoffee.com/ov9281-datasheet-pdf-free.html)
- [Innomaker U20CAM-9281M hardware doc (FSIN tFSIN_High ≥ 2 µs)](https://www.inno-maker.com/wp-content/uploads/2023/11/U20CAM-9281M-HW.pdf)

### OpenMV / general references

- [Need help with OV5640 exposure timing — OpenMV Forum](https://forums.openmv.io/t/need-help-with-ov5640-exposure-timing/8238)
- [OV5640 FREX mode — OpenMV Forum](https://forums.openmv.io/t/ov5640-frex-mode/1713)

### Driver source references (register sequences)

- [Linux mainline ov5640.c driver](https://github.com/torvalds/linux/blob/master/drivers/media/i2c/ov5640.c)
- [Rockchip ov9281.c kernel driver](https://github.com/rockchip-linux/kernel/blob/develop-4.4/drivers/media/i2c/ov9281.c)
- [Antmicro tx2-deep-learning-kit ov9281.c](https://github.com/antmicro/tx2-deep-learning-kit-bsp/blob/master/kernel/kernel-4.4/drivers/media/i2c/ov9281.c)

### Astrophotography / evaluation write-ups

- [Long Exposure and Astro-Photography Using Raspberry Pi — Instructables](https://www.instructables.com/Long-Exposure-and-Astro-Photography-Using-Raspberr/)
- [Raspberry Pi Camera evaluation for Astronomy — openastronomy.ca](https://www.gordtulloch.com/2020/07/03/raspberry-pi-camera-evaluation-for-astronomy/)
- [ZivaVatra raspberrypi-astro-capture (GitHub)](https://github.com/ZivaVatra/raspberrypi-astro-capture)

### ESP-IDF / ESP32-P4 references

- [ESP-IDF esp_cam_sensor component (sensor list)](https://components.espressif.com/components/espressif/esp_cam_sensor)
- [ESP-Techpedia — DVP & MIPI-CSI Camera Solution](https://docs.espressif.com/projects/esp-techpedia/en/latest/esp-friends/solution-introduction/camera/dvp-mipi-csi-camera-solution.html)
- [ESP-FAQ — Camera Application](https://docs.espressif.com/projects/esp-faq/en/latest/application-solution/camera-application.html)

### Innomaker module references

- [Innomaker CAM-MIPIOV9281 V2 user manual (PDF)](https://docs.rs-online.com/c69c/A700000009169905.pdf)
- [Innomaker CAM-MIPIOV9281-V1 product page](https://www.inno-maker.com/product/cam-mipiov9281/)
- [Innomaker CAM-MIPI9281RAW-V2 product page](https://www.inno-maker.com/product/cam-mipi9281raw-v2/)
- [Innomaker GS Trigger camera category (TLP281 opto-isolation note)](https://www.inno-maker.com/gs-camera/)
- [Innomaker U20CAM-9281M datasheet](https://manuals.plus/m/9fa0ce4cc55b23b7993e5af28a05031665460942b2b5fbf7c14762d57a093f4a)
- [Innomaker IMX296 trigger camera — GitHub (similar trigger circuit reference)](https://github.com/INNO-MAKER/cam-imx296raw-trigger)
- [TLP281 optocoupler reference](https://docs.cirkitdesigner.com/component/1c3f1d95-b7b0-4b0a-9b11-407881ce58b5/tlp281-4-channel-opto-isolator-ic-module)
