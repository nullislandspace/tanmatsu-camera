# Tanmatsu Camera Port — Hardware and Software Notes

This document captures what is known about the Tanmatsu's camera connector,
the hardware wiring between the connector and the ESP32-P4, and the software
stack required to bring up a Raspberry Pi-compatible camera module (notably
the OV5647).

It is intended as a starting point for anyone adding camera support to an
application on the Tanmatsu — the information here was gathered from the
public Tanmatsu hardware documentation, the KiCAD schematics, the vendored
ESP-IDF 5.5.1 tree, the `badgeteam__badge-bsp` component, and the
`espressif/esp_cam_sensor` driver component.

---

## 1. The camera connector

The Tanmatsu exposes a 22-pin 0.5 mm FPC camera connector that is pinout-
compatible with the camera connector on the Raspberry Pi Zero and the
Raspberry Pi 5. Any camera module built for those boards is mechanically and
electrically compatible, but **software support is limited to a subset of
sensor chips** — currently the OV5647 is the reference target.

The connector carries the following signals:

| Pin                 | Signal                        | Notes                                                                 |
|---------------------|-------------------------------|-----------------------------------------------------------------------|
| CSI data lane 0 +/− | MIPI CSI-2 data pair 0        | Wired directly to the ESP32-P4's dedicated CSI PHY balls.            |
| CSI data lane 1 +/− | MIPI CSI-2 data pair 1        | Wired directly to the ESP32-P4's dedicated CSI PHY balls.            |
| CSI clock lane +/−  | MIPI CSI-2 clock pair         | Wired directly to the ESP32-P4's dedicated CSI PHY balls.            |
| 3V3                 | 3.3 V power for the module    | Always-on when the board is powered.                                  |
| Pin 6 (`CAM_IO0`)   | Camera enable                 | **Shared with the ESP32-C6 radio enable** — see below.               |
| Pin 5 (`E2`)        | LED control / GPIO            | Wired to **ESP32-P4 GPIO6**, shared with internal expansion pin E2.  |
| `SYS_SDA`           | I²C (SCCB) data               | Same bus as the coprocessor and the radio.                            |
| `SYS_SCL`           | I²C (SCCB) clock              | Same bus as the coprocessor and the radio.                            |

There are **six CSI lines** in total between the ESP32-P4 and the connector
— that is, three differential pairs (two data pairs plus one clock pair).

### What is NOT on the connector

The Tanmatsu camera port does **not** provide:

- A dedicated **reset** or **PWDN / XSHUTDN** line from the ESP32-P4. Raspberry
  Pi-style camera modules handle reset and power sequencing internally.
- A host-driven **XCLK / MCLK**. Raspberry Pi camera modules have their own
  25 MHz crystal on-board and do not need an external clock.

This is why the connector only has three MIPI pairs plus power, enable and an
optional LED line — everything else the sensor needs is generated on the
module itself.

---

## 2. MIPI CSI-2 data path

The three CSI differential pairs route directly from the camera connector
into the ESP32-P4's dedicated MIPI CSI-2 receiver PHY. These pins are **not**
on the GPIO matrix — they are analog balls belonging to the CSI block, so
there is no GPIO configuration, pinmux, or lane assignment to do in software.

Practical consequences:

- No `gpio_config()` call is needed for the CSI lines.
- The `esp_driver_cam` CSI controller driver talks to the PHY directly.
- Lane count is fixed by the hardware routing: **two data lanes** plus the
  clock lane.
- No lane swap or polarity inversion needs to be configured in software — the
  schematic routes the pairs straight through.

---

## 3. Power and enable — the shared C6 radio line

This is the single most important hardware quirk to understand about the
Tanmatsu's camera port.

The camera module's 3.3 V rail is always powered from the board's 3V3, but
the module's **enable** input (pin 6, `CAM_IO0`) is **not** driven by the
ESP32-P4 directly. It is driven by the on-board coprocessor, on the **exact
same pin** that the coprocessor uses to enable the ESP32-C6 radio module.

That means:

- There is **no way to power the camera without also powering the ESP32-C6
  radio**.
- Turning the enable line off to save power on the camera **also turns the
  C6 radio off**, which means WiFi, Bluetooth, and 802.15.4 all drop.
- There is no independent refcount in hardware — it is one line with one
  state.

### How to drive it from software

The BSP already exposes this line as part of the radio power API in
`badge_bsp_power.c`. Reuse these entry points instead of trying to speak to
the coprocessor directly:

- `bsp_power_set_radio_state(bsp_radio_state_t state)` — sets the enable
  line via `tanmatsu_coprocessor_set_radio_state()` over the internal I²C
  bus.
- `bsp_power_get_radio_state(bsp_radio_state_t *out_state)` — reads the
  current state back.

The three valid states are:

| State                              | Enable line | C6 behaviour           | Camera powered? |
|------------------------------------|-------------|------------------------|-----------------|
| `BSP_POWER_RADIO_STATE_OFF`        | deasserted  | powered down           | no              |
| `BSP_POWER_RADIO_STATE_BOOTLOADER` | asserted    | held in bootloader     | yes             |
| `BSP_POWER_RADIO_STATE_APPLICATION`| asserted    | running its application| yes             |

For a camera application, either `BOOTLOADER` or `APPLICATION` will power
the sensor — the camera itself only cares whether the enable line is
asserted. `APPLICATION` is the normal choice because it leaves WiFi/BT
usable at the same time.

### Recommended coordination pattern

Because the enable line is shared, a camera application must not blindly
toggle it on close. The clean pattern is:

1. On camera open, call `bsp_power_get_radio_state()`.
2. If it is `OFF`, transition to `APPLICATION` and remember that **we**
   were the one who turned it on.
3. On camera close, only transition back to `OFF` if we turned it on and
   no-one else is relying on it.

For an initial proof-of-concept it is acceptable to simply document
"camera requires radio enable" as a precondition and leave state management
to the caller.

---

## 4. I²C (SCCB) — sensor configuration

The camera module's I²C pins (SCCB SDA/SCL in OmniVision terminology) are
wired to the Tanmatsu's `SYS_SDA` / `SYS_SCL` bus — GPIO 9 (SDA) / GPIO 10
(SCL) on the ESP32-P4. This is the **same I²C bus** that the BSP already
uses to talk to:

- the on-board coprocessor,
- the ESP32-C6 radio module,
- and other system peripherals.

The bus is already brought up by the BSP at boot (see
`badge_bsp_i2c.c` and `bsp_i2c_primary_bus_initialize()`), so a camera
application should **not** create a new I²C master instance for the
sensor. Instead, obtain the existing bus handle from the BSP and add the
sensor as an additional device on it.

### Sensor I²C address

The OV5647 responds at **7-bit address `0x36`** (SCCB write `0x6C`, read
`0x6D`). This does not collide with the Tanmatsu coprocessor or the C6
radio on the shared bus.

### The I²C bus is shared — concurrency matters

Because the coprocessor, the radio, and now the camera sensor all share
one I²C bus, **camera traffic must be serialised against other
system-level I²C traffic**. The BSP already exposes the concurrency
primitive for this — see `badge_bsp_i2c.c`:

- `bsp_i2c_primary_bus_get_handle(&bus)` — obtain the existing master
  bus handle.
- `bsp_i2c_primary_bus_get_semaphore(&sem)` — obtain the shared
  concurrency semaphore.
- `bsp_i2c_primary_bus_claim()` / `bsp_i2c_primary_bus_release()` —
  convenience wrappers around the same semaphore.

The previously-working community camera app confirmed this pattern:
every call into `esp_cam_sensor` (sensor detect, `get_format`,
`set_format`, `S_STREAM` ioctl) was wrapped in a claim/release pair on
this semaphore. Do the same.

### No bus plumbing to write

Because the sensor lives on the primary system I²C bus, the following are
all handled for you by the BSP and `esp_cam_sensor`:

- SDA/SCL GPIO configuration
- Pull-up selection
- Bus speed negotiation
- Bus arbitration between sensor traffic and coprocessor/radio traffic
  (as long as you honour the concurrency semaphore above)

You just pass the existing bus handle down to the sensor init code.

### Sensor configuration in the driver

The `esp_cam_sensor_config_t` used by the old working app looks like
this, and the **reset / PWDN / XCLK fields are all `-1`** — that is the
empirical confirmation that none of those signals are wired on the
Tanmatsu:

```c
esp_cam_sensor_config_t cam_config = {
    .sccb_handle = sccb_io_handle, // created from the BSP bus handle
    .reset_pin   = -1,             // no reset line to the module
    .pwdn_pin    = -1,             // no PWDN line to the module
    .xclk_pin    = -1,             // module has its own crystal
    .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
};
```

Detection is done by iterating over the linker-registered
`__esp_cam_sensor_detect_fn_array_*` section and letting each sensor
driver probe its own I²C address — no need to hard-code `0x36`, the
OV5647 driver does that itself.

---

## 5. LED control (pin 5 / E2)

The camera connector's LED control line is wired to **ESP32-P4 GPIO6**,
which is also exposed on the internal expansion port as pin `E2`. This is
a plain GPIO — not a special camera function.

Two things to keep in mind:

1. **GPIO6 is shared with the internal expansion port pin E2.** Any
   application that uses the LED must be aware that it is driving a pin
   that is electrically connected to an expansion header, and vice versa.
2. **Many camera modules do not have an LED** (for example, plain OV5647
   modules without the indicator LED). On those modules, the pin is simply
   not connected on the camera side and GPIO6 can be left alone entirely.

Do not configure GPIO6 unless your specific camera module actually has an
LED and your application actually wants to control it.

---

## 6. Software stack

All of the software pieces required to drive an OV5647 over MIPI CSI-2 are
already present in the vendored ESP-IDF tree at `esp-idf/` (ESP-IDF 5.5.1).

### Components involved

- `esp_driver_cam` — the CSI controller driver that receives MIPI CSI-2
  frames from the sensor into DMA buffers. Located at
  `esp-idf/components/esp_driver_cam/`.
- `driver/isp` — the ESP32-P4 Image Signal Processor driver, used to
  convert the sensor's native RAW output (RAW8/RAW10) into a more useful
  pixel format such as RGB565 or YUV422.
- `espressif/esp_cam_sensor` — a managed component from the ESP Component
  Registry that provides sensor drivers. OV5647 has been supported since
  version 0.6.0; the current 1.3.0 release supports resolutions up to
  1920x1080 RAW10, with bayer and autofocus fixes.
- The Tanmatsu BSP (`managed_components/badgeteam__badge-bsp/`) for the
  primary I²C bus and the radio/camera enable line.

### Reference example

The ESP-IDF tree ships a directly relevant example at:

```
esp-idf/examples/peripherals/camera/mipi_isp_dsi/main/mipi_isp_dsi_main.c
```

It covers the full CSI → ISP → framebuffer path. Note that its
"CSI → ISP → DSI" zero-copy variant is **not** directly usable on the
Tanmatsu launcher, because the DSI display is already owned by the
launcher's display driver
(`managed_components/badgeteam__badge-bsp/targets/tanmatsu/badge_bsp_display.c`).
On the Tanmatsu the practical pattern is to capture into a framebuffer
and then blit it to the display via the existing PAX-based display
pipeline.

A second helper worth reading is:

```
esp-idf/examples/peripherals/camera/common_components/sensor_init/example_sensor_init.c
```

which shows the sensor-init / I²C bring-up pattern in isolation.

### BSP status

The Tanmatsu BSP currently contains **no camera support**:

- `tanmatsu_hardware.h` has no camera/CSI/CAM defines.
- There is no `bsp_camera_*` API.
- There are no pin macros for camera reset, enable, LED, or SCCB.
- The launcher itself has no camera code.

Adding camera support therefore means adding a new BSP module (or an
application-level module) that:

1. Obtains the primary I²C bus handle from the BSP.
2. Asserts the camera/radio enable line via `bsp_power_set_radio_state()`.
3. Initialises the OV5647 through `esp_cam_sensor` on that I²C bus.
4. Configures `esp_driver_cam`'s CSI controller for the negotiated format
   and resolution.
5. Configures the ISP for the desired output pixel format.
6. Provides a frame-delivery API to the application.

### Prebuilt proof of existence

A third-party community application,
`com.orangemurker.camera` ("Camera test" by OrangeMurker), exists in the
official Tanmatsu app repository as a prebuilt binary with no source:

```
/home/cavac/src/tanmatsu/tanmatsu-app-repository-official/com.orangemurker.camera/camera.bin
```

It demonstrates that a working camera app is achievable on the Tanmatsu
today, but it is not a usable source reference.

---

## 7. Practical resolution and memory notes

The OV5647 can deliver up to 2592x1944 RAW10, but that is well beyond the
ESP32-P4's practical memory budget on the Tanmatsu. Use one of the modest
preview-friendly formats exposed by the `esp_cam_sensor` OV5647 driver.

The previously-working community camera app picked:

```
MIPI_2lane_24Minput_RAW8_800x640_50fps
```

That is **800x640, RAW8 from the sensor, two MIPI data lanes**, at a
configured **lane bit rate of 200 Mbps per lane**. The ISP then converts
the RAW8 stream to RGB565 inline. A full 800x640 RGB565 frame is 1 MB,
which fits comfortably in PSRAM.

Other points worth knowing:

- Use **RAW8** from the sensor, not RAW10 — the old working code did, and
  RAW10 doubles the bus bandwidth for no visible gain at preview sizes.
- **DMA descriptors** for CSI frames must live in internal SRAM; only the
  frame buffer itself should be placed in PSRAM.
- The internal SRAM budget on the ESP32-P4 is 768 KB, and PSRAM on the
  Tanmatsu is 32 MB. Plan your frame-buffer count accordingly.
- Frame buffers must be **cache-line aligned**. Use
  `heap_caps_aligned_calloc(cache_hal_get_cache_line_size(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_DATA), ...)`
  with `MALLOC_CAP_SPIRAM` (and `MALLOC_CAP_DMA` for the output/scaled
  buffer if it will be read by a DMA peripheral such as the PPA or the
  display).

---

## 8. Capture pipeline — what the old working app actually did

A partial source copy of the prior community camera-test app is available
at `/home/cavac/src/tanmatsu/camera_old/` (three files: `camera.c`,
`sensor.c`, `sensor.h`). It gives us a concrete, known-working recipe for
a Tanmatsu-targeted OV5647 capture pipeline. The headline structure:

### Pipeline stages

1. **Sensor over SCCB** (`sensor.c`)
   - Get the BSP's primary I²C bus handle and its concurrency semaphore.
   - For each registered `esp_cam_sensor` detect function, create an
     SCCB I/O handle on that bus at 100 kHz and call the detect
     function. First one that responds wins.
   - Query the sensor's supported format list, look up
     `MIPI_2lane_24Minput_RAW8_800x640_50fps`, and set it.
   - Start the sensor stream via
     `esp_cam_sensor_ioctl(cam, ESP_CAM_SENSOR_IOC_S_STREAM, &enable_flag)`.
   - All SCCB calls are wrapped in
     `bsp_i2c_primary_bus_claim()` / `bsp_i2c_primary_bus_release()`.

2. **CSI controller** (`camera.c`)
   - `esp_cam_new_csi_ctlr()` with:
     - `ctlr_id = 0`
     - `h_res = 800`, `v_res = 640`
     - `data_lane_num = 2` (from the sensor's reported `mipi_info`)
     - `lane_bit_rate_mbps = 200`
     - `input_data_color_type = CAM_CTLR_COLOR_RAW8`
     - `output_data_color_type = CAM_CTLR_COLOR_RGB565`
     - `queue_items = 1`
     - `byte_swap_en = false`, `bk_buffer_dis = false`
   - Register event callbacks for `on_get_new_trans` and
     `on_trans_finished`.
   - `esp_cam_ctlr_enable()` → `esp_cam_ctlr_start()`.

3. **ISP** (`camera.c`)
   - `esp_isp_new_processor()` with:
     - `clk_hz = 80 MHz`
     - `input_data_source = ISP_INPUT_DATA_SOURCE_CSI`
     - `input_data_color_type = ISP_COLOR_RAW8`
     - `output_data_color_type = ISP_COLOR_RGB565`
     - `h_res = 800`, `v_res = 640`
     - No line start / line end packets.
   - `esp_isp_enable()`.
   - The ISP is inline between the CSI front-end and the DMA buffer, so
     the frame that lands in `camera_buffer` is already RGB565.

4. **PPA (Pixel Processing Accelerator)** (`camera.c`)
   - A separate ESP32-P4 peripheral used here to **scale and mirror**
     the raw 800x640 capture into a preview-sized buffer that fits the
     UI.
   - `ppa_register_client()` with `PPA_OPERATION_SRM` (scale / rotate /
     mirror), `data_burst_length = PPA_DATA_BURST_LENGTH_128`.
   - Per frame: `ppa_do_scale_rotate_mirror()` with
     `srm_cm = PPA_SRM_COLOR_MODE_RGB565` on both sides, the computed
     `scale_x` / `scale_y`, and — important — **`mirror_y = true`**.
     The sensor image as delivered is vertically flipped relative to
     the display, so the PPA flips it back in hardware at no CPU cost.
   - The PPA is optional if you can render the full-size frame
     directly, but for fitting 800x640 into an arbitrary preview
     widget it is by far the cheapest option.

5. **Task / semaphore wiring**
   - `receive_task` — a tight task that blocks on
     `esp_cam_ctlr_receive()` and hands frames to the controller.
   - `on_get_new_trans` callback — checks a `render_ready` binary
     semaphore and only accepts a new CSI transaction when the UI has
     finished consuming the previous frame.
   - `on_trans_finished` callback — gives a `transfer_done` binary
     semaphore.
   - `update_task` — waits on `transfer_done`, kicks the PPA SRM to
     scale the new frame into the preview buffer, waits on `srm_done`,
     then invalidates the UI widget so the display driver redraws.
   - `render_ready` is given by the display's "refresh ready" event,
     which back-pressures the whole pipeline to the display's frame
     rate.
   - This back-pressured structure is what prevents the camera from
     overrunning the display driver on a memory-constrained SoC — it
     is worth preserving in any port.

### Buffers in the old app

- `camera_buffer` — full-size CSI output, 800 × 640 × 2 bytes = 1 MB in
  PSRAM, cache-line aligned, `MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM`.
- `output_buffer` — preview-sized RGB565, cache-line aligned in PSRAM,
  `MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA` (because the PPA is a DMA
  peripheral).
- Preview height is rounded down to the next multiple of 64 to align
  with the cache line geometry.

### What the old app did NOT do

- It did **not** touch `bsp_power_set_radio_state()` — the radio/enable
  line was assumed to already be asserted by the rest of the
  application. A clean port should at minimum document this as a
  precondition, or assert the enable line itself (and accept the shared
  C6 radio side-effect).
- It did not configure any reset, PWDN, or XCLK pin — confirmed via the
  `-1` values in `esp_cam_sensor_config_t`.
- It did not drive GPIO6 / the LED line at all — consistent with the
  modules in use having no LED.

### Display-side notes

The old app is built against LVGL (`bsp_lvgl.h`, `lv_image_set_src`,
etc.). The tanmatsu-launcher does **not** use LVGL — it uses PAX — so
the capture / ISP / PPA parts of the pipeline carry over directly, but
the "blit into a UI widget" step has to be rewritten to hand the
`output_buffer` to the launcher's existing PAX display pipeline instead
of an `lv_image_dsc_t`. That is the main porting work.

## 9. Recording pipeline — writing video files

A natural follow-on use case is **recording video from the camera to a
file** in a format the existing Tanmatsu video player
(`/home/cavac/src/tanmatsu/tanmatsu-videoplayer`, the "Zōtorōpu" app)
can play back. That player reads **AVI** files containing either
**H.264 Constrained Baseline** or **MJPEG** video, with **MP3** audio.

### What the ESP32-P4 provides on the encode side

- **Hardware JPEG encoder** — via `esp_driver_jpeg`, already in the
  vendored ESP-IDF tree. Espressif's own IDF CI benchmark acceptance
  threshold is **≥160 fps for RGB888 → JPEG at 480p**, so MJPEG
  encoding is not the bottleneck at any resolution the camera produces.
- **Hardware H.264 encoder** — a dedicated block on the ESP32-P4, exposed
  through the managed component **`espressif/esp_h264`** (version 1.3.0
  at time of writing — **not** bundled with ESP-IDF; must be added to
  `idf_component.yml`). The encoder is rated for **1920×1080 @ 30 fps
  real-time**, so 400×240 @ 15 fps has very large headroom. Input
  formats are **I420** or **YUV422** (there is also an `O_UYY_E_VYY`
  variant).
- **MP3 encoder (software)** — the managed component
  `espressif/esp_audio_codec` used to be listed here as the MP3
  encode source, but **it only ships an MP3 *decoder*** (as of
  v2.4.1, which is what the video player vendors). Its encoder
  set is sbc / opus / aac / adpcm / amr / alac / g711 / pcm / lc3
  — no MP3.
  The camera project instead vendors the
  **[Shine](https://github.com/toots/shine)** fixed-point MP3
  encoder as an ESP-IDF component under `components/shine/`.
  Shine is small (nine `.c` files), LGPL-2.0, and designed for
  real-time embedded encode with no psychoacoustic analysis — at
  16 kHz mono 64 kbps it costs well under 10 % of one P4 core.

### What the video player expects — the decode-side constraints

From the video player's README and `h264_changes.md`:

- **Container:** AVI.
- **Video:** H.264 **Constrained Baseline** (no B-frames, no CABAC,
  single slice, one reference frame), OR MJPEG.
- **Audio:** MP3, time-synced to video.
- **Recommended H.264 resolution:** **up to 400×240 @ 15 fps** — this
  is the empirical sweet spot for the `h264bsd` software decoder on the
  ESP32-P4 frame budget. Larger H.264 streams go over the 66 ms budget
  and drop frames.
- **MJPEG resolution:** up to **800×480 @ 20 fps**, because MJPEG
  decoding runs on the hardware JPEG block.
- **Alignment:** video dimensions must be a multiple of **16** for
  macroblock alignment.

### The SD-card bandwidth problem

MJPEG at full resolution is the obvious "just reuse the hardware JPEG
encoder" answer, but it creates a real SD-card write-throughput
problem that H.264 almost entirely avoids. Rough numbers:

| Format         | Resolution | FPS | Typical bitrate | SD write rate |
|----------------|------------|----:|----------------:|--------------:|
| MJPEG q5       | 800×480    |  20 |   ~8–16 Mbps    | **1–2 MB/s**  |
| MJPEG q5       | 400×240    |  20 |   ~2–4 Mbps     |  250–500 KB/s |
| H.264 CRF 28   | 400×240    |  15 |  ~200–400 kbps  | **25–50 KB/s**|

So full-resolution MJPEG is roughly **40–80× more data per second**
than half-resolution H.264 at equivalent perceived quality.

Why that matters specifically for SD cards (and not just "average"
bandwidth):

- A Class 10 / U1 card promises 10 MB/s *on average*, but internal
  garbage collection can stall writes for **hundreds of milliseconds**
  at a time, with no warning.
- To survive those stalls without dropping frames, the recorder needs
  a **RAM ring buffer** in front of the FatFS write path.
- For MJPEG @ 800×480 @ 20 fps, a safe write buffer is in the order of
  **8–16 MB** of PSRAM (absorbs ~5–10 seconds of stall). Possible with
  32 MB PSRAM, but not free — the camera capture buffers, encoder
  working memory and display pipeline also want PSRAM.
- For H.264 @ 400×240 @ 15 fps, **256 KB** of write buffer is already
  comfortable — you could almost get away without one.
- The video player is known to sustain **read** at ~1–2 MB/s for
  800×480 MJPEG playback, so the SD driver and FatFS can in principle
  hit those rates. But SD **writes** are typically 2–3× slower than
  reads, which is exactly where "borderline" lives for full-res MJPEG.

### Recommendation

1. **Primary path: H.264 half-resolution recording.** Pay the one-time
   cost of verifying that the `esp_h264` bitstream is `h264bsd`-
   compatible (single slice, one reference frame, Constrained Baseline
   flag). Once that is confirmed, it is by far the more robust option:
   tiny files, trivial SD write load, long recording times, and no
   burst-stall concerns.
2. **Fallback / "simple mode": MJPEG at reduced settings.** For
   example, **640×480 @ 15 fps** at a modest quality factor. This
   reuses the hardware JPEG encoder with zero bitstream-validation
   work, and drops the sustained write rate to a safe ~300–500 KB/s.
3. **Full-resolution MJPEG** only if someone explicitly asks for it
   and is willing to carry a multi-megabyte PSRAM write buffer and
   accept occasional stall-induced frame drops.

### H.264 validation checklist

Before committing to the H.264 path, a short empirical check:

1. Encode a test clip on the device with `esp_h264` at the target
   resolution and framerate.
2. Inspect the resulting SPS with a tool such as `h264bitstream`:
   - `profile_idc == 66` (Baseline).
   - `constraint_set1_flag == 1` (Constrained Baseline).
   - `num_ref_frames == 1`.
3. Confirm that each encoded picture is one slice.
4. Feed the clip through the video player's `h264bsd` pipeline as the
   final gate.

If any of those fail, options in order of preference: (a) see if
`esp_h264` has a config knob we missed, (b) write a small post-encode
SPS rewriter, (c) fall back to reduced-resolution MJPEG.

### Extra pieces we would need either way

Both recording paths — H.264 and MJPEG — need the same non-codec
plumbing:

- **Pixel-format bridge.** The camera capture pipeline currently ends
  in RGB565 (produced inline by the ISP). The H.264 encoder wants
  **I420** or **YUV422**; the JPEG encoder can take RGB888 or YUV.
  Either reconfigure the ISP to output a YUV format directly during
  recording, or use the PPA (which also supports colour-space
  conversion) to produce the encoder's expected format.
- **AVI muxer.** A small amount of new code: an AVI RIFF header, a
  `hdrl` chunk (stream headers for video + audio), a `movi` chunk
  with interleaved video/audio chunks (`00dc` for video, `01wb` for
  audio), and an `idx1` index written at end-of-file. Straightforward,
  well-documented container.
- **MP3 audio encoder.** `espressif/esp_audio_codec` does **not** ship
  an MP3 encoder (decoder only). The camera project instead vendors
  the Shine fixed-point MP3 encoder as `components/shine/` (see
  above). A full silent-track encode + AVI-mux path is validated in
  `main/video.c`.
- **Memory budget sanity check.** Rough shopping list:
  - ~140 KB for the H.264 encoder state (if H.264 path)
  - ~1 MB for the full-resolution camera frame
  - a few hundred KB for the MP3 encoder state and audio buffers
  - the chosen SD write ring buffer (256 KB for H.264, 8–16 MB for
    full-res MJPEG)
  - plus whatever the launcher's display pipeline still holds
  All of this fits comfortably in the 32 MB PSRAM on the H.264 path;
  full-res MJPEG is the one that makes the PSRAM budget tight.

### Audio recording

Adding audio to the H.264 path is essentially free in both bandwidth
and CPU terms.

**Bandwidth impact:**

| Stream                          | Bitrate       | SD write     |
|---------------------------------|---------------|--------------|
| H.264 400×240 @ 15 fps (CRF 28) | ~200–400 kbps | 25–50 KB/s   |
| MP3 mono 64 kbps @ 16 kHz       |   64 kbps     | ~8 KB/s      |
| AVI muxing overhead             | negligible    | ~1–2 KB/s    |
| **Total**                       |               | **35–60 KB/s** |

That is still an order of magnitude below what any reasonable SD card
can sustain, and well below what even full-res MJPEG would cost.

**CPU impact:**

- Software MP3 encode via the vendored **Shine** encoder
  (`components/shine/`) at **16 kHz mono** is the easy case —
  expect well under 10 % of one P4 core. 22.05 kHz or 44.1 kHz
  mono is still cheap; stereo CD-quality MP3 starts to cost more
  but is still well within budget. Shine runs on a dedicated
  audio task pinned to **core 1** in `main/video.c`, which will
  give the future I²S microphone capture a deterministic CPU
  budget that is not contended by the main UI / camera tasks.
- The H.264 encode runs on the dedicated hardware block and takes
  effectively no CPU, so there is plenty of headroom for MP3, AVI
  muxing and a live preview.

**Memory impact:** I2S DMA buffers (~8–16 KB) + MP3 encoder state
(~30 KB) + audio ring buffer (~32 KB) ≈ **~80 KB**. Negligible.

### Current audio hardware state

As of today, the Tanmatsu BSP only brings up audio **output**. See
`managed_components/badgeteam__badge-bsp/targets/tanmatsu/badge_bsp_audio.c`:

- A single I2S channel is created on **I2S port 0** in **TX-only mode**
  (the RX handle argument to `i2s_new_channel()` is `NULL`, and the
  standard-mode config sets `.din = I2S_GPIO_UNUSED`).
- That channel drives the on-board **ES8156** DAC over I²C control and
  I²S data for the speaker / headphone output.
- There is **no microphone / I²S input path in the BSP** today.

The ESP32-P4 has **three I²S peripherals**, so I²S0 staying dedicated to
the output path is fine — an I²S microphone will live on I²S1 or I²S2
on whichever expansion pins the mic ends up wired to, with its own
independent clock / word-select / data-in GPIOs. No conflict with the
existing output, including running at a different sample rate.

### Bring-up plan — silent MP3 track first

Rather than waiting for the I²S microphone to arrive, the recording
pipeline should include the **full audio encode-and-mux path from day
one**, fed with a silent source:

1. Every time a video frame is encoded, generate a matching block of
   **PCM silence** (zero samples) covering the same time interval —
   e.g. at 16 kHz mono, one 15-fps video frame corresponds to ~1067
   samples of silence.
2. Push that silence through the `esp_audio_codec` MP3 encoder exactly
   as real microphone audio would later be pushed.
3. Write the resulting MP3 frames into the AVI `01wb` audio stream
   alongside the `00dc` video chunks, with timestamps from a common
   monotonic clock (`esp_timer_get_time()`).
4. The AVI header already declares a valid audio stream; the file on
   disk is a fully-formed video + audio AVI that just happens to be
   silent.

Doing it this way buys us several things up front, before any mic
hardware arrives:

- It **validates the MP3 encoder end-to-end** — we measure its real
  CPU cost, memory footprint, and frame timing, and flush out any
  integration issues with `esp_audio_codec` while they are the only
  moving part on the audio side.
- It **validates the AVI muxer's interleaving and timestamping**
  logic, including the `idx1` index, using a known-constant audio
  stream that is trivial to cross-check.
- It gives the **video player a proper audio stream to work with**.
  Files produced in "silent" mode will play back through the existing
  `Zōtorōpu` pipeline exactly like any other AVI, with MP3 audio
  decoded and synced — we just hear silence. No code paths in the
  player need to treat "no audio stream" as a special case.
- When the I²S mic arrives, **only the audio source changes**. The
  encoder, muxer, timing logic and AVI layout stay identical — we
  replace the "silence generator" call with a read from the I²S RX
  ring buffer.

### What is still needed when the real mic arrives

- A small BSP extension analogous to `initialize_i2s()` that opens a
  **second I²S channel in RX mode** on I²S1 or I²S2, with `.din` set
  to the mic's data GPIO and the chosen sample rate. The existing TX
  channel on I²S0 is untouched.
- Wiring the mic to whichever expansion pins are convenient — the
  specific GPIO numbers are not known until we see where it physically
  lands.
- A read task that pulls samples from the I²S RX channel into the
  audio ring buffer that already feeds the MP3 encoder. At that point
  the silence-generator call site becomes the only line that changes.
- A common monotonic clock stamping each captured audio chunk and
  each encoded video frame so the muxer can interleave them correctly
  — this should already exist from the silent-track bring-up.

## 10. Photo mode

Photo mode is a separate capture path from the live preview / video
recording pipeline: press the shutter, capture one frame at the
highest resolution the sensor driver will give us, save it to the
SD card as a JPEG, and show a scaled-down version on the display.

### Target resolution — 1920×1080

The OV5647 sensor chip is physically a **5 MP (2592×1944)** part,
but that full native resolution is **deliberately not the target
for photo mode**. Full-res 5 MP buffers are too large to handle
comfortably alongside the rest of the launcher's memory footprint,
and 2 MP is ample for a badge-format camera app.

The chosen photo resolution is **1920×1080 (≈2 MP) in RAW10**,
which is conveniently also the maximum format exposed by the
current `esp_cam_sensor` OV5647 driver. No extra work is needed
to unlock a higher resolution.

The driver's known format list includes:

| Format name                                     | Resolution | Pixel fmt | Typical use    |
|-------------------------------------------------|------------|-----------|----------------|
| `MIPI_2lane_24Minput_RAW10_1920x1080_...`       | 1920×1080  | RAW10     | **Photo**      |
| `MIPI_2lane_24Minput_RAW10_1280x960_...`        | 1280×960   | RAW10     | Medium photo   |
| `MIPI_2lane_24Minput_RAW8_800x640_50fps`        | 800×640    | RAW8      | Preview / video|

The exact names depend on the `esp_cam_sensor` version; enumerate
them at runtime with `esp_cam_sensor_query_format()` and pick the
highest-resolution entry whose `port == ESP_CAM_SENSOR_MIPI_CSI`.

Note that the photo formats use **RAW10**, not the RAW8 used for
video preview — the CSI controller and ISP must be told to expect
RAW10 on the input side when switching into photo mode, then
configured to emit RGB565 (or YUV) on the output side as before.

### Capture flow

The recommended shutter sequence:

1. **Pause the preview pipeline.** Stop the CSI controller
   (`esp_cam_ctlr_stop()` / `..._disable()`), stop the sensor stream
   (`esp_cam_sensor_ioctl(cam, ESP_CAM_SENSOR_IOC_S_STREAM, &zero)`),
   and disable / delete the ISP processor. This tears down everything
   that was configured for 800×640.
2. **Switch the sensor format to the photo format** over SCCB via
   `esp_cam_sensor_set_format()`. Remember to wrap the call in
   `bsp_i2c_primary_bus_claim()` / `..._release()` like every other
   SCCB access.
3. **Reallocate the capture buffer** for the new dimensions. A
   1920×1080 RGB565 frame is **~4.15 MB** — allocate it in PSRAM
   with the usual cache-line alignment, and free the previous
   800×640 capture buffer if memory pressure matters.
4. **Reconfigure and restart the CSI controller** with the new
   `h_res` / `v_res`, `input_data_color_type = CAM_CTLR_COLOR_RAW10`,
   `output_data_color_type = CAM_CTLR_COLOR_RGB565`,
   `data_lane_num = 2`, `lane_bit_rate_mbps = 200`.
5. **Reconfigure and restart the ISP** with
   `input_data_color_type = ISP_COLOR_RAW10`,
   `output_data_color_type = ISP_COLOR_RGB565`, and the new
   resolution.
6. **Resume streaming** and **throw away the first 3–5 frames**.
   The sensor's automatic exposure / white balance needs a handful
   of frames to re-converge after a format change, and the ISP
   statistics need time to stabilise. Using the very first frame
   after a mode switch usually produces a washed-out or
   wrong-white-balance photo.
7. **Capture one good frame** into the full-resolution buffer.
8. **Tear down the photo pipeline** and rebuild the preview
   pipeline in 800×640 RAW8 mode again, so the live view on the
   display resumes.

This "stop, reconfigure, capture, stop, reconfigure" dance is the
source of the visible **shutter lag** that every simple embedded
camera has — typically a few hundred milliseconds. There is no way
around it with a single CSI controller on a single sensor.

### Encoding to JPEG

Once a clean full-resolution RGB565 frame is in PSRAM, hand it to
the hardware JPEG encoder (`esp_driver_jpeg`) to produce a
standard `.jpg` file. The encoder accepts RGB888 / YUV as inputs;
if it does not accept RGB565 directly in the ESP-IDF version we
ship, the cheapest conversion to its preferred input format is
either a PPA colour-space pass or a short SIMD loop. A 1920×1080
RGB888 frame is ~6.2 MB, which is still comfortable in 32 MB PSRAM.

Expected JPEG file size at sensible quality settings: **0.5–2 MB**
per photo. SD write bandwidth is not a concern here — the write
happens once per shutter press, not continuously.

### Displaying the captured photo

After capture, scale the full-resolution RGB565 frame down to the
launcher's display size using the **PPA**, exactly the same way
the preview pipeline scales 800×640 down to the preview widget.
The full-res buffer stays around until the user dismisses the
photo (for retake / save / delete UX), then it can be freed.

### Storage

- Save into `/sd/photos/` (or a similar well-known path), with a
  filename derived from a monotonic counter or an RTC timestamp
  so successive shutter presses do not collide.
- Keep the write synchronous and simple — no ring buffer is
  needed, since photos are one-shot.
- If the SD card is not present, surface a clean error to the UI
  and discard the buffer rather than crashing.

### Memory budget for photo mode

Peak PSRAM usage during a shutter press, worst case:

- Full-res RGB565 capture buffer: **~4.15 MB** (1920×1080)
- JPEG encoder working buffer + output: **~1–2 MB**
- Optional RGB888 intermediate (if needed for the JPEG encoder
  input format): **~6.2 MB**
- Preview 800×640 RGB565 buffer, if kept alive across the switch:
  **~1 MB**

Total peak: roughly **7–12 MB** during the shutter sequence,
dropping back to the normal preview budget afterwards. Fine in
32 MB PSRAM, but if the launcher is already memory-hungry when
the camera app is launched, it is worth freeing the preview
buffer during the photo pipeline and re-allocating it on the way
back.

### Open questions for photo mode

- **Exposure / white-balance control.** The ISP provides AE and
  AWB statistics; whether we want manual controls (EV
  compensation, WB presets) in photo mode or just trust the
  automatic settling is a product decision, not a hardware one.
- **RAW save option.** With 32 MB PSRAM and SD storage, saving
  the RAW10 sensor output alongside the JPEG is technically
  feasible, but requires a container / sidecar format choice.
  Worth considering as a power-user feature, not a v1 target.

## 11. Summary — what we know, what is missing

### Known and verified against the old source

- The camera port is MIPI CSI-2, pin-compatible with the Raspberry Pi
  Zero / Pi 5 22-pin connector.
- The three CSI pairs go to the ESP32-P4's dedicated CSI PHY — no GPIO
  config needed. Sensor reports **2 data lanes**; old code runs them at
  **200 Mbps per lane**.
- The sensor's I²C is the Tanmatsu's primary `SYS_SDA`/`SYS_SCL` bus on
  GPIO 9 / GPIO 10, shared with the coprocessor and the C6 radio. **SCCB
  access must be serialised** through the BSP's I²C concurrency
  semaphore. OV5647 address `0x36` does not collide.
- No reset, PWDN, or XCLK line is wired — confirmed by the old working
  app passing `-1` for all three to `esp_cam_sensor_config_t`.
- The camera enable line is pin 6 `CAM_IO0`, driven by the coprocessor on
  the **same pin as the ESP32-C6 radio enable**. Controlled via
  `bsp_power_set_radio_state()`. Note: the old working app did not
  touch it, suggesting the radio was already on when the camera ran.
- The LED line is pin 5, on ESP32-P4 **GPIO6** (expansion pin `E2`).
  Optional, and not connected on LED-less modules. The old working app
  did not touch GPIO6.
- ESP-IDF 5.5.1 includes all driver components needed: `esp_driver_cam`,
  `driver/isp`, and `espressif/esp_cam_sensor` (with OV5647 support).
- Working sensor format on the Tanmatsu:
  `MIPI_2lane_24Minput_RAW8_800x640_50fps` → ISP → RGB565.
- The **PPA** (Pixel Processing Accelerator) peripheral is the right
  tool for scaling and mirroring the captured frame into a preview
  buffer, including the required `mirror_y = true` to undo the sensor's
  vertical flip.

### Still missing

- No camera support in the Tanmatsu BSP — any application-side port
  still needs to add its own thin camera module on top of the existing
  I²C and power APIs.
- The old working source is LVGL-based. A concrete "scale-and-blit into
  the launcher's PAX display pipeline" bridge has not been written yet.
- The old code was extracted with main-app pieces missing (for example,
  `bsp_lvgl.h` / `get_i2c_bus` / `lvgl_lock()` are referenced but not
  included here). Those references map cleanly to the current BSP API
  (`bsp_i2c_primary_bus_get_handle` / `bsp_i2c_primary_bus_claim`), but
  cross-checking against the full original app could still surface
  small details we have not seen.

---

## 12. File paths referenced

- `managed_components/badgeteam__badge-bsp/targets/tanmatsu/tanmatsu_hardware.h`
  — BSP pin header (no camera defines).
- `managed_components/badgeteam__badge-bsp/targets/tanmatsu/badge_bsp_i2c.c`
  — primary I²C bus initialisation (`SYS_SDA`/`SYS_SCL`).
- `managed_components/badgeteam__badge-bsp/targets/tanmatsu/badge_bsp_power.c`
  — `bsp_power_set_radio_state()` / `bsp_power_get_radio_state()`, the
  entry points for the shared camera/C6 enable line.
- `managed_components/badgeteam__badge-bsp/targets/tanmatsu/badge_bsp_display.c`
  — owns the DSI display; camera cannot use the zero-copy CSI→DSI path.
- `esp-idf/components/esp_driver_cam/` — CSI controller driver.
- `esp-idf/examples/peripherals/camera/mipi_isp_dsi/main/mipi_isp_dsi_main.c`
  — CSI + ISP reference example.
- `esp-idf/examples/peripherals/camera/common_components/sensor_init/example_sensor_init.c`
  — sensor I²C bring-up pattern.
- `/home/cavac/src/tanmatsu/tanmatsu-app-repository-official/com.orangemurker.camera/camera.bin`
  — existing prebuilt community camera-test app (no source).
- `/home/cavac/src/tanmatsu/camera_old/camera.c`
  — prior working CSI + ISP + PPA capture pipeline (LVGL-based UI).
- `/home/cavac/src/tanmatsu/camera_old/sensor.c`
  — prior working OV5647 detect and format setup over SCCB.
- `/home/cavac/src/tanmatsu/camera_old/sensor.h`
  — sensor module header.
- `/home/cavac/src/tanmatsu/tanmatsu-videoplayer/README.md`
  — video player format requirements (AVI + H.264 CBP / MJPEG + MP3)
  and recommended resolutions.
- `/home/cavac/src/tanmatsu/tanmatsu-videoplayer/h264_changes.md`
  — h264bsd decoder constraints, measured ESP32-P4 performance, and
  PPA output format details.
- `esp-idf/components/esp_driver_jpeg/` — hardware JPEG encode/decode
  driver (for MJPEG recording).
- `espressif/esp_h264` — managed component for hardware H.264 encode
  (must be added to `idf_component.yml`, not bundled with ESP-IDF).
- `espressif/esp_audio_codec` — managed component providing MP3
  encode (already used by the video player).
