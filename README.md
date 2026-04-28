# Tanmatsu Camera

A still and video camera application for the Tanmatsu (ESP32-P4) running on
top of the ESP-IDF and the badgeteam BSP. Streams a live preview to the 800×480
display, captures JPEG stills and H.264 video with synchronised audio to the
SD card, and provides a built-in viewer for browsing saved photos.

## Features

- Live preview at 15 fps with hardware-accelerated scale/rotate via the
  ESP32-P4 PPA (Pixel Processing Accelerator).
- Still capture: JPEG-encoded via the P4 hardware JPEG encoder, written to
  `/sd/DCIM/IMG_YYYYMMDD_HHMMSS.jpg`.
- Video capture: H.264 in an AVI container with synchronised mono audio
  from an INMP441 microphone, written to `/sd/DCIM/VID_YYYYMMDD_HHMMSS.avi`.
- Built-in photo viewer for reviewing JPEGs already on the SD card.
- Manual focus and ISP-statistics-based autofocus on supported camera
  modules (DW9714P VCM driver included).
- Configurable mic gain, focus driver, autofocus enable, and 180° rotation
  via an on-device settings menu, persisted to `/sd/camera.cfg`.

## Supported camera sensors

The app uses the ESP-IDF `esp_cam_sensor` auto-detect mechanism, so any
sensor whose driver is compiled in and Kconfig-enabled is probed at boot
over the primary I²C bus. The first sensor that responds to its product-ID
register wins.

| Sensor | Status            | Preset                              | Notes                                                          |
|--------|-------------------|-------------------------------------|----------------------------------------------------------------|
| OV5647 | **Tested**        | RAW10 1920×1080 @30fps (capped 15)  | Reference target. Full pipeline: ISP demosaic + autofocus.     |
| OV5640 | Implemented, untested | RGB565 1280×720 @14fps          | Sensor on-chip ISP delivers RGB565; P4 ISP runs in bypass.     |
| OV5645 | Implemented, untested | RGB565 1280×960 @30fps (capped 15)  | Same path as OV5640. Six MIPI presets exist; we pick RGB565.   |
| OV9281 | Implemented, untested | RAW10 1280×800 @30fps (capped 15) | Monochrome global-shutter. RAW10 routed through the demosaicer; output is approximately grayscale. |

The OV5640/OV5645/OV9281 paths were written without access to physical
hardware and have only been verified to compile. They are designed
conservatively to stay structurally close to the OV5647 path:

- One shared sensor format for preview, photo, and video — no shutter-time
  format switch and no PHOTO↔VIDEO sensor reconfiguration.
- For OV5640/OV5645 (RGB565 sensors): the P4 ISP runs in `bypass_isp`
  mode (input==output==RGB565). The CSI bridge still routes data through
  it because the P4 hardware shares the bridge between CSI and ISP.
- For OV9281 (monochrome RAW10): the existing OV5647-style RAW10 →
  demosaic → RGB565 path is reused. Because every input pixel carries
  the same luminance signal regardless of which Bayer position the
  demosaicer assumes, the output is approximately grayscale (R≈G≈B),
  with mild interpolation artefacts at edges.
- Autofocus is disabled on every non-OV5647 sensor. The ISP statistics
  block needed for AF is meaningful only on a true Bayer pipeline, and
  the OV9281 modules are mechanically fixed-focus anyway.

OV9281 support ships as a separate component under `components/` rather
than baked into the host code, so it can be lifted out and published as
a standalone esp_cam_sensor-compatible driver. The init register sequence
is ported from the Linux mainline `ov9282.c` driver (OV9281 and OV9282
are register-compatible and report the same chip ID).

The detected sensor model is shown at the bottom of the right-hand HUD
strip so you can verify at a glance which path the firmware bound to.

### Adding a new sensor

Detection is generic — the app side requires no changes. To bring up an
additional sensor:

1. Ensure the driver lives under
   `managed_components/espressif__esp_cam_sensor/sensors/<name>/` and
   exports a detect function via the `ESP_CAM_SENSOR_DETECT_FN()` macro.
2. Enable `CONFIG_CAMERA_<NAME>=y` and the matching
   `CONFIG_CAMERA_<NAME>_AUTO_DETECT_MIPI_INTERFACE_SENSOR=y` in
   `sdkconfigs/tanmatsu`.
3. If the sensor delivers a pixel format the pipeline doesn't yet handle
   (e.g. mono RAW for OV9281, or YUV422 with no RGB565 alternative), add
   a branch to `camera_sensor_kind_t` and `pick_source()` in `main/main.c`
   plus the corresponding CSI/ISP color-type wiring in
   `main/camera_pipeline.c`.

The `esp_cam_sensor` 5.5.x bundle ships drivers for OV2640, OV2710, OV3660,
OV5640, OV5645, OV5647, GC0308, GC2145, MT9D111, BF20A6, BF3901, BF3925,
BF3A03, OS02N10, SC030IOT, SC035HGS, SC101IOT, SC202CS and SC2336. Most of
those would require pipeline changes to actually stream.

## Hardware

- **Tanmatsu** (ESP32-P4 main MCU). The 22-pin 0.5 mm FPC camera connector
  is pinout-compatible with the Raspberry Pi Zero / Pi 5 camera connectors.
  See `camera.md` for the full hardware breakdown — connector pinout, MIPI
  CSI-2 lane wiring, the shared C6-radio enable line, and the SCCB I²C
  bus topology.
- **Camera module**: any RPi-compatible CSI module with one of the
  supported sensors. The OV5647 modules sold as "Raspberry Pi Camera v1.3"
  are the reference.
- **Microphone (optional)**: INMP441 I²S MEMS mic. See
  `microphone_inmp441.md` for the wiring.
- **Focus motor (optional)**: DW9714P VCM as found on autofocus camera
  modules. Selectable in the on-device config menu.
- **SD card**: required. Photos and videos are saved to `/sd/DCIM/`.

## Building and flashing

The build uses ESP-IDF v5.5.1 (vendored under `esp-idf/`).

```sh
make build              # configure + compile
make flash              # flash to /dev/ttyACM0 (override with PORT=...)
make flashmonitor       # flash and open serial monitor
```

`make build` is the one-stop target: it pulls submodules, sets up the
ESP-IDF environment, and runs `idf.py build` for the `tanmatsu` device
target.

## Controls

The Tanmatsu's function keys map to camera modes; the rest of the
keyboard provides shutter, focus, and gain controls.

| Key            | Action                                                           |
|----------------|------------------------------------------------------------------|
| **F1**         | View saved photos (browse `/sd/DCIM/`)                           |
| **F2**         | Photo mode (live preview, SPACE to capture JPEG)                 |
| **F3**         | Video mode (live preview, SPACE to start/stop H.264 recording)   |
| **F4**         | Open / close the settings menu                                   |
| **ESC**        | Exit to the launcher (also: close menu, stop recording first)    |
| **SPACE**      | Shutter (photo) / record start-stop (video)                      |
| **UP / DOWN**  | Manual focus near/far (hold to scan); cycle settings in menu     |
| **LEFT / RIGHT** | Browse newer/older photos in viewer; adjust setting in menu    |
| **VOL+ / VOL−**| Mic gain trim in video mode                                      |
| **RETURN**     | Toggle / select setting in menu                                  |

## Configuration

`/sd/camera.cfg` is a plain-text key=value file (created with defaults on
first boot). Editable through the on-device F4 menu; manual edits persist
across reboots.

| Key             | Type    | Notes                                                  |
|-----------------|---------|--------------------------------------------------------|
| `focus_driver`  | string  | Active VCM driver name (`dw9714p`, `none`, …)          |
| `focus_enabled` | bool    | Initialise the focus subsystem at boot                 |
| `autofocus_enabled` | bool | Run continuous AF (only meaningful with focus_driver and OV5647) |
| `rotate_180`    | bool    | Flip the live preview 180° (camera mounted upside-down) |
| `mic_enabled`   | bool    | Run the I²S mic in video mode                          |
| `mic_gain`      | int     | Digital gain multiplier for mic samples                |

## File layout

| Path                       | Purpose                                          |
|----------------------------|--------------------------------------------------|
| `main/main.c`              | App entry point, mode loop, HUD render           |
| `main/camera_sensor.c`     | Sensor detect, format selection, VTS fps cap     |
| `main/camera_pipeline.c`   | CSI / ISP / PPA preview pipeline + snapshot path |
| `main/photo.c`             | JPEG encode + save                               |
| `main/video.c`             | H.264 encode + AVI muxer                         |
| `main/microphone.c`        | INMP441 I²S capture                              |
| `main/focus/`              | Focus driver framework + autofocus state machine |
| `main/viewer.c`            | JPEG decoder + browse-on-SD UI                   |
| `main/avi_mux.c`           | Minimal AVI v1 muxer for the recording path      |
| `camera.md`                | Hardware notes (connector, CSI, power, I²C)      |
| `microphone_inmp441.md`    | Microphone wiring + bring-up notes               |
| `FOCUS_TRACKER.md`         | Focus / autofocus implementation tracker         |

## License

The contents of this repository may be considered in the public domain or
[CC0-1.0](https://creativecommons.org/publicdomain/zero/1.0) licensed at
your disposal.
