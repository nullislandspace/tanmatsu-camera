# PR #1 staged merge — TC358743 HDMI input + catprinter

> **This document is the working plan and the status tracker.** It lives in the repo so
> progress is version controlled and any session can resume by reading it. Update the
> status table and the `CURRENT POSITION` line as work proceeds, and commit the change
> together with the code it describes.

## Status

Legend: `[ ]` not started · `[~]` in progress · `[x]` done · `[!]` blocked

```
CURRENT POSITION: Phase 1, step 1.7 (1.1-1.6 done, awaiting hardware confirmation)
```

| Phase | Scope | Testable by cavac? | Status |
|---|---|---|---|
| 0 | Land this plan in the repo | yes | `[x]` |
| 1 | TC358743 HDMI→CSI support | partly (no-regression only) | `[~]` |
| 2 | F5 fullscreen | yes | `[ ]` |
| 3 | Radio cost measurement — **gate for 4–6** | **yes** | `[ ]` |
| 4 | BLE transport scaffold | no | `[ ]` |
| 5 | Catprinter protocol driver | no | `[ ]` |
| 6 | Print UI integration | no | `[ ]` |

Per-step checkboxes live in each phase below. Update both the table and the
`CURRENT POSITION` line as work proceeds, and commit the change with the code.

---

## Context

PR #1 (`nullislandspace/tanmatsu-camera`, author renzenicolai) bundles two unrelated
features: a **TC358743 HDMI-to-MIPI-CSI bridge** and a Bluetooth **catprinter**. It is
`CONFLICTING` against current `main`.

We have neither piece of hardware; the PR author tests. So every phase is ordered so that
the parts *we* can verify come first, and each step up to the point of no return is
provably a no-op for OV5647/OV9281 — flash and confirm no regression at each stage.

### Why we reimplement rather than merge

8 commits, +10089 −5947. Two of them are `Apply formatting` — clang-format churn across 38
files, ~90% of the line count, against a base (`23e382c`) that `main` has moved far past.
The actual work is a **self-contained new component** (9 files, ~1256 lines, zero conflict
risk) plus **~180 lines** for the bridge and **~2100 lines** for the printer. Three hunks
are stale against `main` and must be re-applied by hand, not pasted (`autofocus_init`
predates `autoexposure_init`; the letterbox block was rewritten by `6636fae`; the HUD block
gained a dirty-counter). So: cherry-pick the component verbatim, hand-write the rest.

Commit map:

| Commit | Content | Phase |
|---|---|---|
| `20c071f` | Apply formatting | never |
| `a4ca9b7` | Add TC358743 support | 1 |
| `c536d59` | Fix color conversion | 1 |
| `3d6ecfd` | No signal + stream recovery | 1 |
| `40e9bb3` | Pixel swap **and** fullscreen | 1 (swap) + 2 (fullscreen) |
| `2c1d2dc` | Catprinter support | 4 + 5 + 6 |
| `009a2c7` | Print from photo viewer | 6 |
| `980051f` | Apply formatting | never |

### Decisions taken

| Question | Decision |
|---|---|
| GPIO6 enable line | Two-pass detect + electrical probe + **settings opt-in, default OFF, red warning** |
| Detection order | TC358743 only probed when no other camera answered |
| F5 fullscreen | Its own phase, after phase 1 lands |
| Video colour path | **Option C** (YUV422→YUV420, PPA does the rest), with **Option A** (via RGB565) selectable in the settings menu and persisted in the config file |

---

## Phase 0 — Land this plan

- `[x]` Create `.claudeplans/testhardware.md` in the repo from this document.
- `[x]` Commit it on its own so the plan has a stable history separate from the code.

Thereafter: update the status table in the *repo copy* as part of each phase's commit.

---

## Phase 1 — TC358743 HDMI→CSI support

### Hardware/driver facts established

- **The PPA cannot ingest YUV422.** `PPA_SRM_COLOR_MODE_YUV422` is commented out in
  `hal/include/hal/ppa_types.h:48` (`TODO: P4 ECO2 supports YUV422`), with matching dead
  validation at `ppa_srm.c:187-191`. **The ISP cannot either** —
  `hal/esp32p4/include/hal/isp_ll.h`'s `isp_ll_set_input_data_color_format` accepts only
  `COLOR_SPACE_RAW`. A CPU conversion is genuinely unavoidable; bypass is the only route
  and the PPA is the only consumer.
- **The PPA *can* ingest YUV420** (`ppa_srm.c:181-185`, even w/h/offsets — 800×480
  qualifies) and does **YUV→RGB in hardware** (`ppa_srm.c:144-146` sets `rx_yuv_range` and
  `rx_yuv2rgb_std`). This is what makes Option C possible.
- **`esp_isp_enable()` rejects bypass mode** — `esp_driver_isp/src/isp_core.c:234`.
  `camera_pipeline.c:504` calls it unconditionally while `:496` sets
  `bypass_isp = !bayer_input`, so **ISP bypass has never worked in this repo**. The
  OV5640/OV5645 RGB565 path has been aborting since it was written.
- **Video geometry needs zero changes to `video.c`.** `pick_video_dims`
  (`video.c:637-646`) on 800×480: k=16 exceeds `VIDEO_MAX_PIXELS`; k=14/12/10 fail `%256`;
  **k=8 → 400×240**, both multiples of 16, 96000 ≤ 128000, no encoder padding. Aspect 5:3
  preserved. Bitrate scales to 300000 bps.
- **Hardware encoding is unaffected** — `video.c:714` uses `esp_h264_enc_hw_new`; the PPA
  scale is hardware too. The only new CPU cost is the conversion.
- **The driver forces the source geometry.** Its EDID advertises CVT 800×480@60 as the sole
  mode. It exposes no exposure/gain controls (`get_para_value`/`set_para_value` return
  `ESP_ERR_NOT_SUPPORTED`).
- **GPIO6 is camera-connector pin 5 (LED) and internal expansion pin E2** (`camera.md:33`,
  `:215-231`, `:872-874`), *not* camera enable — that is connector pin 6 `CAM_IO0`, driven
  by the coprocessor via `bsp_power_set_radio_state()`. `camera.md:228` says verbatim not
  to configure GPIO6 speculatively.

### Blocking defects in the PR

1. **500 ms boot delay for every user.** `tc358743_power_on`'s `reset_pin < 0` branch does
   `delay_ms(500)`, and `camera_sensor.c:157-162` always passes `-1`. Detection walks the
   linker array in link order, so an OV board can pay this before the I2C NACK at 0x0F.
2. **`usb_initialize();` is commented out** in `app_main`. `main`'s `c69bf2c` depends on it.
3. **GPIO6 driven high unconditionally**, before detection, on every board.
4. `printf("Option: %s\r\n", ...)` debug loop in `set_format_by_name`.
5. Dead unreachable `return SHARED_FORMAT_TC358743;` in `format_name_for`'s `default:`.
6. Comment says "after 2 s", code tests `>= 4000`.

### Environment gotchas

- **`CONFIG_COMPILER_OPTIMIZATION_DEBUG=y`** (`sdkconfig_tanmatsu:759`) → `-Og` everywhere.
  A pixel loop pays roughly double. Step 5a puts the conversion in its own TU at `-O2`.
- **`CONFIG_PM_ENABLE=y` + `CONFIG_PM_DFS_INIT_AUTO=y`** (`sdkconfigs/tanmatsu:36-37`) —
  DFS adds frequency-ramp jitter to a newly CPU-bound path. Say so, so noisy fps numbers
  aren't misdiagnosed.
- Build with `env -u IDF_PATH make build` — `IDF_PATH` leaks from the session env.

### Steps

- `[x]` **1.1 — Fix the latent `esp_isp_enable` abort.** `main/camera_pipeline.c`: add
  `static bool s_isp_enabled`, guard the enable at `:504` with `if (bayer_input)`, guard
  `esp_isp_disable` at `:632` with the flag, clear on the `fail:` path. Comment citing
  `isp_core.c:234`. *Verify:* `bayer_input` is true for RAW8/RAW10 → OV path identical.
  Flash; check preview, photo, video, F1/F2/F3 (these exercise `camera_preview_stop`).

- `[x]` **1.2 — Capability gate for the OmniVision register bank.**
  `main/camera_sensor.{c,h}`: extend `sensor_ae_caps_t` (`:71-77`) with a leading
  `bool has_ov_regs` rather than adding a parallel `sensor_is_omnivision()` predicate — the
  struct already *is* the per-kind register-bank description (`gain_reg_h/l`,
  `gain_max_q4`, `exp_margin` are all OmniVision-bank facts), so "does the bank exist at
  all" belongs on the same axis. One switch to maintain; a parallel predicate would
  duplicate the kind list and drift. Set `true` for all existing kinds. Guard
  `capture_fps_base` (`:120`), `set_preview_fps` (`:397`), `set_exposure_eg` (`:484`),
  `read_exposure` (`:586`), `set_auto_exposure` (`:633`). Add
  `camera_sensor_has_manual_exposure()`.

  This matters because `camera_sensor_write_reg` goes through `ESP_CAM_SENSOR_IOC_S_REG`,
  which the TC358743 driver implements as a live **16-bit** write — `write_reg(0x3503, x)`
  is a real write to TC358743 address 0x3503. Three of the five sites happen to be saved
  today by `row_time_ns == 0` / `!ae_mode_valid`, but `read_exposure` is genuinely
  unguarded (reachable from `enter_manual_exposure`, `main.c:489`) and the accidental
  safety breaks the next time anyone touches `capture_fps_base`.
  *Verify:* all reachable kinds have `has_ov_regs = true` → every guard is `if (true)`.
  Test the OV9281 specifically; it already exercises the `gain_reg_h == 0` branch.

- `[x]` **1.3 — Introduce the sensor kind (still unreachable).** Enum value appended after
  `CAMERA_SENSOR_OV9281`; `SHARED_FORMAT_TC358743`; `name_to_kind` strcmp; `ae_caps` case
  returning all-zero **before `default:`**; `format_name_for` case. Do not merge the PR's
  `printf` loop or its dead `return`.

- `[x]` **1.4 — HUD / config-menu gating.** `main/main.c`: `cfg_item_enabled` (`:220-228`)
  gains `camera_sensor_has_manual_exposure(...) &&` on the `cam_brightness` row;
  `enter_manual_exposure` (`:476`) and the Q/A handler early-return; HUD exposure rows
  (`~:1589-1607`) skipped. `auto_exposure_possible` needs no change — `autoexposure_init`
  only runs under `if (bayer_input)` (`camera_pipeline.c:519`), so it is already NULL.

- `[x]` **1.5 — YUV422 pipeline support (still unreachable).**
  - **5a** New `main/yuv_convert.{c,h}`, added to `SRCS` with
    `set_source_files_properties(yuv_convert.c PROPERTIES COMPILE_OPTIONS "-O2")`. A
    separate TU is the only way to raise the optimization level for the hot loop without
    changing codegen for the OV pipeline in the same file — it keeps "OV bit-identical"
    literally true. Matches the videoplayer's `main/yuv_convert.c` naming.
    ```c
    typedef struct { uint8_t y0, v, y1, u; } yuv422_lanes_t;
    extern const yuv422_lanes_t YUV422_ORDERS[8];
    extern const char *const     YUV422_ORDER_NAMES[8];
    #define YUV422_ORDER_DEFAULT 4   /* UYVY, 32-bit word reversed */

    void yuv422_to_yuv420(const uint8_t*, uint8_t*, uint32_t w, uint32_t h,
                          yuv422_lanes_t);   // Option C
    void yuv422_to_rgb565(const uint8_t*, uint8_t*, uint32_t w, uint32_t h,
                          yuv422_lanes_t);   // Option A fallback
    ```
    `yuv422_to_yuv420` is the elegant case: YUV422 already carries one U and one V per two
    horizontal pixels per row, and the encoder's `O_UYY_E_VYY` layout wants one U per two
    pixels on odd rows and one V per two on even rows. The conversion is **pure byte
    selection — drop V on odd rows, drop U on even rows.** No multiplies, no clamps, no
    averaging. Keep the PR's BT.601 coefficients verbatim for the RGB565 fallback
    (298/409/516/100/208, `>> 8`); replace the `CLAMP8` macro with a `static inline`.
  - **5b** `camera_pipeline.h`: add `CAMERA_INPUT_YUV422`; add
    `camera_yuv_path_t { CAMERA_YUV_PATH_YUV420 = 0, CAMERA_YUV_PATH_RGB565 }` and a
    `yuv_path` field on `camera_source_t` so the choice arrives through the existing
    descriptor and `switch_pipeline_to_source` rebuilds naturally; add the yuv-order
    getter/setter; fix `camera_preview_get_raw_pixels`'s now-wrong comment (`:140-146`,
    zero callers).
  - **5c** `camera_pipeline.c`: `s_yuv_conv_buf` sized `w*h*3/2` or `w*h*2` by path,
    allocated **right after the `s_cam_buf` loop (`:385`)** so the existing `goto fail;`
    ordering is undisturbed, freed beside `s_preview_buffer` (`:659`). Introduce
    `csi_out_ct` (`:424`, used at `:456`) — numerically a **no-op** for existing cases
    (`esp_cam_ctlr_csi.c:138-152` uses colour types only for bit depths; `color_hal.c`
    returns 16 for both RGB565 and YUV422) but it makes the YUV422 case honest. New
    `case CAMERA_INPUT_YUV422:` with `isp_in_ct == isp_out_ct` (bypass requires it,
    `isp_core.c:82-84`). **Keep `main`'s `if (bayer_input) { autofocus_init();
    autoexposure_init(); }` at `:510-524` exactly as-is** — the PR's version predates
    `autoexposure_init` and would silently delete software AE for the OV5647. Extract a
    `static const uint8_t *maybe_convert(const uint8_t *src)` used at all three sites —
    `render_task` (`:243`), `camera_photo_snapshot` (`:744`), `camera_video_snapshot`
    (`:875`) — so they cannot drift; it must
    `esp_cache_msync(src, s_cam_buf_sz, ESP_CACHE_MSYNC_FLAG_DIR_M2C)` first, because
    `s_cam_buf` is DMA-written and has never been CPU-read before now and nothing
    invalidates it (address and size are already cache-line aligned, `:378-380`). The
    converted buffer needs no manual flush — `ppa_srm.c:245` writes it back. Set
    `.in.srm_cm` per path, and for YUV420 also `.in.yuv_range = PPA_COLOR_RANGE_LIMITED`
    and `.in.yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601` to match the driver's
    `MASK_VOUT_COLOR_601_YCBCR_LIMITED`. Add per-frame conversion timing, reusing the
    pattern at `:293-299`.
  - **5d** `main.c` `pick_source` (`:95`): new case before the OV5640/OV5645/`default`
    group, taking `yuv_path` from `g_cfg`.

- `[x]` **1.6 — Config keys and settings rows.** The config file is plain `key=value` text
  on the SD card (`config.c:127-134` writes, `:168-219` parses). Three new keys, all
  persisted, all defaulted so existing files keep working:

  | Key | Type | Default | Menu row |
  |---|---|---|---|
  | `hdmi_probe` | bool | `0` | "HDMI probe" — **red warning text**; gates step 1.8 entirely |
  | `hdmi_color_path` | int 0/1 | `0` | "HDMI colour": 0 = YUV420 direct, 1 = via RGB565 |
  | `hdmi_yuv_order` | int 0..7 | `4` | "HDMI byte order" |

  The last two are enabled only when the detected sensor is a TC358743. Changing
  `hdmi_color_path` changes both buffer size and PPA input colour mode, so its handler must
  call `switch_pipeline_to_source`. `hdmi_yuv_order` is read per frame; no rebuild.

- `[ ]` **1.7 — Vendor the driver component (first OV-visible step).** Copy
  `components/esp_cam_sensor_tc358743/` from `a4ca9b7`, all 9 files verbatim except the POR
  fix. Add to `main/CMakeLists.txt` `PRIV_REQUIRES`. Add to `sdkconfigs/tanmatsu` after
  line 53:
  ```
  CONFIG_CAMERA_TC358743=y
  CONFIG_CAMERA_TC358743_AUTO_DETECT_MIPI_INTERFACE_SENSOR=y
  ```
  The component's CMakeLists is a structural clone of `esp_cam_sensor_ov9281`'s, including
  the `-u tc358743_detect` link-keep, so integration risk is nil. No I2C collision at 0x0F
  (coprocessor 0x5F, BMI270 0x68, ES8156 0x08).
  **Required fix:** move the `delay_ms(500)` POR wait out of `tc358743_power_on` into
  `tc358743_set_format` — it only matters before configuration, not before an I2C ID read.
  Comment the deviation so the author can upstream it.
  *Verify:* boot an OV5647; measure reset→first-frame against the 1.5 baseline, must be
  within noise. Confirm the log shows the TC358743 probe failing and the OV driver binding
  after it. Repeat on OV9281.

- `[ ]` **1.8 — GPIO6, two-pass detect.** Gated entirely on `g_cfg.hdmi_probe`; with it off
  none of this runs and GPIO6 is never configured.
  ```
  1. camera_sensor_detect()
       OK  -> GPIO6 never touched. If kind == TC358743, raise it anyway
              (a bridge that answers I2C may still need it to stream).
       FAIL and g_cfg.hdmi_probe:
  2.   Electrical probe: input + internal pull-up, read; input + internal
       pull-down, read. The P4's internal pulls are ~45k, sourcing/sinking
       ~70uA, safe even into a dead short. Follows the pull both times ->
       net is floating, nothing else drives it -> safe to go push-pull.
       Stuck either way -> something is driving it: log, abort the probe.
  3.   Drive high, log loudly (cite camera.md section 5), wait ~50 ms.
  4.   camera_sensor_detect() again.
         OK and TC358743 -> keep high.
         otherwise -> restore Hi-Z (GPIO_MODE_DISABLE, both pulls off),
                      fall through to the existing "No sensor" splash.
  ```
  Use `GPIO_MODE_DISABLE`, **not** `gpio_reset_pin()` — the latter enables the internal
  pull-up, which is not "untouched". `camera_sensor_detect` is safely re-callable: it
  `memset`s `*out` at `:150` and deletes each failed probe's SCCB handle. Drop the PR's
  `splash(RED, WHITE, "Camera error", "Cannot power camera")` — `gpio_config` on a valid
  pin cannot fail and the message is factually wrong anyway.
  *Verify:* with `hdmi_probe=0`, the OV boot log must never mention GPIO6. Scope the pin to
  confirm it stays Hi-Z.

- `[ ]` **1.9 — "NO SIGNAL" overlay + stream recovery.** `main.c:1194-1201` currently
  `continue`s when no frame arrives, with an explicit comment about not redrawing an
  identical HUD ten times a second. The PR deletes that for all sensors. Gate it instead on
  `const bool tolerate_no_frame = (sensor.kind == CAMERA_SENSOR_TC358743);` — an HDMI bridge
  legitimately has no signal for seconds (source unplugged, EDID renegotiation); for an
  image sensor a missing frame is a glitch. Gate the recovery block the same way so
  `switch_pipeline_to_source` can never fire on an OV board, and fix the comment to match
  the code (**4000 ms** is right for HDMI renegotiation latency).
  **Re-apply the overlay onto `main`'s current `:1325-1373`, do not paste the PR's hunk** —
  `6636fae` rewrote it and it now handles left/right pillars and `preview_bars_dirty`,
  neither of which exists in the PR's base. The no-signal branch fills the whole preview
  area black, so the margins are already correct and `preview_bars_dirty` needs no poke.
  `fbdraw_hershey_string_width` exists (`fbdraw.h:113`).
  Worth doing beyond cosmetics: `set_format` drops HPD and rewrites the EDID,
  `set_stream(1)` raises HPD after 150 ms, and the source then needs 1–3 s to read EDID and
  start TMDS. Without the overlay the app just sits black at boot. Put that in the commit
  message.

- `[ ]` **1.10 — Byte-order affordance for the author.** **Ship `[Y1, V, Y0, U]` as the
  default with high confidence.** The driver programs `MASK_YCBCRFMT_422_8_BIT`, emitting
  CSI-2 data type 0x1E whose spec wire order is `U, Y0, V, Y1` (UYVY) — and the PR's
  empirical order is the **exact 32-bit byte-reversal** of that. Not one of 24 arbitrary
  permutations: the single permutation a word-endianness flip produces. The mechanism is
  right there — under bypass, `isp_core.c:154` reconfigures the input as *"the number of
  32-bit in one line"*. The author's two corrections converged on what theory predicts.
  Still, ship the 8-entry cycler (four YUV422 permutations, then the same four word-
  reversed) wired to `hdmi_yuv_order`, and document the symptoms:
  - **Wrong luma lane (Y0↔Y1):** 1-pixel comb on vertical edges; isolated bright pixels
    shift one column by column parity.
  - **Wrong chroma lane (U↔V):** skin tones blue/green, sky orange, luma perfect.
  - **Both:** both at once.

  A row-parity error in the YUV420 packer produces a global U/V swap, already covered by
  the same table — no separate knob. Note for the author: if the winner is a word-reversed
  entry, the real fix may be `csi_cfg.byte_swap_en` (`camera_pipeline.c:459`) or
  `isp_cfg.flags.byte_swap_en`, free in hardware — a follow-up once the order is known,
  not something to attempt blind.

### Never merge

`20c071f` / `980051f` (formatting) · `// usb_initialize();` (grep before committing) · the
`printf` loop · the dead `return` in `format_name_for` · the PR's `autofocus_init` hunk
(drops `autoexposure_init`) · the PR's letterbox hunk (drops `6636fae`'s pillar/dirty
logic) · the PR's unguarded GPIO6 block.

### Phase 1 risks

| # | Risk | Sev | Mitigation |
|---|---|---|---|
| 1 | **ISP bypass has never worked in this repo** — the foundation the TC358743 sits on has zero mileage, and we have no bypass-capable hardware | High | Cannot fix blind. Author's *first* check: `s_cnt_get_new_trans` / `s_cnt_trans_finished` / `s_cnt_srm_done` in the warning at `camera_pipeline.c:196-201` distinguish "CSI never started" from "CSI runs, PPA doesn't" |
| 2 | 500 ms boot delay for every OV user | High | Step 1.7. Must not ship without it |
| 3 | GPIO6 on a documented do-not-touch pin | High | Step 1.8 + `hdmi_probe` off by default |
| 4 | `usb_initialize()` left commented out | High | Pre-commit grep |
| 5 | PPA YUV420-**input** never exercised here; YUV420-in + 90° rotation unverified | Med | Option A is one config toggle away — that is what it is for |
| 6 | `s_cam_buf` never cache-invalidated before its first CPU read | Med | Step 1.5c `esp_cache_msync(M2C)`. Currently masked by double-buffer eviction — latent, silicon-dependent |
| 7 | Conversion compiled at `-Og` | Med | Step 1.5a: own TU at `-O2` |
| 8 | PM/DFS jitter on a newly CPU-bound path | Low | Document it |

### Phase 1 verification

Per step, on OV5647 and OV9281: `env -u IDF_PATH make build`, flash, confirm preview, photo
capture, video record + playback, F1/F2/F3, Q/A brightness and the config menu are
unchanged. Steps 1.1–1.6 should be observably identical; 1.7 is the first that can regress
anything (boot time); 1.8–1.10 are inert with `hdmi_probe=0`.

Record on OV5647, copy the AVI off, confirm it still plays in
`~/src/tanmatsu/tanmatsu-videoplayer`. The TC358743 lands at 400×240, *smaller* than the
OV5647's already-working 400×320, and the videoplayer reads dimensions from the AVI header
(`main.c:562`) with no hardcoded ceiling.

**Photo path needs no changes** and is cleaner than the OV5647's: 800/16 = 50 and
480/16 = 30 are both exact, so the JPEG encoder's YUV420 subsampling has no padding.

**Hand to the author** with: the per-frame timing log, the byte-order cycler and symptom
table, the three CSI/PPA counters for risk 1, and the `hdmi_color_path` toggle. Ask for the
timing number and the winning byte-order index.

---

## Phase 2 — F5 fullscreen

Sensor-agnostic, fully testable here, and it must land **before phase 6** because the
printer's HUD hunks sit inside the `if (!fullscreen)` block.

Geometry checks out. `camera_preview_start:335-352` gives:

| Source | 600×480 area | 800×480 fullscreen |
|---|---|---|
| TC358743 800×480 | frag 12 → 600×360, 60 px bars | frag 16 → **exact 1:1, no letterbox** |
| OV5647 1920×1080 | existing | frag 6 → 720×480 |
| OV9281 1280×800 | existing | frag 9 → 720×450 |

The 1:1 fullscreen mode is clearly why the author wanted this.

- `[ ]` **2.1** De-`const` `preview_area_w/h`, `hud_area_x/w` (`main.c:886-889`). Keep the
  derived expressions (`hud_area_x = preview_area_w`) rather than the PR's hardcoded `600`.
- `[ ]` **2.2** `bool fullscreen = false;` near `prev_mode` (`~:738`). F5 handler: no-op
  while recording, no-op in `MODE_VIEW`/`MODE_CONFIG`, else toggle and
  `switch_pipeline_to_source(...)`. `BSP_INPUT_NAVIGATION_KEY_F5` exists
  (`badge-bsp/bsp/input.h:177`) and is currently unbound.
- `[ ]` **2.3** Wrap the HUD block (`:1420-1408`) in `if (!fullscreen)` **and re-indent the
  body.** The PR left it un-indented, which poisons every future diff of that region.
- `[ ]` **2.4** Force `preview_bars_dirty = 2` in the F5 handler — the geometry check at
  `:1332` catches `pw` changes but not `preview_area_w` changes.

*Verify:* F5 in photo and video mode on both sensors; toggle and back; no stale HUD pixels;
F5 ignored while recording; F5→F1→ESC→F5 leaves consistent geometry. Also exercise
`viewer_open(DCIM_PATH, preview_area_w, preview_area_h)` (`:1069`) and the banner
`fill_rect` (`:1714`), which now see 800 in fullscreen — both are generic (`viewer.c:214-225`
scales by n/16 to fit) but confirm.

---

## Phase 3 — Radio cost measurement (gate for phases 4–6)

**This is the phase you flagged, and it is fully testable on hardware you have.** Do not
start phase 4 until this passes. No printer required — the question is whether the radio
stack can coexist with video recording at all.

The PR brings in `nicolaielectronics/tanmatsu-wifi ^1.2.0`, `esp-hosted-tanmatsu` and `bt`
(NimBLE), and calls `wifi_remote_initialize()` + `wifi_connection_init_stack()` +
`esp_hosted_bt_controller_init/enable()` + `nimble_port_init()` **unconditionally in
`app_main`, before the camera**, with a fatal splash on failure.

### The constraint I found

**The app partition is 2048 KB** (`partition_tables/*.csv`, `ota_0`/`ota_1` both 2048K).
WiFi + esp-hosted + BT controller + NimBLE plausibly adds several hundred KB of flash. If
it does not fit, phase 4 is blocked on repartitioning, which touches the AppFS layout.

Second constraint: `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP` is **not set**
(`sdkconfig_tanmatsu:1626`), so WiFi/LWIP buffers land in **internal SRAM** — 768 KB total
on the P4, already shared with the CSI/PPA/H.264 working set.

- `[ ]` **3.1** On a throwaway branch, add only the three dependencies and the bring-up
  calls — no `ble_peer.c`, no `catprinter.c`, no UI. Build.
- `[ ]` **3.2** Record `idf.py size` / `size-components` before and after. **Gate: does the
  binary still fit in 2048 KB, with headroom?**
  **Baseline measured at step 1.1: `application.bin` = 0xccfd0 = 838096 bytes (819 KB),
  leaving 0x133030 = 1257520 bytes (60%) free in the 2048 KB partition.** Re-measure at
  the end of phase 2 before adding the radio, so the delta is attributable.
- `[ ]` **3.3** Add heap logging to `app_main` (there is none today):
  `heap_caps_get_free_size(MALLOC_CAP_INTERNAL)` and `MALLOC_CAP_SPIRAM`, logged before
  radio init, after radio init, and after `camera_preview_start`. **Gate: how much internal
  SRAM does the radio stack cost?**
- `[ ]` **3.4** With the radio up and BLE scanning forever (the PR uses
  `ble_gap_disc(own_addr_type, BLE_HS_FOREVER, ...)`, passive, filter_duplicates), record
  a video. Compare against a phase-2 baseline recording: preview fps, dropped frames, any
  `srm_done timeout` warnings, AVI frame count vs wall clock, and audio continuity.
  **Gate: is recording still clean?**
- `[ ]` **3.5** Repeat 3.4 with scanning stopped after boot, to separate "the stack exists"
  from "the stack is actively scanning". This directly informs step 4.2.

**Outcomes:**
- All gates pass → proceed to phase 4 as designed.
- Recording degrades only while scanning → phase 4 must make scanning on-demand (it should
  anyway; see 4.2).
- Flash or SRAM does not fit → stop. Options are repartitioning, dropping the WiFi half of
  `tanmatsu-wifi` and bringing up only the BT controller, or shipping the printer as a
  separate app. Decide with real numbers, not guesses.

Record the measurements in this file under phase 3 so the decision is auditable later.

---

## Phase 4 — BLE transport scaffold

~1140 lines: `main/ble_peer.{c,h}` (NimBLE's stock `peer` GATT-cache helper, vendored) plus
~207 lines of GAP scaffold in `main.c`.

- `[ ]` **4.1** Vendor `ble_peer.{c,h}` from `2c1d2dc` (take the post-`980051f` formatted
  version to avoid re-formatting later). Add the three dependencies and the `main.c` GAP
  scaffold: `ble_gap_event`, `ble_on_disc_complete`, `ble_should_connect`,
  `ble_connect_to_device`, `ble_scan`, `ble_on_sync`, `ble_host_task`.
- `[ ]` **4.2** **Make radio bring-up lazy and non-fatal.** The PR's version aborts the
  whole app if the radio is unavailable — a camera that refuses to start because a printer
  stack failed. Move it behind a config key (`printer_enabled`, default `0`, same
  `config.c` pattern as phase 1.6) and demote every failure to a logged warning plus a
  disabled printer, never `splash` + `return`.
- `[ ]` **4.3** **Make scanning on-demand, not `BLE_HS_FOREVER`.** Scan when the user opens
  the print UI or explicitly enables the printer; stop on connect and on leaving the UI.
  Continuous passive scanning is permanent SDIO traffic, CPU and power for a feature used
  seconds at a time. Tune with phase 3.5's numbers. **Never scan while recording.**
- `[ ]` **4.4** Fix the misleading comment claiming the radio shares the camera enable line
  "above" (i.e. GPIO6). It does not — camera enable is `CAM_IO0` via
  `bsp_power_set_radio_state()`. With phase 1.8's design GPIO6 is normally never touched, so
  the PR's comment would be doubly wrong.
- `[ ]` **4.5** Make `ble_addr_str`'s `static char buf` non-reentrant hazard explicit
  (caller-supplied buffer, or document it as log-only).

*Verify:* with `printer_enabled=0`, boot time, heap and recording are identical to phase 2.
With it on and no printer present, the app still starts and records; scanning starts and
stops where expected.

---

## Phase 5 — Catprinter protocol driver

~525 lines, `main/catprinter.{c,h}`. Self-contained: GATT service 0xae30 (advertised
0xaf30), TX characteristic 0xae01, RX 0xae02 for paper-status notifications.

- `[ ]` **5.1** Vendor `catprinter.{c,h}` from the formatted tip. Route the GAP events from
  phase 4 into `catprinter_on_disc_complete` / `_on_disconnect` / `_on_notify`.
- `[ ]` **5.2** Review the image path. 384-dot fixed width; a 800×480 photo scales to
  384×230, so the 1-bit bitmap is `230 × 48 ≈ 11 KB` — negligible. Floyd–Steinberg uses two
  `float` error rows (`calloc(out_w, sizeof(float))`); fine, though integer would be
  cheaper if it ever shows up in profiling.
- `[ ]` **5.3** Review the transfer path. Chunks at `peer->mtu - 3` with
  `vTaskDelay(20 ms)` per chunk in an 8 KB-stack task at priority 5. A full page is ~23
  chunks ≈ 0.5 s of delays — acceptable, but **confirm priority 5 does not preempt the
  video encoder task**, and forbid starting a print while recording.
- `[ ]` **5.4** `catprinter_print_rgb565` does the dither synchronously on the caller's
  thread before spawning the transfer task. On the main loop that is a visible stall.
  Measure it; move it into the task if it is more than a frame or two.

---

## Phase 6 — Print UI integration

- `[ ]` **6.1** `P` key handler (`main.c:~1011`), enabled in `MODE_PHOTO` and — from
  `009a2c7` — in `MODE_VIEW` when `viewer_has_image()`. All four viewer accessors
  (`viewer_has_image`, `_get_pixels`, `_get_width`, `_get_height`) already exist in current
  `main` (`viewer.h:31-38`), so that hunk applies cleanly.
- `[ ]` **6.2** HUD rows: "P print" plus a `Printer: ready / sending / none` status line,
  inside the `if (!fullscreen)` block from phase 2.
- `[ ]` **6.3** Banners: "Printer not connected" / "Printer busy" / "Printing..." /
  "Print failed (%d)". In `MODE_VIEW` the viewer's already-decoded pre-scaled RGB565 bitmap
  is used directly — no snapshot needed, and the printer downscales to 384 dots anyway.
- `[ ]` **6.4** Hide every printer UI element when `printer_enabled=0`, so the default build
  looks exactly as it does today.

*Verify (author):* pair, print from photo mode, print from the viewer, print with the
printer switched off mid-transfer, print with paper out.

---

## Deferred / out of scope

- **Convert-once dedup.** During recording, `render_task` and `camera_video_snapshot`
  convert the same `s_cam_stable` frame independently — 2× cost for nothing. Fix with an
  `s_cam_stable_seq` counter incremented in `on_get_new_trans`. Deliberately not in phase 1:
  it puts an ISR-shared counter into a path the author is about to debug for the first time.
- **Skip the sensor reconfigure on PHOTO↔VIDEO** when `format_name_for` returns the same
  string — today every F2/F3 press drops HPD and costs 1–3 s of black. Touches OV5640/5645.
- **`DMA2D_CSC_TX_YUV422_TO_RGB888_601`** (`hal/include/hal/dma2d_types.h:227`) — the 2D-DMA
  *can* do this conversion in hardware; the PPA driver just doesn't expose it. Driving
  dma2d directly would eliminate the CPU conversion entirely. The real long-term fix. Do
  not attempt blind.
- **PIE SIMD** for the conversion, if `-O2` still isn't enough once the author reports
  numbers.
