# Shine MP3 encoder — vendored copy

Fixed-point MPEG Layer 3 encoder imported verbatim from
<https://github.com/toots/shine> (commit `ab5e352`, main branch, April 2026).

Only the encoder library files from `src/lib/` are copied here — the
autotools / JS / Android / frontend pieces are not needed on ESP-IDF and
were left behind upstream.

## Why Shine

- **Fixed-point** — no FPU required, fast on the ESP32-P4's RISC-V cores.
- **Tiny** — nine `.c` files, no psychoacoustic analysis, no look-ahead.
- **Real-time** — designed for streaming encode, not archival quality.
- **Self-contained** — no external dependencies beyond `libc` and `libm`.

At 16 kHz mono 64 kbps (our default for the silent-track bring-up) the
encoder costs well under 10 % of one P4 core, which leaves the other
core free for the H.264 hardware block feed and the AVI muxer.

## Compile-time target selection

`src/types.h` picks a fixed-point multiply backend based on
`__mips__` / `__arm__` preprocessor macros, and falls through to
`mult_noarch_gcc.h` on anything else. The ESP32-P4 is RISC-V so neither
MIPS nor ARM macros are defined and the portable path is used
unmodified. Nothing to patch.

## Memory layout

`shine_initialise()` does exactly two heap allocations:

1. `calloc(1, sizeof(shine_global_config))` — roughly 100 KB of encoder
   state (IMDCT tables, granule buffers, the `int2idx` power-of-3/4 LUT).
2. A small bitstream buffer (a few KB).

The ESP-IDF default sdkconfig has `CONFIG_SPIRAM_USE_MALLOC=1` and
`CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384`, so the ~100 KB struct lands
in PSRAM automatically and the small bitstream buffer stays in internal
SRAM. No source patching required to get sensible placement.

## License

Shine is **LGPL-2.0-or-later**. The full licence text is in `COPYING`,
preserved verbatim from upstream. We link the encoder statically, so to
comply with section 6 of LGPL v2 we must make the unmodified Shine
sources (this directory) available alongside any binary distribution of
the camera firmware. Since this repository is public on GitHub, that
obligation is already met by the presence of the source in this tree.
