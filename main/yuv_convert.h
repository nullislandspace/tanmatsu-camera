#pragma once

#include <stddef.h>
#include <stdint.h>

// CPU-side unpacking of the packed YUV422 the TC358743 HDMI bridge
// delivers over MIPI CSI-2.
//
// This exists because neither piece of hardware that would normally do
// it can. The ESP32-P4's ISP only accepts RAW Bayer input --
// isp_ll_set_input_data_color_format() rejects anything whose colour
// space is not COLOR_SPACE_RAW -- so a YUV422 sensor has to run the ISP
// in bypass, which means the bytes reach PSRAM exactly as the sensor
// sent them. And the PPA cannot take YUV422 either: on this silicon
// revision PPA_SRM_COLOR_MODE_YUV422 is commented out of
// ppa_srm_color_mode_t entirely (hal/include/hal/ppa_types.h, "TODO:
// P4 ECO2 supports YUV422"), along with the validation that would have
// used it. So something has to unpack the frame, and the only thing
// left is the CPU.
//
// Two targets are offered because they cost very different amounts:
//
//   yuv422_to_yuv420  is pure byte selection -- no multiplies, no
//                     clamps, no colour maths at all -- and hands the
//                     PPA a colour space it can convert in hardware.
//                     Keeps full chroma resolution.
//
//   yuv422_to_rgb565  does the full BT.601 conversion on the CPU,
//                     roughly an order of magnitude more work per
//                     pixel, and throws away colour precision on the
//                     way. It exists as a fallback: it feeds the PPA
//                     the same RGB565 input the OV5640/OV5645 path
//                     already uses, so if PPA YUV420-input turns out
//                     not to work on real hardware there is somewhere
//                     to retreat to that is known-good.

// Byte offsets of the four components within one 4-byte YUV422
// macropixel (two horizontal pixels sharing a U and a V sample).
//
// Naming a *permutation* rather than hardcoding one costs nothing at
// runtime and buys the ability to settle the byte order empirically on
// hardware nobody here can test.
typedef struct {
    uint8_t y0;  // luma of the even column (2k)
    uint8_t v;   // Cr, shared by both columns
    uint8_t y1;  // luma of the odd column (2k+1)
    uint8_t u;   // Cb, shared by both columns
} yuv422_lanes_t;

// The four standard YUV422 wire orders, then the same four with the
// 4-byte word reversed. See the table in yuv_convert.c for why the
// reversed half is not padding.
#define YUV422_ORDER_COUNT   8
#define YUV422_ORDER_DEFAULT 4

extern const yuv422_lanes_t YUV422_ORDERS[YUV422_ORDER_COUNT];
extern const char *const     YUV422_ORDER_NAMES[YUV422_ORDER_COUNT];

// Clamp an arbitrary integer to a valid index into the tables above.
int yuv422_order_clamp(int index);

// Unpack `w` x `h` packed YUV422 into the packed YUV420 layout the P4's
// PPA and hardware H.264 encoder both use (esp_h264_types.h calls it
// ESP_H264_RAW_FMT_O_UYY_E_VYY): even rows are `u y y u y y ...`, odd
// rows are `v y y v y y ...`, so each row is w * 3 / 2 bytes and the
// whole frame is w * h * 3 / 2.
//
// The vertical chroma decimation is free and implicit. YUV422 already
// carries one U and one V per two horizontal pixels on *every* row,
// and the destination wants a U on even rows and a V on odd rows, so
// converting is a matter of discarding the sample each row does not
// need. No averaging, and no arithmetic of any kind.
//
// `w` must be even. `dst` must have room for w * h * 3 / 2 bytes.
void yuv422_to_yuv420(const uint8_t *src, uint8_t *dst,
                      uint32_t w, uint32_t h, yuv422_lanes_t lanes);

// Unpack `w` x `h` packed YUV422 into RGB565, BT.601 limited range --
// which is what the TC358743 emits when programmed for
// MASK_VOUT_COLOR_601_YCBCR_LIMITED, as its driver does.
//
// `w` must be even. `dst` must have room for w * h * 2 bytes.
void yuv422_to_rgb565(const uint8_t *src, uint8_t *dst,
                      uint32_t w, uint32_t h, yuv422_lanes_t lanes);
