#include "yuv_convert.h"

// The eight candidate byte orders.
//
// The first four are the four ways the two luma and two chroma samples
// can be arranged in a macropixel, under their usual names. The second
// four are those same four with the 4-byte word reversed (byte i
// becomes byte 3-i).
//
// The reversed half is not there for completeness. Under ISP bypass the
// driver reconfigures the input width as
//
//     isp_ll_set_intput_data_h_pixel_num(hw,
//         ISP_DIV_ROUND_UP(h_res * in_bits_per_pixel, 32));
//
// with the comment "Hsize now stands for the number of 32-bit in one
// line" (esp_driver_isp/src/isp_core.c). The bypass path is explicitly
// a 32-bit-word conduit, which is exactly where a word-endianness flip
// can live -- and a word flip maps each of the four wire orders onto
// one of the reversed four.
//
// That is not a hypothetical. The MIPI CSI-2 spec order for data type
// 0x1E (YUV422 8-bit), which is what the TC358743 emits when programmed
// for MASK_YCBCRFMT_422_8_BIT, is U Y0 V Y1 -- plain UYVY, index 0.
// The order the PR author converged on empirically, over two separate
// corrections on real hardware, is index 4: UYVY with the word
// reversed. Theory and measurement agree, which is why index 4 is the
// default rather than index 0.
const yuv422_lanes_t YUV422_ORDERS[YUV422_ORDER_COUNT] = {
    //                                   wire order      bytes
    { .y0 = 1, .v = 2, .y1 = 3, .u = 0 },  // 0 UYVY     U  Y0 V  Y1
    { .y0 = 0, .v = 3, .y1 = 2, .u = 1 },  // 1 YUYV     Y0 U  Y1 V
    { .y0 = 0, .v = 1, .y1 = 2, .u = 3 },  // 2 YVYU     Y0 V  Y1 U
    { .y0 = 1, .v = 0, .y1 = 3, .u = 2 },  // 3 VYUY     V  Y0 U  Y1
    { .y0 = 2, .v = 1, .y1 = 0, .u = 3 },  // 4 UYVY-rev Y1 V  Y0 U   <- default
    { .y0 = 3, .v = 0, .y1 = 1, .u = 2 },  // 5 YUYV-rev V  Y1 U  Y0
    { .y0 = 3, .v = 2, .y1 = 1, .u = 0 },  // 6 YVYU-rev U  Y1 V  Y0
    { .y0 = 2, .v = 3, .y1 = 0, .u = 1 },  // 7 VYUY-rev Y1 U  Y0 V
};

const char *const YUV422_ORDER_NAMES[YUV422_ORDER_COUNT] = {
    "UYVY", "YUYV", "YVYU", "VYUY",
    "UYVY-r", "YUYV-r", "YVYU-r", "VYUY-r",
};

int yuv422_order_clamp(int index) {
    if (index < 0 || index >= YUV422_ORDER_COUNT) return YUV422_ORDER_DEFAULT;
    return index;
}

void yuv422_to_yuv420(const uint8_t *src, uint8_t *dst,
                      uint32_t w, uint32_t h, yuv422_lanes_t lanes) {
    if (src == NULL || dst == NULL || w < 2 || h == 0) return;

    const uint32_t dst_stride = w * 3u / 2u;

    for (uint32_t row = 0; row < h; row++) {
        const uint8_t *s = src + (size_t)row * w * 2u;
        uint8_t       *d = dst + (size_t)row * dst_stride;

        // Even rows carry Cb, odd rows carry Cr. Picking the lane once
        // per row is the entire cost of the 422->420 decimation.
        const uint8_t c_lane = (row & 1u) ? lanes.v : lanes.u;

        for (uint32_t x = 0; x < w; x += 2u, s += 4, d += 3) {
            d[0] = s[c_lane];
            d[1] = s[lanes.y0];
            d[2] = s[lanes.y1];
        }
    }
}

// BT.601 limited range, in fixed point with 8 fractional bits:
//   Y contribution:  298 = round(256 * 1.164)
//   V -> R:          409 = round(256 * 1.596)
//   U -> B:          516 = round(256 * 2.017)
//   U -> G:          100 = round(256 * 0.392)
//   V -> G:          208 = round(256 * 0.813)
static inline int clamp8(int x) {
    if (x < 0)   return 0;
    if (x > 255) return 255;
    return x;
}

void yuv422_to_rgb565(const uint8_t *src, uint8_t *dst,
                      uint32_t w, uint32_t h, yuv422_lanes_t lanes) {
    if (src == NULL || dst == NULL || w < 2 || h == 0) return;

    const uint8_t *s = src;
    uint16_t      *d = (uint16_t *)dst;
    const uint32_t n = w * h;

    for (uint32_t i = 0; i < n; i += 2u, s += 4, d += 2) {
        const int y0 = (int)s[lanes.y0] - 16;
        const int y1 = (int)s[lanes.y1] - 16;
        const int u  = (int)s[lanes.u]  - 128;
        const int v  = (int)s[lanes.v]  - 128;

        const int c0 = 298 * y0 + 128;   // +128 rounds the >> 8 below
        const int c1 = 298 * y1 + 128;
        const int rv =  409 * v;
        const int gu = -100 * u;
        const int gv = -208 * v;
        const int bu =  516 * u;

        const int r0 = clamp8((c0 + rv)      >> 8);
        const int g0 = clamp8((c0 + gu + gv) >> 8);
        const int b0 = clamp8((c0 + bu)      >> 8);
        const int r1 = clamp8((c1 + rv)      >> 8);
        const int g1 = clamp8((c1 + gu + gv) >> 8);
        const int b1 = clamp8((c1 + bu)      >> 8);

        d[0] = (uint16_t)(((r0 & 0xF8) << 8) | ((g0 & 0xFC) << 3) | (b0 >> 3));
        d[1] = (uint16_t)(((r1 & 0xF8) << 8) | ((g1 & 0xFC) << 3) | (b1 >> 3));
    }
}
