// Host-side verification of main/yuv_convert.c. No IDF, no hardware.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "yuv_convert.h"

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
    printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

// Build a YUV422 frame in a chosen wire order from per-macropixel
// (y0,u,y1,v) samples supplied by a callback.
static void pack422(uint8_t *dst, uint32_t w, uint32_t h, yuv422_lanes_t L,
                    void (*gen)(uint32_t x2, uint32_t y,
                                uint8_t *y0, uint8_t *u, uint8_t *y1, uint8_t *v)) {
    for (uint32_t row = 0; row < h; row++)
        for (uint32_t x = 0; x < w / 2; x++) {
            uint8_t y0, u, y1, v;
            gen(x, row, &y0, &u, &y1, &v);
            uint8_t *p = dst + ((size_t)row * (w / 2) + x) * 4;
            p[L.y0] = y0; p[L.u] = u; p[L.y1] = y1; p[L.v] = v;
        }
}

static void gen_ramp(uint32_t x2, uint32_t y,
                     uint8_t *y0, uint8_t *u, uint8_t *y1, uint8_t *v) {
    *y0 = (uint8_t)(16 + ((2 * x2)     % 220));
    *y1 = (uint8_t)(16 + ((2 * x2 + 1) % 220));
    *u  = (uint8_t)(x2 % 256);
    *v  = (uint8_t)((y * 7 + x2 * 3) % 256);
}

// ---- test 1: 420 packing is exactly O_UYY_E_VYY, for every order ----
static void test_420_layout(void) {
    const uint32_t w = 16, h = 8;
    uint8_t *src = malloc(w * h * 2), *dst = malloc(w * h * 3 / 2);
    for (int oi = 0; oi < YUV422_ORDER_COUNT; oi++) {
        yuv422_lanes_t L = YUV422_ORDERS[oi];
        pack422(src, w, h, L, gen_ramp);
        memset(dst, 0xAA, w * h * 3 / 2);
        yuv422_to_yuv420(src, dst, w, h, L);
        for (uint32_t row = 0; row < h; row++) {
            const uint8_t *d = dst + (size_t)row * (w * 3 / 2);
            for (uint32_t x = 0; x < w / 2; x++) {
                uint8_t y0, u, y1, v;
                gen_ramp(x, row, &y0, &u, &y1, &v);
                uint8_t want_c = (row & 1) ? v : u;   // even row -> U, odd -> V
                CHECK(d[x*3+0] == want_c, "order %d row %u x %u chroma %02x != %02x",
                      oi, row, x, d[x*3+0], want_c);
                CHECK(d[x*3+1] == y0, "order %d row %u x %u y0 %02x != %02x",
                      oi, row, x, d[x*3+1], y0);
                CHECK(d[x*3+2] == y1, "order %d row %u x %u y1 %02x != %02x",
                      oi, row, x, d[x*3+2], y1);
            }
        }
    }
    free(src); free(dst);
    printf("test_420_layout: all %d orders round-trip exactly\n", YUV422_ORDER_COUNT);
}

// ---- test 2: the 8 orders are distinct permutations ----
static void test_orders_distinct(void) {
    for (int i = 0; i < YUV422_ORDER_COUNT; i++) {
        yuv422_lanes_t a = YUV422_ORDERS[i];
        int seen[4] = {0,0,0,0};
        seen[a.y0]++; seen[a.v]++; seen[a.y1]++; seen[a.u]++;
        for (int b = 0; b < 4; b++)
            CHECK(seen[b] == 1, "order %d (%s) is not a permutation: lane %d used %d times",
                  i, YUV422_ORDER_NAMES[i], b, seen[b]);
        for (int j = i + 1; j < YUV422_ORDER_COUNT; j++) {
            yuv422_lanes_t c = YUV422_ORDERS[j];
            CHECK(!(a.y0==c.y0 && a.v==c.v && a.y1==c.y1 && a.u==c.u),
                  "orders %d and %d are identical", i, j);
        }
    }
    // second half must be the first half with the word reversed
    for (int i = 0; i < 4; i++) {
        yuv422_lanes_t a = YUV422_ORDERS[i], r = YUV422_ORDERS[i + 4];
        CHECK(r.y0 == 3 - a.y0 && r.v == 3 - a.v && r.y1 == 3 - a.y1 && r.u == 3 - a.u,
              "order %d is not the word-reverse of order %d", i + 4, i);
    }
    // index 0 must be genuine UYVY: bytes U Y0 V Y1
    yuv422_lanes_t z = YUV422_ORDERS[0];
    CHECK(z.u == 0 && z.y0 == 1 && z.v == 2 && z.y1 == 3, "order 0 is not UYVY");
    // the default must be UYVY word-reversed == the PR's empirical result
    yuv422_lanes_t d = YUV422_ORDERS[YUV422_ORDER_DEFAULT];
    CHECK(d.y0 == 2 && d.v == 1 && d.y1 == 0 && d.u == 3,
          "default order is not the PR's measured y0=s[2] v=s[1] y1=s[0] u=s[3]");
    printf("test_orders_distinct: 8 distinct permutations, reversal + default correct\n");
}

// ---- test 3: RGB565 against known BT.601 limited-range colours ----
static void one_colour(const char *name, uint8_t Y, uint8_t U, uint8_t V,
                       int er, int eg, int eb, int tol) {
    yuv422_lanes_t L = YUV422_ORDERS[0];
    uint8_t src[4], dst[4];
    src[L.y0] = Y; src[L.y1] = Y; src[L.u] = U; src[L.v] = V;
    yuv422_to_rgb565(src, dst, 2, 1, L);
    uint16_t px = (uint16_t)(dst[0] | (dst[1] << 8));
    int r = ((px >> 11) & 0x1F) << 3, g = ((px >> 5) & 0x3F) << 2, b = (px & 0x1F) << 3;
    CHECK(abs(r-er) <= tol && abs(g-eg) <= tol && abs(b-eb) <= tol,
          "%s: got (%d,%d,%d) want (%d,%d,%d)", name, r, g, b, er, eg, eb);
    printf("  %-8s Y=%3u U=%3u V=%3u -> rgb(%3d,%3d,%3d)\n", name, Y, U, V, r, g, b);
}

static void test_rgb565_colours(void) {
    printf("test_rgb565_colours:\n");
    one_colour("black",  16, 128, 128,   0,   0,   0, 8);
    one_colour("white", 235, 128, 128, 255, 255, 255, 8);
    one_colour("grey",  126, 128, 128, 128, 128, 128, 8);
    one_colour("red",    81,  90, 240, 255,   0,   0, 12);
    one_colour("green", 145,  54,  34,   0, 255,   0, 12);
    one_colour("blue",   41, 240, 110,   0,   0, 255, 12);
}

// ---- test 4: no write outside the destination ----
static void test_bounds(void) {
    const uint32_t w = 32, h = 16;
    size_t n420 = (size_t)w * h * 3 / 2, n565 = (size_t)w * h * 2;
    uint8_t *src = malloc(w * h * 2);
    uint8_t *buf = malloc(n565 + 64);
    pack422(src, w, h, YUV422_ORDERS[0], gen_ramp);

    memset(buf, 0x5A, n565 + 64);
    yuv422_to_yuv420(src, buf, w, h, YUV422_ORDERS[0]);
    for (size_t i = n420; i < n420 + 64; i++)
        CHECK(buf[i] == 0x5A, "yuv420 wrote past the end at +%zu", i - n420);

    memset(buf, 0x5A, n565 + 64);
    yuv422_to_rgb565(src, buf, w, h, YUV422_ORDERS[0]);
    for (size_t i = n565; i < n565 + 64; i++)
        CHECK(buf[i] == 0x5A, "rgb565 wrote past the end at +%zu", i - n565);

    free(src); free(buf);
    printf("test_bounds: neither converter overruns its destination\n");
}

int main(void) {
    test_orders_distinct();
    test_420_layout();
    test_rgb565_colours();
    test_bounds();
    printf("\n%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
    return fails != 0;
}
