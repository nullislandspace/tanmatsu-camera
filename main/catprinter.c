#include "catprinter.h"

#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "os/os_mbuf.h"

static const char *TAG = "catprinter";

// For some reason the printer advertises a different 16-bit service
// UUID than the one its actual GATT service uses.
#define CATPRINTER_SVC_UUID_ADVERTISED 0xaf30
#define CATPRINTER_SVC_UUID 0xae30
#define CATPRINTER_CHR_TX_UUID 0xae01 // characteristic we write commands to
#define CATPRINTER_CHR_RX_UUID 0xae02 // characteristic we subscribe to for status

// Printer prints a fixed 384-dot-wide line (48 bytes, 1 bit/dot).
#define PRINTER_WIDTH_PX 384
#define PRINTER_ROW_BYTES (PRINTER_WIDTH_PX / 8)

static uint16_t s_conn_handle = 0;
static volatile bool s_ready = false;
static volatile bool s_busy = false;
static volatile catprinter_paper_status_t s_paper_status =
    CATPRINTER_PAPER_UNKNOWN;

bool catprinter_matches_disc(const struct ble_gap_disc_desc *disc) {
  struct ble_hs_adv_fields fields;
  if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) != 0) {
    return false;
  }
  for (int i = 0; i < fields.num_uuids16; i++) {
    uint16_t uuid = ble_uuid_u16(&fields.uuids16[i].u);
    if (uuid == CATPRINTER_SVC_UUID || uuid == CATPRINTER_SVC_UUID_ADVERTISED) {
      return true;
    }
  }
  return false;
}

static int on_subscribe_write(uint16_t conn_handle,
                              const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr, void *arg) {
  (void)attr;
  (void)arg;
  if (error->status != 0) {
    ESP_LOGE(TAG, "failed to subscribe to printer notifications; status=%d",
             error->status);
    return 0;
  }
  ESP_LOGI(TAG, "cat printer ready; conn_handle=%d", conn_handle);
  s_conn_handle = conn_handle;
  s_ready = true;
  return 0;
}

void catprinter_on_disc_complete(const struct peer *peer) {
  const struct peer_svc *svc =
      peer_svc_find_uuid(peer, BLE_UUID16_DECLARE(CATPRINTER_SVC_UUID));
  if (svc == NULL) {
    return; // not a cat printer
  }

  const struct peer_dsc *dsc = peer_dsc_find_uuid(
      peer, BLE_UUID16_DECLARE(CATPRINTER_SVC_UUID),
      BLE_UUID16_DECLARE(CATPRINTER_CHR_RX_UUID),
      BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16));
  if (dsc == NULL) {
    ESP_LOGE(TAG, "cat printer lacks a CCCD on its RX characteristic");
    return;
  }

  uint8_t value[2] = {1, 0};
  int rc = ble_gattc_write_flat(peer->conn_handle, dsc->dsc.handle, value,
                                sizeof(value), on_subscribe_write, NULL);
  if (rc != 0) {
    ESP_LOGE(TAG, "failed to subscribe to cat printer; rc=%d", rc);
  }
}

void catprinter_on_disconnect(uint16_t conn_handle) {
  if (conn_handle != s_conn_handle) {
    return;
  }
  s_ready = false;
  s_conn_handle = 0;
  s_paper_status = CATPRINTER_PAPER_UNKNOWN;
}

void catprinter_on_notify(uint16_t conn_handle, uint16_t attr_handle,
                          struct os_mbuf *om) {
  (void)attr_handle;
  if (conn_handle != s_conn_handle) {
    return;
  }
  uint8_t data[16];
  uint16_t len = OS_MBUF_PKTLEN(om);
  if (len > sizeof(data)) {
    len = sizeof(data);
  }
  os_mbuf_copydata(om, 0, len, data);

  // Device state response (cmd 0xa3): payload[0] == 1 -> no paper.
  if (len >= 9 && data[0] == 0x51 && data[1] == 0x78 && data[2] == 0xa3) {
    s_paper_status =
        data[6] ? CATPRINTER_PAPER_OUT : CATPRINTER_PAPER_PRESENT;
  }
}

bool catprinter_is_ready(void) { return s_ready; }

bool catprinter_is_busy(void) { return s_busy; }

catprinter_paper_status_t catprinter_paper_status(void) {
  return s_paper_status;
}

// ===== GATT command protocol =====
//
// Packet format: 51 78 <cmd> 00 <len_lo> <len_hi> <payload...> <crc8> ff.
// Ported from the reference cat-printer BLE client; the length field
// is widened to 16 bits here (the original hardcoded a zero high
// byte) because Floyd-Steinberg-dithered photo lines can legitimately
// need an RLE payload up to PRINTER_WIDTH_PX bytes, well past 255.

static uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
  }
  return crc;
}

static void ble_write_raw(uint16_t conn_handle, uint16_t val_handle,
                          const uint8_t *data, size_t len) {
  const struct peer *peer = peer_find(conn_handle);
  uint16_t max_chunk = (peer && peer->mtu > 3) ? peer->mtu - 3 : 20;
  size_t pos = 0;
  while (pos < len) {
    size_t chunk = len - pos;
    if (chunk > max_chunk) {
      chunk = max_chunk;
    }
    int rc = ble_gattc_write_no_rsp_flat(conn_handle, val_handle, &data[pos],
                                         chunk);
    if (rc == BLE_HS_ENOMEM) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (rc != 0) {
      ESP_LOGE(TAG, "BLE write failed: rc=%d", rc);
      return;
    }
    pos += chunk;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Max payload we ever send is one dithered print line's worst-case
// RLE expansion (one run per pixel = PRINTER_WIDTH_PX bytes).
#define MAX_PAYLOAD_LEN PRINTER_WIDTH_PX

static void write_packet(uint16_t conn_handle, uint16_t val_handle,
                         uint8_t cmd, const uint8_t *payload,
                         uint16_t payload_len) {
  uint8_t buf[8 + MAX_PAYLOAD_LEN];
  if (payload_len > MAX_PAYLOAD_LEN) {
    ESP_LOGE(TAG, "payload too large: %d", payload_len);
    return;
  }
  buf[0] = 0x51;
  buf[1] = 0x78;
  buf[2] = cmd;
  buf[3] = 0x00;
  buf[4] = (uint8_t)(payload_len & 0xff);
  buf[5] = (uint8_t)(payload_len >> 8);
  memcpy(&buf[6], payload, payload_len);
  buf[6 + payload_len] = crc8(payload, payload_len);
  buf[7 + payload_len] = 0xff;
  ble_write_raw(conn_handle, val_handle, buf, 8 + payload_len);
}

// cmd 0xa3 - query device status (CMD_GET_DEV_STATE)
static void write_get_device_state(uint16_t conn_handle, uint16_t val_handle) {
  uint8_t payload[] = {0x00};
  write_packet(conn_handle, val_handle, 0xa3, payload, sizeof(payload));
}

// cmd 0xa4 - set print quality / 200 DPI mode (default 0x32)
static void write_set_quality(uint16_t conn_handle, uint16_t val_handle,
                              uint8_t quality) {
  write_packet(conn_handle, val_handle, 0xa4, &quality, 1);
}

// cmd 0xaf - set print energy, 16-bit little-endian
static void write_set_energy(uint16_t conn_handle, uint16_t val_handle,
                             uint16_t energy) {
  uint8_t payload[] = {energy & 0xff, energy >> 8};
  write_packet(conn_handle, val_handle, 0xaf, payload, sizeof(payload));
}

// cmd 0xbe - set print mode: 0x00 = image, 0x01 = text. Empirically
// the RLE bitmap-line path below only prints correctly with 0x01, so
// that's what we send regardless of content.
static void write_set_print_mode(uint16_t conn_handle, uint16_t val_handle,
                                 uint8_t mode) {
  write_packet(conn_handle, val_handle, 0xbe, &mode, 1);
}

// cmd 0xa6 - draw lattice border line (11-byte pattern)
static void write_draw_lattice(uint16_t conn_handle, uint16_t val_handle,
                               const uint8_t data[11]) {
  write_packet(conn_handle, val_handle, 0xa6, data, 11);
}

static void write_lattice_start(uint16_t conn_handle, uint16_t val_handle) {
  static const uint8_t data[] = {0xaa, 0x55, 0x17, 0x38, 0x44,
                                0x5f, 0x5f, 0x5f, 0x44, 0x38, 0x2c};
  write_draw_lattice(conn_handle, val_handle, data);
}

static void write_lattice_end(uint16_t conn_handle, uint16_t val_handle) {
  static const uint8_t data[] = {0xaa, 0x55, 0x17, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x17};
  write_draw_lattice(conn_handle, val_handle, data);
}

// cmd 0xbd - feed paper to tear-off position (default 0x19 = 25 lines)
static void write_feed_to_tear(uint16_t conn_handle, uint16_t val_handle,
                               uint8_t lines) {
  write_packet(conn_handle, val_handle, 0xbd, &lines, 1);
}

// cmd 0xa1 - feed N lines, 16-bit little-endian
static void write_feed_paper(uint16_t conn_handle, uint16_t val_handle,
                             uint16_t lines) {
  uint8_t payload[] = {lines & 0xff, lines >> 8};
  write_packet(conn_handle, val_handle, 0xa1, payload, sizeof(payload));
}

// cmd 0xbf - print one RLE-compressed bitmap line
static void write_print_rle_line(uint16_t conn_handle, uint16_t val_handle,
                                 const uint8_t *rle_data, uint16_t len) {
  write_packet(conn_handle, val_handle, 0xbf, rle_data, len);
}

// Run-length encode one PRINTER_ROW_BYTES-byte, LSB-first packed
// bitmap row (bit=1 -> white/no-ink, bit=0 -> black/ink) into the
// printer's line format: a plain byte count for a white run, or
// 0x80|count for a black run (count is 7 bits, so runs longer than
// 127 dots are split). `out` must hold at least PRINTER_WIDTH_PX
// bytes — the worst case is one run per pixel (a fully alternating
// dithered line), which the original reference implementation's
// 128-byte buffer did not account for.
static uint16_t rle_encode_line(const uint8_t bitmap[PRINTER_ROW_BYTES],
                                uint8_t out[PRINTER_WIDTH_PX]) {
  uint16_t len = 0;
  int px = 0;
  while (px < PRINTER_WIDTH_PX) {
    int color = (bitmap[px / 8] >> (px % 8)) & 1;
    int count = 0;
    while (px < PRINTER_WIDTH_PX && count < 127) {
      if (((bitmap[px / 8] >> (px % 8)) & 1) != color) {
        break;
      }
      count++;
      px++;
    }
    out[len++] = color ? (uint8_t)count : (uint8_t)(0x80 | count);
  }
  return len;
}

// ===== Print job =====

typedef struct {
  uint16_t conn_handle;
  uint16_t val_handle;
  uint8_t *bitmap; // out_h * PRINTER_ROW_BYTES bytes
  uint32_t out_h;
} print_job_t;

static void print_task(void *arg) {
  print_job_t *job = arg;

  write_get_device_state(job->conn_handle, job->val_handle);
  write_set_quality(job->conn_handle, job->val_handle, 0x32);
  write_set_energy(job->conn_handle, job->val_handle, 0xffff);
  write_set_print_mode(job->conn_handle, job->val_handle, 0x01);
  write_lattice_start(job->conn_handle, job->val_handle);

  uint8_t rle_buf[PRINTER_WIDTH_PX];
  for (uint32_t row = 0; row < job->out_h; row++) {
    uint16_t rle_len =
        rle_encode_line(&job->bitmap[row * PRINTER_ROW_BYTES], rle_buf);
    write_print_rle_line(job->conn_handle, job->val_handle, rle_buf, rle_len);
  }

  write_feed_to_tear(job->conn_handle, job->val_handle, 0x19);
  write_feed_paper(job->conn_handle, job->val_handle, 0x0030);
  write_feed_paper(job->conn_handle, job->val_handle, 0x0030);
  write_feed_paper(job->conn_handle, job->val_handle, 0x0030);
  write_lattice_end(job->conn_handle, job->val_handle);
  write_get_device_state(job->conn_handle, job->val_handle);

  heap_caps_free(job->bitmap);
  free(job);
  s_busy = false;
  vTaskDelete(NULL);
}

static inline uint8_t rgb565_luma(uint16_t px) {
  uint8_t r5 = (px >> 11) & 0x1f;
  uint8_t g6 = (px >> 5) & 0x3f;
  uint8_t b5 = px & 0x1f;
  uint8_t r = (uint8_t)((r5 << 3) | (r5 >> 2));
  uint8_t g = (uint8_t)((g6 << 2) | (g6 >> 4));
  uint8_t b = (uint8_t)((b5 << 3) | (b5 >> 2));
  return (uint8_t)((r * 299u + g * 587u + b * 114u) / 1000u);
}

esp_err_t catprinter_print_rgb565(const uint16_t *pixels, uint32_t width,
                                  uint32_t height) {
  if (!s_ready) {
    return ESP_ERR_INVALID_STATE;
  }
  if (s_busy) {
    return ESP_ERR_INVALID_STATE;
  }
  if (pixels == NULL || width == 0 || height == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  const struct peer *peer = peer_find(s_conn_handle);
  if (peer == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  const struct peer_chr *chr =
      peer_chr_find_uuid(peer, BLE_UUID16_DECLARE(CATPRINTER_SVC_UUID),
                        BLE_UUID16_DECLARE(CATPRINTER_CHR_TX_UUID));
  if (chr == NULL) {
    return ESP_ERR_NOT_FOUND;
  }

  uint32_t out_w = PRINTER_WIDTH_PX;
  uint32_t out_h = (uint32_t)((uint64_t)height * out_w / width);
  if (out_h == 0) {
    out_h = 1;
  }

  uint8_t *bitmap =
      heap_caps_calloc(out_h, PRINTER_ROW_BYTES, MALLOC_CAP_8BIT);
  if (bitmap == NULL) {
    return ESP_ERR_NO_MEM;
  }

  // Area-average downscale straight into greyscale, with a rolling
  // 2-row error buffer for Floyd-Steinberg dithering — avoids ever
  // materialising a full-size intermediate image for what's a fairly
  // small (384-wide) output.
  float *err_cur = calloc(out_w, sizeof(float));
  float *err_next = calloc(out_w, sizeof(float));
  if (err_cur == NULL || err_next == NULL) {
    free(err_cur);
    free(err_next);
    heap_caps_free(bitmap);
    return ESP_ERR_NO_MEM;
  }

  for (uint32_t oy = 0; oy < out_h; oy++) {
    uint32_t sy0 = (uint32_t)((uint64_t)oy * height / out_h);
    uint32_t sy1 = (uint32_t)((uint64_t)(oy + 1) * height / out_h);
    if (sy1 <= sy0) {
      sy1 = sy0 + 1;
    }
    if (sy1 > height) {
      sy1 = height;
    }

    memset(err_next, 0, out_w * sizeof(float));

    for (uint32_t ox = 0; ox < out_w; ox++) {
      uint32_t sx0 = (uint32_t)((uint64_t)ox * width / out_w);
      uint32_t sx1 = (uint32_t)((uint64_t)(ox + 1) * width / out_w);
      if (sx1 <= sx0) {
        sx1 = sx0 + 1;
      }
      if (sx1 > width) {
        sx1 = width;
      }

      uint32_t sum = 0, n = 0;
      for (uint32_t sy = sy0; sy < sy1; sy++) {
        const uint16_t *row = pixels + (size_t)sy * width;
        for (uint32_t sx = sx0; sx < sx1; sx++) {
          sum += rgb565_luma(row[sx]);
          n++;
        }
      }
      float gray = (n ? (float)sum / (float)n : 0.0f) + err_cur[ox];

      // Paper is white; a dot only gets marked when it's dark enough
      // to print ink. Bit convention matches rle_encode_line() above:
      // 1 = white/no-ink, 0 = black/ink.
      bool white = gray >= 128.0f;
      float actual = white ? 255.0f : 0.0f;
      float qerr = gray - actual;

      if (ox + 1 < out_w) {
        err_cur[ox + 1] += qerr * (7.0f / 16.0f);
      }
      if (ox > 0) {
        err_next[ox - 1] += qerr * (3.0f / 16.0f);
      }
      err_next[ox] += qerr * (5.0f / 16.0f);
      if (ox + 1 < out_w) {
        err_next[ox + 1] += qerr * (1.0f / 16.0f);
      }

      if (white) {
        bitmap[oy * PRINTER_ROW_BYTES + (ox / 8)] |= (uint8_t)(1u << (ox % 8));
      }
    }

    float *tmp = err_cur;
    err_cur = err_next;
    err_next = tmp;
  }

  free(err_cur);
  free(err_next);

  print_job_t *job = malloc(sizeof(*job));
  if (job == NULL) {
    heap_caps_free(bitmap);
    return ESP_ERR_NO_MEM;
  }
  job->conn_handle = s_conn_handle;
  job->val_handle = chr->chr.val_handle;
  job->bitmap = bitmap;
  job->out_h = out_h;

  s_busy = true;
  if (xTaskCreate(print_task, "catprinter_print", 8192, job, 5, NULL) !=
      pdPASS) {
    s_busy = false;
    heap_caps_free(bitmap);
    free(job);
    return ESP_FAIL;
  }
  return ESP_OK;
}
