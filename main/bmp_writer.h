#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

// Write an RGB565 pixel buffer out as an uncompressed 24-bit BMP file.
// The file is opened via fastopen() for SD card throughput. Rows are
// written bottom-up as BMP requires.
//
// Intended as a debug helper — diagnostic dumps of the camera preview
// pipeline — but also the simplest path to on-disk pixel data for
// inspection on a host.
esp_err_t bmp_writer_save_rgb565(const char *path, const uint16_t *pixels, uint32_t width, uint32_t height);
