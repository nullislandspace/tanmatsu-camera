#pragma once

#include "fbdraw.h"

// Try to load and display the splash screen JPEG for 2 seconds.
// Looks for tanmatype.jpg in the app asset directories on SD and
// internal flash. If the file can't be found or decoded, logs a
// warning and returns immediately — not fatal.
void splash_show(fbdraw_t *fb);
