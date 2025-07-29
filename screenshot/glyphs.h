#ifndef SCREENSHOT_GLYPHS_H
#define SCREENSHOT_GLYPHS_H

#include <stdint.h>

// initialize the glyph storage directory
void init_glyphs_store(void);

// save a binary glyph bitmap of size w×h (1-bit per pixel)
// bits is a packed bit-array, row-major, 8 pixels per byte
void save_glyph(const uint8_t *bits, int w, int h);

#endif // SCREENSHOT_GLYPHS_H
