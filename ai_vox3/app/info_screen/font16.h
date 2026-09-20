#ifndef __FONT16_H
#define __FONT16_H
#include <stdint.h>

#define FONT16_W  16   /* glyph width in pixels */
#define FONT16_H  16   /* glyph height in pixels */
#define FONT16_BPR 2   /* bytes per row (1bpp, MSB-first, width 16) */

extern const int FONT16_COUNT;
extern const uint16_t font16_codes[];
extern const uint8_t  font16_data[][FONT16_BPR * FONT16_H];

/* Returns a pointer to the 16x16 packed bitmap for 'code', or NULL. */
const uint8_t *font16_lookup(uint16_t code);

#endif
