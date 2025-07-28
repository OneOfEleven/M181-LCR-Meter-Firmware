
#ifndef _FONTS_H
#define _FONTS_H

#ifdef __cplusplus
	extern "C"
	{
#endif

#include <stdint.h>

typedef struct {
	const uint8_t   width;      // Font width in pixels
	const uint8_t   height;     // Font height in pixels
	const uint8_t   char_first; //
	const uint8_t   char_last;  //
	const uint16_t *data;       // Pointer to data font data array
} t_font;

extern const t_font font_8x12;
//extern const t_font font_7x10;
extern const t_font font_11x18;
//extern const t_font font_16x26;
//extern const t_font font_16x24;
extern const t_font font_16x32;

#ifdef __cplusplus
	}
#endif

#endif
