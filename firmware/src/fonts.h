
#ifndef _FONTS_H
#define _FONTS_H

#ifdef __cplusplus
	extern "C"
	{
#endif

#include <stdint.h>

//  Structure used to define fonts
typedef struct {
	const uint8_t   width;    // Font width in pixels
	const uint8_t   height;   // Font height in pixels
	const uint16_t *data;     // Pointer to data font data array
} t_font;

extern const t_font Font_7x10;
extern const t_font Font_11x18;
//extern const t_font Font_16x26;

#ifdef __cplusplus
	}
#endif

#endif
