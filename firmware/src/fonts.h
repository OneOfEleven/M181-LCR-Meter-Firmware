
#ifndef _FONTS_H
#define _FONTS_H

#ifdef __cplusplus
	extern "C"
	{
#endif

#include <stdint.h>

//  Structure used to define fonts
typedef struct {
	const uint8_t   FontWidth;    // Font width in pixels
	uint8_t         FontHeight;   // Font height in pixels
	const uint16_t *data;         // Pointer to data font data array
} FontDef;

extern const FontDef Font_7x10;
extern const FontDef Font_11x18;
//extern const FontDef Font_16x26;

#ifdef __cplusplus
	}
#endif

#endif
