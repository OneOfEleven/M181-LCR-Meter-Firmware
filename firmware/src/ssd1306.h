
#ifndef _SSD1306_H
#define _SSD1306_H

#include <string.h>

#include "stm32f1xx_hal.h"
#include "fonts.h"

#ifndef SSD1306_I2C_ADDR
	#define SSD1306_I2C_ADDR     0x78
#endif

#ifndef SSD1306_WIDTH
	#define SSD1306_WIDTH        128
#endif

#ifndef SSD1306_HEIGHT
	#define SSD1306_HEIGHT       64
#endif

#ifndef SSD1306_COM_LR_REMAP
	#define SSD1306_COM_LR_REMAP 0
#endif

#ifndef SSD1306_COM_ALTERNATIVE_PIN_CONFIG
	#define SSD1306_COM_ALTERNATIVE_PIN_CONFIG    1
#endif

uint8_t _txBuffer[6];
uint8_t _rxBuffer[6];
uint8_t _reg[2];

typedef enum
{
	Black = 0x00, // Black color, no pixel
	White = 0x01, // Pixel is set. Color depends on LCD
} SSD1306_COLOR;

//  Struct to store transformations
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t  Inverted;
	uint8_t  Initialized;
} SSD1306_t;

// uint8_t ssd1306_Init(I2C_HandleTypeDef *hi2c);
uint8_t ssd1306_Init();
//void ssd1306_UpdateScreen(I2C_HandleTypeDef *hi2c);
void ssd1306_UpdateScreen();
void ssd1306_Fill(SSD1306_COLOR color);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
char ssd1306_WriteString(const char *str, FontDef Font, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
void ssd1306_InvertColors(void);
void ssd1306_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);

#endif
