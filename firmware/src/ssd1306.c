
#include <math.h>

#include "main.h"
#include "ssd1306.h"
#include "stm32_sw_i2c.h"
#include "delay.h"

static const uint8_t SSD1306_init_commands[] = {
	0xAE,       // Display off
	0x20,       // Set Memory Addressing Mode
	0x10,       // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	0xB0,       // Set Page Start Address for Page Addressing Mode,0-7
	0xC8,       // Set COM Output Scan Direction
	0x00,       // Set low column address
	0x10,       // Set high column address
	0x40,       // Set start line address
	0x81,       // set contrast control register
	0xFF,
	0xA1,       // Set segment re-map 0 to 127
	0xA6,       // Set normal display

	0xA8,       // Set multiplex ratio(1 to 64)
	SSD1306_HEIGHT - 1,

	0xA4,       // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	0xD3,       // Set display offset
	0x00,       // No offset
	0xD5,       // Set display clock divide ratio/oscillator frequency
	0xF0,       // Set divide ratio
	0xD9,       // Set pre-charge period
	0x22,

	0xDA,       // Set com pins hardware configuration
	SSD1306_COM_LR_REMAP << 5 | SSD1306_COM_ALTERNATIVE_PIN_CONFIG << 4 | 0x02,

	0xDB,       // Set vcomh
	0x20,       // 0x20,0.77xVcc
	0x8D,       // Set DC-DC enable
	0x14,       //
	0xAF        // Turn on SSD1306 panel
};

// 1-bit per pixel screen buffer
uint8_t SSD1306_Buffer[SSD1306_WIDTH * (SSD1306_HEIGHT / 8)];

#define USE_LINE_TABLE           // to speed up the pixel drawing

#ifdef USE_LINE_TABLE
	uint16_t line_table[SSD1306_HEIGHT]       = {0};
	uint8_t  line_table_pixel[SSD1306_HEIGHT] = {0};
#endif

SSD1306_t SSD1306 = {0};

// send a byte to the command register
uint8_t ssd1306_WriteCommand(const uint8_t command)
{
	const uint8_t txBuffer[] = {0x00, command};
	I2C_transmit(SSD1306_I2C_ADDR, txBuffer, sizeof(txBuffer));
	return 0;

	// return HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, 0x00, 1, &command, 1, 10);
}

//  initialize the oled screen and line table
//
uint8_t ssd1306_Init(void)
{
	SSD1306.Initialized = 0;

	#ifdef USE_LINE_TABLE
		for (uint16_t y = 0; y < SSD1306_HEIGHT; y++)
		{
			line_table[y]       = (y / 8) * SSD1306_WIDTH;
			line_table_pixel[y] = 1u << (y % 8);
		}
	#endif

	// Wait for the screen to boot
	HAL_Delay(100);

	// Init LCD
	for (unsigned int i = 0; i < ARRAY_SIZE(SSD1306_init_commands); i++)
		ssd1306_WriteCommand(SSD1306_init_commands[i]); // Display off

	// Clear screen
	ssd1306_Fill(Black);

	// Flush buffer to screen
	ssd1306_UpdateScreen();

	// Set default values for screen object
	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;

	SSD1306.Initialized = 1;

	return 0;
}

//  fill the whole screen with the given color
void ssd1306_Fill(SSD1306_COLOR color)
{
	memset(SSD1306_Buffer, (color == Black) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

//  write the screenbuffer with changed to the screen
//
void ssd1306_UpdateScreen(void)
{
	uint8_t txBuffer[SSD1306_WIDTH + 1]; // +1 for control byte

	txBuffer[0] = 0x40;                  // Control byte for data

	for (int i = 0; i < 8; i++)
	{
		ssd1306_WriteCommand(0xB0 + i);
		ssd1306_WriteCommand(0x00);
		ssd1306_WriteCommand(0x10);
		memcpy(&txBuffer[1], &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
		I2C_transmit(SSD1306_I2C_ADDR, txBuffer, SSD1306_WIDTH + 1);
		// HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, 0x40, 1, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 100);
	}
}

//  draw one pixel in the screenbuffer
//  x     .. X Coordinate
//  y     .. Y Coordinate
//  color .. Pixel color
//
void ssd1306_DrawPixel(const unsigned int x, const unsigned int y, SSD1306_COLOR color)
{
	if (!SSD1306.Initialized || x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
		return;

	if (SSD1306.Inverted)
		color = !color;

	#ifdef USE_LINE_TABLE
		const unsigned int m = line_table[y] + x;
		const unsigned int p = line_table_pixel[y];
		if (color == White)
			SSD1306_Buffer[m] |=  p;
		else
			SSD1306_Buffer[m] &= ~p;
	#else
		if (color == White)
			SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |=   1u << (y % 8);
		else
			SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1u << (y % 8));
	#endif
}

//  draw 1 char to the screen buffer
//  ch    .. Character to write
//  Font  .. Font to use
//  color .. Black or White
//
char ssd1306_WriteChar(const char ch, const t_font *font, const SSD1306_COLOR color)
{
	if (!SSD1306.Initialized || SSD1306_WIDTH <= SSD1306.CurrentX || SSD1306_HEIGHT <= SSD1306.CurrentY)
		return 0;
	if (ch < font->char_first || ch > font->char_last)
		return 0;

	const unsigned int h = font->height;
	const unsigned int w = font->width;
	const unsigned int m = ((unsigned int)ch - font->char_first) * h;

	for (unsigned int y = 0; y < h; y++)
	{
		const unsigned int b  = font->data[m + y];
		const unsigned int yy = SSD1306.CurrentY + y;
		if (yy >= SSD1306_HEIGHT)
			break;
		for (unsigned int x = 0; x < w; x++)
		{
			const unsigned int xx = SSD1306.CurrentX + x;
			if (xx >= SSD1306_WIDTH)
				break;
			ssd1306_DrawPixel(xx, yy, ((b << x) & 0x8000) ? color : !color);
		}
	}

	SSD1306.CurrentX += font->width;

	return ch;
}

// write full string to screen buffer
//
char ssd1306_WriteString(const char str[], const t_font *font, const SSD1306_COLOR color)
{
	while (*str)
		ssd1306_WriteChar(*str++, font, color);
	return *str;
}

// invert background/foreground colors
//
void ssd1306_InvertColors(void)
{
	SSD1306.Inverted = !SSD1306.Inverted;
}

// set cursor position
//
void ssd1306_SetCursor(const uint16_t x, const uint16_t y)
{
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

// get cursor position
//
void ssd1306_GetCursor(uint16_t *x, uint16_t *y)
{
	*x = SSD1306.CurrentX;
	*y = SSD1306.CurrentY;
}

// move cursor
//
void ssd1306_MoveCursor(const int x, const int y)
{
	SSD1306.CurrentX = (((int)SSD1306.CurrentX + x) <= 0) ? 0 : (((int)SSD1306.CurrentX + x) > SSD1306_WIDTH ) ? SSD1306_WIDTH  : SSD1306.CurrentX + x;
	SSD1306.CurrentY = (((int)SSD1306.CurrentY + y) <= 0) ? 0 : (((int)SSD1306.CurrentY + y) > SSD1306_HEIGHT) ? SSD1306_HEIGHT : SSD1306.CurrentY + y;
}

void ssd1306_FillRectangle(const uint8_t x1, const uint8_t y1, const uint8_t x2, const uint8_t y2, const SSD1306_COLOR color)
{
	if (SSD1306.Initialized)
	{
		const unsigned int x_start = (x1 <= x2) ? x1 : x2;
		const unsigned int x_end   = (x1 <= x2) ? x2 : x1;
		const unsigned int y_start = (y1 <= y2) ? y1 : y2;
		const unsigned int y_end   = (y1 <= y2) ? y2 : y1;

		for (unsigned int y = y_start; y <= y_end && y < SSD1306_HEIGHT; y++)
			for (unsigned int x = x_start; x <= x_end && x < SSD1306_WIDTH; x++)
				ssd1306_DrawPixel(x, y, color);
	}
}

void ssd1306_dotted_hline(const unsigned int x1, const unsigned int x2, const unsigned int x_step, const unsigned int y, const SSD1306_COLOR colour)
{
	if (SSD1306.Initialized)
		for (unsigned int x = x1; x <= x2; x += x_step)
			ssd1306_DrawPixel(x, y, colour);
}

void ssd1306_symbol(const unsigned int x, const unsigned int y, const uint16_t symbol[], const unsigned int symbol_width, const unsigned int symbol_height)
{
    for (unsigned int row = 0; row < symbol_height; row++)
    {
        uint16_t row_data = symbol[row] << (16u - symbol_width);
        for (unsigned int col = 0; col < symbol_width; col++, row_data <<= 1)
			ssd1306_DrawPixel(x + col, y + row, (row_data & 0x8000) ? White : Black);
    }
}
