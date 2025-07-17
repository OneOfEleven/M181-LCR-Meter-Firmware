
#include <math.h>

#include "ssd1306.h"
#include "stm32_sw_i2c.h"
#include "delay.h"

// 1-bit per pixel screen buffer
uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

#define USE_LINE_TABLE           // to speed up the pixel drawing

#ifdef USE_LINE_TABLE
	uint16_t line_table[SSD1306_HEIGHT]       = {0};
	uint8_t  line_table_pixel[SSD1306_HEIGHT] = {0};
#endif

SSD1306_t SSD1306;

//  Send a byte to the command register
static uint8_t ssd1306_WriteCommand(const uint8_t command)
{
	const uint8_t txBuffer[] = {0x00, command};
	I2C_transmit(SSD1306_I2C_ADDR, txBuffer, sizeof(txBuffer));
	return 0;

	// return HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, 0x00, 1, &command, 1, 10);
}

//  Initialize the oled screen and line table
//
// uint8_t ssd1306_Init(I2C_HandleTypeDef *hi2c)
uint8_t ssd1306_Init(void)
{
	#ifdef USE_LINE_TABLE
		for (uint16_t y = 0; y < SSD1306_HEIGHT; y++)
		{
			line_table[y]       = (y / 8) * SSD1306_WIDTH;
			line_table_pixel[y] = 1u << (y % 8);
		}
	#endif

	// Wait for the screen to boot
	HAL_Delay(100);

	int status = 0;

	// Init LCD
	status += ssd1306_WriteCommand(0xAE); // Display off
	status += ssd1306_WriteCommand(0x20); // Set Memory Addressing Mode
	status += ssd1306_WriteCommand(0x10); // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	status += ssd1306_WriteCommand(0xB0); // Set Page Start Address for Page Addressing Mode,0-7
	status += ssd1306_WriteCommand(0xC8); // Set COM Output Scan Direction
	status += ssd1306_WriteCommand(0x00); // Set low column address
	status += ssd1306_WriteCommand(0x10); // Set high column address
	status += ssd1306_WriteCommand(0x40); // Set start line address
	status += ssd1306_WriteCommand(0x81); // set contrast control register
	status += ssd1306_WriteCommand(0xFF);
	status += ssd1306_WriteCommand(0xA1); // Set segment re-map 0 to 127
	status += ssd1306_WriteCommand(0xA6); // Set normal display

	status += ssd1306_WriteCommand(0xA8); // Set multiplex ratio(1 to 64)
	status += ssd1306_WriteCommand(SSD1306_HEIGHT - 1);

	status += ssd1306_WriteCommand(0xA4); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	status += ssd1306_WriteCommand(0xD3); // Set display offset
	status += ssd1306_WriteCommand(0x00); // No offset
	status += ssd1306_WriteCommand(0xD5); // Set display clock divide ratio/oscillator frequency
	status += ssd1306_WriteCommand(0xF0); // Set divide ratio
	status += ssd1306_WriteCommand(0xD9); // Set pre-charge period
	status += ssd1306_WriteCommand(0x22);

	status += ssd1306_WriteCommand(0xDA); // Set com pins hardware configuration
	status += ssd1306_WriteCommand(SSD1306_COM_LR_REMAP << 5 | SSD1306_COM_ALTERNATIVE_PIN_CONFIG << 4 | 0x02);

	status += ssd1306_WriteCommand(0xDB); // Set vcomh
	status += ssd1306_WriteCommand(0x20); // 0x20,0.77xVcc
	status += ssd1306_WriteCommand(0x8D); // Set DC-DC enable
	status += ssd1306_WriteCommand(0x14); //
	status += ssd1306_WriteCommand(0xAF); // Turn on SSD1306 panel

	if (status != 0)
		return 1;

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

//  Fill the whole screen with the given color
void ssd1306_Fill(SSD1306_COLOR color)
{
	memset(SSD1306_Buffer, (color == Black) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

//  Write the screenbuffer with changed to the screen
//
// void ssd1306_UpdateScreen(I2C_HandleTypeDef *hi2c)
void ssd1306_UpdateScreen(void)
{
	uint8_t txBuffer[SSD1306_WIDTH + 1]; // +1 for control byte

	txBuffer[0] = 0x40;                  // Control byte for data

	for (int i = 0; i < 8; i++)
	{
		ssd1306_WriteCommand(0xB0 + i);
		ssd1306_WriteCommand(0x00);
		ssd1306_WriteCommand(0x10);

		// Copy display data for the current page into txBuffer
		memcpy(&txBuffer[1], &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);

		// Use I2C_transmit to send the data buffer
		I2C_transmit(SSD1306_I2C_ADDR, txBuffer, SSD1306_WIDTH + 1);
		// HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, 0x40, 1, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 100);
	}
}

//  Draw one pixel in the screenbuffer
//  X => X Coordinate
//  Y => Y Coordinate
//  color => Pixel color
void ssd1306_DrawPixel(const unsigned int x, const unsigned int y, SSD1306_COLOR color)
{
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
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

//  Draw 1 char to the screen buffer
//  ch      => Character to write
//  Font    => Font to use
//  color   => Black or White
char ssd1306_WriteChar(const char ch, const t_font *font, const SSD1306_COLOR color)
{
	if (SSD1306_WIDTH <= (SSD1306.CurrentX + font->width) || SSD1306_HEIGHT <= (SSD1306.CurrentY + font->height))
		return 0;    // not enough space on current line

	for (unsigned int i = 0; i < font->height; i++)
	{
		const unsigned int b = font->data[(ch - 32) * font->height + i];
		for (unsigned int j = 0; j < font->width; j++)
			ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), ((b << j) & 0x8000) ? color : !color);
	}

	SSD1306.CurrentX += font->width;

	return ch;
}

//  Write full string to screenbuffer
char ssd1306_WriteString(const char *str, const t_font *font, const SSD1306_COLOR color)
{
	while (*str)
	{
		if (ssd1306_WriteChar(*str, font, color) != *str)
			return *str;   // Char could not be written
		str++;
	}

	return *str;
}

//  Invert background/foreground colors
void ssd1306_InvertColors(void)
{
	SSD1306.Inverted = !SSD1306.Inverted;
}

//  Set cursor position
void ssd1306_SetCursor(const uint16_t x, const uint16_t y)
{
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

void ssd1306_GetCursor(uint16_t *x, uint16_t *y)
{
	*x = SSD1306.CurrentX;
	*y = SSD1306.CurrentY;
}

void ssd1306_MoveCursor(const int x, const int y)
{
	SSD1306.CurrentX = (((int)SSD1306.CurrentX + x) <= 0) ? 0 : (((int)SSD1306.CurrentX + x) > SSD1306_WIDTH ) ? SSD1306_WIDTH  : SSD1306.CurrentX + x;
	SSD1306.CurrentY = (((int)SSD1306.CurrentY + y) <= 0) ? 0 : (((int)SSD1306.CurrentY + y) > SSD1306_HEIGHT) ? SSD1306_HEIGHT : SSD1306.CurrentY + y;
}

// Draw a filled rectangle
void ssd1306_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
	const unsigned int x_start = (x1 <= x2) ? x1 : x2;
	const unsigned int x_end   = (x1 <= x2) ? x2 : x1;
	const unsigned int y_start = (y1 <= y2) ? y1 : y2;
	const unsigned int y_end   = (y1 <= y2) ? y2 : y1;

	for (unsigned int y = y_start; y <= y_end && y < SSD1306_HEIGHT; y++)
		for (unsigned int x = x_start; x <= x_end && x < SSD1306_WIDTH; x++)
			ssd1306_DrawPixel(x, y, color);
}

// dotted line
void ssd1306_dotted_hline(const unsigned int x1, const unsigned int x2, const unsigned int x_step, const unsigned int y, const SSD1306_COLOR colour)
{
	for (unsigned int x = x1; x <= x2; x += x_step)
		ssd1306_DrawPixel(x, y, colour);
}
