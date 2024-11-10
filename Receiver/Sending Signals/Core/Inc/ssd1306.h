#ifndef SSD1306_H
#define SSD1306_H 100

#ifdef __cplusplus
extern C {
#endif

#include "stm32f4xx_hal.h"
#include "fonts.h"
#include "stdlib.h"
#include "string.h"

#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR         0x78
#endif

#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH            128
#endif

#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT           64
#endif

typedef enum {
	SSD1306_COLOR_BLACK = 0x00,
	SSD1306_COLOR_WHITE = 0x01
} SSD1306_COLOR_t;

uint8_t SSD1306_Init(void);
void SSD1306_UpdateScreen(void);
void SSD1306_ToggleInvert(void);
void SSD1306_Fill(SSD1306_COLOR_t Color);
void SSD1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color);
void SSD1306_GotoXY(uint16_t x, uint16_t y);
char SSD1306_Putc(char ch, FontDef_t* Font, SSD1306_COLOR_t color);
char SSD1306_Puts(char* str, FontDef_t* Font, SSD1306_COLOR_t color);
void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR_t c);
void SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c);
void SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c);
void SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color);
void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c);
void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c);

#ifndef ssd1306_I2C_TIMEOUT
#define ssd1306_I2C_TIMEOUT					20000
#endif

void ssd1306_I2C_Init();
void ssd1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t data);
void ssd1306_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count);
void SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color);
void SSD1306_ScrollRight(uint8_t start_row, uint8_t end_row);
void SSD1306_ScrollLeft(uint8_t start_row, uint8_t end_row);
void SSD1306_Scrolldiagright(uint8_t start_row, uint8_t end_row);
void SSD1306_Scrolldiagleft(uint8_t start_row, uint8_t end_row);
void SSD1306_Stopscroll(void);
void SSD1306_InvertDisplay (int i);
void SSD1306_ShiftBufferLeft(void);
void SSD1306_DrawVerticalLineInRightmostColumn(uint8_t y1, uint8_t y2, SSD1306_COLOR_t color);
void SSD1306_Clear (void);

#ifdef __cplusplus
}
#endif

#endif
