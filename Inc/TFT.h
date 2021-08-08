/*
 * tft_buffer.h
 *
 *  Created on: 18 ott 2020
 *      Author: Brian
 */
#ifndef INC_TFT_H_
#define INC_TFT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <complex.h>
#include "stm32h7xx_hal.h"
#include "fonts.h"

#define TFT_MADCTL_MY  0x80
#define TFT_MADCTL_MX  0x40
#define TFT_MADCTL_MV  0x20
#define TFT_MADCTL_ML  0x10
#define TFT_MADCTL_RGB 0x00
#define TFT_MADCTL_BGR 0x08
#define TFT_MADCTL_MH  0x04

#define TFT_SPI_PORT hspi2
extern SPI_HandleTypeDef TFT_SPI_PORT;

#define TFT_RES_Pin       GPIO_PIN_0
#define TFT_RES_GPIO_Port GPIOB
#define TFT_CS_Pin        GPIO_PIN_12
#define TFT_CS_GPIO_Port  GPIOB
#define TFT_DC_Pin        GPIO_PIN_1
#define TFT_DC_GPIO_Port  GPIOB

#define TFT_WIDTH  320
#define TFT_HEIGHT 240
#define TFT_ROTATION (TFT_MADCTL_MV | TFT_MADCTL_BGR)

#define	COLOR_BLACK   0x0000
#define	COLOR_BLUE    0x001F
#define	COLOR_RED     0xF800
#define	COLOR_GREEN   0x07E0
#define COLOR_CYAN    0x07FF
#define COLOR_MAGENTA 0xF81F
#define COLOR_YELLOW  0xFFE0
#define COLOR_WHITE   0xFFFF
#define COLOR_GRAY	0x528A
#define COLOR_COLOR565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

void TFT_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void TFT_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);
void TFT_WriteString(uint16_t x, uint16_t y, char* str, FontDef font, uint16_t color, uint16_t bgcolor);
void TFT_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void TFT_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void TFT_FillScreen(uint16_t color);
void TFT_Update();
void TFT_Select();
void TFT_Unselect();
void TFT_Reset();
void TFT_Init();
void TFT_WriteCommand(uint8_t cmd);
void TFT_WriteData(uint8_t* buff, size_t buff_size);
void TFT_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void TFT_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);

#endif /* INC_TFT_H_ */
