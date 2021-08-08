/*
 * TFT.c
 *
 *  Created on: 18 ott 2020
 *      Author: Brian
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <complex.h>
#include "stm32h7xx_hal.h"
#include "fonts.h"
#include "TFT.h"
#include "variables.h"

void TFT_Select() {
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
}

void TFT_Unselect() {
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
}

void TFT_Reset() {
	HAL_GPIO_WritePin(TFT_RES_GPIO_Port, TFT_RES_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(TFT_RES_GPIO_Port, TFT_RES_Pin, GPIO_PIN_SET);
}

void TFT_WriteCommand(uint8_t cmd) {
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&TFT_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
}

void TFT_WriteData(uint8_t *buff, size_t buff_size) {
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	while (buff_size > 0) {
		uint16_t chunk_size = buff_size > 32768 ? 32768 : buff_size;
		HAL_SPI_Transmit(&TFT_SPI_PORT, buff, chunk_size, HAL_MAX_DELAY);
		buff += chunk_size;
		buff_size -= chunk_size;
	}
}

void TFT_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {

	TFT_WriteCommand(0x2A); // CASET
	{
		uint8_t data[] = { (x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1
				& 0xFF };
		TFT_WriteData(data, sizeof(data));
	}

	TFT_WriteCommand(0x2B); // RASET
	{
		uint8_t data[] = { (y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1
				& 0xFF };
		TFT_WriteData(data, sizeof(data));
	}

	TFT_WriteCommand(0x2C); // RAMWR
}

void TFT_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
	tft_buffer[y][x][0] = color >> 8;
	tft_buffer[y][x][1] = color & 0xFF;
}

void TFT_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font,
		uint16_t color, uint16_t bgcolor) {
	uint32_t i, b, j;

	for (i = 0; i < font.height; i++) {
		b = font.data[(ch - 32) * font.height + i];
		for (j = 0; j < font.width; j++)
			if ((b << j) & 0x8000)
				TFT_DrawPixel(j + x, i + y, color);
			else
				TFT_DrawPixel(j + x, i + y, bgcolor);
	}
}

void TFT_WriteString(uint16_t x, uint16_t y, char *str, FontDef font,
		uint16_t color, uint16_t bgcolor) {
	while (*str) {
		if (x + font.width >= TFT_WIDTH) {
			x = 0;
			y += font.height;
			if (y + font.height >= TFT_HEIGHT) {
				break;
			}

			if (*str == ' ') {
				// skip spaces in the beginning of the new line
				str++;
				continue;
			}
		}

		TFT_WriteChar(x, y, *str, font, color, bgcolor);
		x += font.width;
		str++;
	}
}

void TFT_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
		uint16_t color) {

}

void TFT_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint16_t color) {
	for (int xp = x; xp < x + w; xp++) {
		for (int yp = y; yp < y + h; yp++) {
			tft_buffer[yp][xp][0] = color >> 8;
			tft_buffer[yp][xp][1] = color & 0xFF;
		}
	}
}

void TFT_FillScreen(uint16_t color) {
	TFT_FillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, color);
}

void TFT_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		const uint16_t *data) {
	if ((x >= TFT_WIDTH) || (y >= TFT_HEIGHT))
		return;
	if ((x + w - 1) >= TFT_WIDTH)
		return;
	if ((y + h - 1) >= TFT_HEIGHT)
		return;

	TFT_Select();
	TFT_SetAddressWindow(x, y, x + w - 1, y + h - 1);
	TFT_WriteData((uint8_t*) data, sizeof(uint16_t) * w * h);
	TFT_Unselect();
}

void TFT_Update() {
	TFT_DrawImage(0, 0, TFT_WIDTH, TFT_HEIGHT, (uint16_t *) tft_buffer);
}

void TFT_Init() {
	TFT_Select();
	TFT_Reset();
	TFT_WriteCommand(0x01);
	HAL_Delay(1000);
	TFT_WriteCommand(0xCB);
	{
		uint8_t data[] = { 0x39, 0x2C, 0x00, 0x34, 0x02 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xCF);
	{
		uint8_t data[] = { 0x00, 0xC1, 0x30 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xE8);
	{
		uint8_t data[] = { 0x85, 0x00, 0x78 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xEA);
	{
		uint8_t data[] = { 0x00, 0x00 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xED);
	{
		uint8_t data[] = { 0x64, 0x03, 0x12, 0x81 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xF7);
	{
		uint8_t data[] = { 0x20 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xC0);
	{
		uint8_t data[] = { 0x23 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xC1);
	{
		uint8_t data[] = { 0x10 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xC5);
	{
		uint8_t data[] = { 0x3E, 0x28 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xC7);
	{
		uint8_t data[] = { 0x86 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0x36);
	{
		uint8_t data[] = { 0x48 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0x3A);
	{
		uint8_t data[] = { 0x55 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xB1);
	{
		uint8_t data[] = { 0x00, 0x18 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xB6);
	{
		uint8_t data[] = { 0x08, 0x82, 0x27 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xF2);
	{
		uint8_t data[] = { 0x00 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0x26);
	{
		uint8_t data[] = { 0x01 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xE0);
	{
		uint8_t data[] = { 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37,
				0x07, 0x10, 0x03, 0x0E, 0x09, 0x00 };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_WriteCommand(0xE1);
	{
		uint8_t data[] = { 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48,
				0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F };
		TFT_WriteData(data, sizeof(data));
	}

	TFT_WriteCommand(0x11);
	HAL_Delay(120);
	TFT_WriteCommand(0x29);
	TFT_WriteCommand(0x36);
	{
		uint8_t data[] = { TFT_ROTATION };
		TFT_WriteData(data, sizeof(data));
	}
	TFT_Unselect();
}
