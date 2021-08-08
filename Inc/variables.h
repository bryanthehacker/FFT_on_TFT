/*
 * variables.h
 *
 *  Created on: 18 ott 2020
 *      Author: Brian
 */

#ifndef INC_VARIABLES_H_
#define INC_VARIABLES_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <complex.h>
#include "stm32h7xx_hal.h"
#include "fonts.h"
#include "TFT.h"

#define FFT_POINTS (1 << 13)
#define x_grids 10.0
#define y_grids 10.0
#define bandwidth 20000.0
#define time_multiplier 1
#define ADC_multiplier 1
#define FDG_ENABLE 1
#define TDG_ENABLE 1

int timevalues[FFT_POINTS];
double complex fft_vectors[FFT_POINTS];
uint8_t tft_buffer[TFT_HEIGHT][TFT_WIDTH][2];
uint32_t countus;
uint16_t FPS;
char textbuffer[50];
uint32_t S_FRQ;

#endif /* INC_VARIABLES_H_ */
