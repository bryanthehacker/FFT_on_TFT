#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <complex.h>
#include "stm32h7xx_hal.h"
#include "fonts.h"
#include "variables.h"


double hamming(int n, int N) {
	return (0.54 - 0.46 * cos(2.0 * M_PI * (double) n / (double) (N - 1)));
}

uint32_t log_2(uint32_t N) {
	uint32_t k = N;
	uint32_t i = 0;
	while (k) {
		k >>= 1;
		i++;
	}
	return i - 1;
}

uint32_t ispowof2(uint32_t n) {
	return n > 0 && (n & (n - 1)) == 0;
}

uint32_t reverse(uint32_t N, uint32_t n) {
	uint32_t j, p = 0;
	for (j = 1; j <= log_2(N); j++) {
		if (n & (1 << (uint32_t) (log_2(N) - j)))
			p |= (1 << (j - 1));
	}
	return p;
}

void ordina(double complex f1[], uint32_t N) {
	double complex f2[N];
	for (uint32_t i = 0; i < N; i++)
		f2[i] = f1[reverse(N, i)];
	for (uint32_t j = 0; j < N; j++)
		f1[j] = f2[j];
}

void transform(double complex f[], uint32_t N) {
	ordina(f, N);
	double complex W[N / 2];
	W[1] = cos(-2.0 * M_PI / N) + sin(-2.0 * M_PI / N) * I;
	W[0] = 1.0 + 0.0 * I;
	for (uint32_t i = 2; i < N / 2; i++)
		W[i] = cpowf(W[1], i);
	uint32_t n = 1;
	uint32_t a = N / 2;
	for (uint32_t j = 0; j < log_2(N); j++) {
		for (uint32_t i = 0; i < N; i++) {
			if (!(i & n)) {
				double complex temp = f[i];
				double complex Temp = W[(i * a) % (n * a)] * f[i + n];
				f[i] = temp + Temp;
				f[i + n] = temp - Temp;
			}
		}
		n *= 2;
		a = a / 2;
	}
}

void FFT_start(double complex f[], uint32_t N, double d) {
	for (uint32_t n = 0; n < N; n++)
		f[n] *= hamming(n, N);
	transform(f, N);
	for (uint32_t n = 0; n < N; n++)
		f[n] *= d; //moltiplica il vettore per il passo in modo da avere il vettore trasformato effettivo
}

void FFT_print(int timefunction[], double complex FFT_vect[], uint32_t arraysizes, int samplefreq, uint32_t tft_height, uint32_t tft_width) {

	double magnitude, magnitudep = 0;

	//print FFT and signal
	if (TDG_ENABLE) {
		for (int x = 0; x < TFT_WIDTH && (int)(x * time_multiplier) < arraysizes; x++) {
			magnitude = (timefunction[x*time_multiplier] * (tft_height / 65535.0));
			magnitudep = (timefunction[(x + 1)*time_multiplier] * (tft_height / 65535.0));
			if (magnitude > (tft_height / 2) - 2)
				magnitude = (tft_height / 2) - 2;
			if (magnitudep > (tft_height / 2) - 2)
				magnitudep = (tft_height / 2) - 2;
			if (magnitude > magnitudep)
				for (int y = magnitude; y >= magnitudep - 1; y--)
					TFT_DrawPixel(x, (TFT_HEIGHT/2)-y, COLOR_GREEN);
			if (magnitude < magnitudep)
				for (int y = magnitudep; y >= magnitude - 1; y--)
					TFT_DrawPixel(x, (TFT_HEIGHT/2)-y, COLOR_GREEN);
			if (magnitude == magnitudep)
				TFT_DrawPixel(x, (TFT_HEIGHT/2)-(int)magnitude, COLOR_GREEN);
		}
	}

	if (FDG_ENABLE) {
		for (int x = 0; x < TFT_WIDTH && x < (arraysizes / 2); x++) {
			//uint16_t i = tft_width*log2(x+1)/log2(arraysizes / 2);
			//uint16_t i = pow(2, (x+1)*log2(arraysizes / 2)/tft_width) - 1;
			//uint16_t i1 = ((arraysizes/2.0)-1.0)*pow(10.0, (x - tft_width - 1.0)/(arraysizes/(2.0*64.0)));
			uint16_t i1 = x * (arraysizes * bandwidth / samplefreq) / (tft_width - 1.0);
			uint16_t i2 = (x + 1) * (arraysizes * bandwidth / samplefreq) / (tft_width - 1.0);
			if (i1 >= arraysizes) i1 = arraysizes - 1;
			if (i2 >= arraysizes) i2 = arraysizes - 1;

			magnitude =(tft_height / 6.0)* log10(1.0+ sqrt((creal(FFT_vect[i1])* creal(FFT_vect[i1]))+ (cimag(FFT_vect[i1])* cimag(FFT_vect[i1]))));
			magnitudep =(tft_height / 6.0)* log10(1.0+ sqrt((creal(FFT_vect[i2])* creal(FFT_vect[i2]))+ (cimag(FFT_vect[i2])* cimag(FFT_vect[i2]))));
			if (magnitude > tft_height - 2)
				magnitude = tft_height - 2;
			if (magnitudep > tft_height - 2)
				magnitudep = tft_height - 2;
			if (magnitude < 0.0)
				magnitude = 0;
			if (magnitudep < 0.0)
				magnitudep = 0;

			if (magnitude >= magnitudep)
				for (int y = magnitude; y >= magnitudep - 1; y--)
					TFT_DrawPixel(x, TFT_HEIGHT - 1 - y, COLOR_YELLOW);
			else
				for (int y = magnitudep; y >= magnitude - 1; y--)
					TFT_DrawPixel(x, TFT_HEIGHT - 1 - y, COLOR_YELLOW);
		}
	}



}


/*
 void fft_examples(uint32_t arraysizes, uint32_t tft_height, uint32_t tft_width)
 {
 for (uint16_t i = 0; i < arraysizes; i++) {
 //timevalues[i] = adc_read() - 32768;
 double t = i - (arraysizes / 2);
 timevalues[i] = 20000.0 * (cos(2.0 * M_PI * t / 100.0));
 fft_vectors[i] = timevalues[i] + 0.0 * I;
 }
 FFT_start(fft_vectors, arraysizes, T_SAMPLE);
 FFT_print(timevalues, fft_vectors, arraysizes, tft_height, tft_width);
 ILI9341_WriteString(0, 0, "s(t)=cos(wt)", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
 HAL_Delay(3000);


 for (uint16_t i = 0; i < arraysizes; i++) {
 double t = i - (arraysizes / 2);
 if(t == 0)
 timevalues[i] = 20000.0;
 else
 timevalues[i] = 20000.0 * (sin(2.0 * M_PI * t / 100.0)/(2.0 * M_PI * t / 100.0));
 fft_vectors[i] = timevalues[i] + 0.0 * I;
 }
 FFT_start(fft_vectors, arraysizes, T_SAMPLE);
 FFT_print(timevalues, fft_vectors, arraysizes, tft_height, tft_width);
 ILI9341_WriteString(0, 0, "s(t)=sinc(t)", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
 HAL_Delay(3000);


 for (uint16_t i = 0; i < arraysizes; i++) {
 double t = i - (arraysizes / 2);

 if(t > -100 && t < 100)
 timevalues[i] = 20000;
 else
 timevalues[i] = 0;

 fft_vectors[i] = timevalues[i] + 0.0 * I;
 }
 FFT_start(fft_vectors, arraysizes, T_SAMPLE);
 FFT_print(timevalues, fft_vectors, arraysizes, tft_height, tft_width);
 ILI9341_WriteString(0, 0, "s(t)=rect(t)", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
 HAL_Delay(3000);


 for (uint16_t i = 0; i < arraysizes; i++) {
 double t = i - (arraysizes / 2);
 timevalues[i] = 20000.0 * (cos(2.0 * M_PI * t / 100.0)*cos(2.0 * M_PI * t / 50.0));
 fft_vectors[i] = timevalues[i] + 0.0 * I;
 }
 FFT_start(fft_vectors, arraysizes, T_SAMPLE);
 FFT_print(timevalues, fft_vectors, arraysizes, tft_height, tft_width);
 ILI9341_WriteString(0, 0, "s(t)=cos(wt)cos(2wt)", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
 HAL_Delay(3000);


 for (uint16_t i = 0; i < arraysizes; i++) {
 double t = i - (arraysizes / 2);

 if(t > -25 && t < 25)
 timevalues[i] = 20000.0 * (cos(2.0 * M_PI * t / 100.0));
 else
 timevalues[i] = 0;

 fft_vectors[i] = timevalues[i] + 0.0 * I;
 }
 FFT_start(fft_vectors, arraysizes, T_SAMPLE);
 FFT_print(timevalues, fft_vectors, arraysizes, tft_height, tft_width);
 ILI9341_WriteString(0, 0, "s(t)=cos(wt)rect(t)", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
 HAL_Delay(3000);


 for (uint16_t i = 0; i < arraysizes; i++) {
 double t = i - (arraysizes / 2);

 if(t > 0)
 timevalues[i] = 20000.0;
 else
 timevalues[i] = 0;

 fft_vectors[i] = timevalues[i] + 0.0 * I;
 }
 FFT_start(fft_vectors, arraysizes, T_SAMPLE);
 FFT_print(timevalues, fft_vectors, arraysizes, tft_height, tft_width);
 ILI9341_WriteString(0, 0, "s(t)=u(t)", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
 HAL_Delay(3000);


 for (uint16_t i = 0; i < arraysizes; i++) {
 double t = i - (arraysizes / 2);

 if(t > -100 && t < 100)
 timevalues[i] = 20000 - 200 * sqrt(t*t);
 else
 timevalues[i] = 0;

 fft_vectors[i] = timevalues[i] + 0.0 * I;
 }
 FFT_start(fft_vectors, arraysizes, T_SAMPLE);
 FFT_print(timevalues, fft_vectors, arraysizes, tft_height, tft_width);
 ILI9341_WriteString(0, 0, "s(t)=triangle(t)", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
 HAL_Delay(3000);


 for (uint16_t i = 0; i < arraysizes; i++) {
 timevalues[i] = 20000;
 fft_vectors[i] = timevalues[i] + 0.0 * I;
 }
 FFT_start(fft_vectors, arraysizes, T_SAMPLE);
 FFT_print(timevalues, fft_vectors, arraysizes, tft_height, tft_width);
 ILI9341_WriteString(0, 0, "s(t)=c", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
 HAL_Delay(3000);
 }
 */
