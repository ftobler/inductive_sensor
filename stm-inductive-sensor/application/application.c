/*
 * application.c
 *
 *  Created on: Dec 27, 2023
 *      Author: ftobler
 */


#include "application.h"
#include "stm32_hal.h"
#include "stdint.h"
#include "string.h"
#include "math.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim17;

#define DATASIZE 512
#define DATAWIDTH 3
static uint16_t dma_buf[DATASIZE*DATAWIDTH];
static void process_data();
static uint32_t dma_finish = 0;

typedef struct {
	float sin;
	float cos;
	float rms_sin;
	float rms_cos;
	float rms_tx;
	float amplitude;
	float angle;
	float distance;
	float distance_filter;
	int n;
} App_t;

static volatile App_t app = {0};

void application_init() {
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
    HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_Delay(10);
	htim17.Instance->ARR = 512;

}

void appliation_loop() {
	//start the timer to generate PWM for the TX coil
	htim17.Instance->CCR1 = htim17.Instance->ARR / 2;

	//delay very slightly so a few cycles pass
	volatile uint16_t i;
	i = 2500; while (i--);

	//start the DMA transfer
	dma_finish = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_buf, DATASIZE*DATAWIDTH);

	//wait for the DMA finish interrupt (flag DMA_FLAG_TC1)
	while (dma_finish == 0);

	//stop the PWM
	htim17.Instance->CCR1 = 0;

	//process all data
	process_data();

	//wait for next conversion
	//HAL_Delay(5);
}

void application_dma_complete_isr() {
	dma_finish = 1;
}


#define get_sin(x) (dma_buf[(x)*DATAWIDTH+2])
#define get_cos(x) (dma_buf[(x)*DATAWIDTH+0])
#define get_tx(x)  (dma_buf[(x)*DATAWIDTH+1])

//for ARR=888
//float bias_cal_sin = 6.0f;
//float bias_cal_cos = 5.5f;
//float bias_cal_tx = 87.0f;

//for ARR=512
float coupling_cal_sin = 0.0f;   //to cancel out signal bleed in from TX
float coupling_cal_cos = 0.009f; //to cancel out signal bleed in from TX
float bias_cal_sin = 5.5f;
float bias_cal_cos = 1.5f;
float bias_cal_tx = 87.0f;

static void process_data() {
	//float sin_hp = 0.0f;
	//float cos_hp = 0.0f;
	//float tx_hp  = 0.0f;
	float sum_sin = 0.0f;
	float sum_cos = 0.0f;

	float avg_sin = 0.0f;
	float avg_cos = 0.0f;
	float avg_tx = 0.0f;

	float rms_sin = 0.0f;
	float rms_cos = 0.0f;
	float rms_tx  = 0.0f;

	const int n = DATASIZE;

	for (int i = 0; i < n; i++) {
		avg_sin += get_sin(i);
		avg_cos += get_cos(i);
		avg_tx  += get_tx(i);
	}
	avg_sin = avg_sin / n + bias_cal_sin;
	avg_cos = avg_cos / n + bias_cal_cos;
	avg_tx  = avg_tx  / n + bias_cal_tx;

	for (int i = 0; i < n; i++) {
		float tx_hp  = get_tx(i)  - avg_tx;
		float sin_hp = get_sin(i) - avg_sin + tx_hp * coupling_cal_sin;
		float cos_hp = get_cos(i) - avg_cos + tx_hp * coupling_cal_cos;

		sum_sin = sum_sin + sin_hp * tx_hp;
		sum_cos = sum_cos + cos_hp * tx_hp;

	    rms_sin += sin_hp * sin_hp;
		rms_cos += cos_hp * cos_hp;
		rms_tx  += tx_hp * tx_hp;
	}
	sum_sin = sum_sin / n / 4096;
	sum_cos = sum_cos / n / 4096;
	app.sin = sum_sin;
	app.cos = sum_cos;

	rms_sin = sqrtf(rms_sin / n);
	rms_cos = sqrtf(rms_cos / n);
	rms_tx  = sqrtf(rms_tx  / n);

	app.rms_sin = rms_sin * 0.1f + app.rms_sin * 0.9f;
	app.rms_cos = rms_cos * 0.1f + app.rms_cos * 0.9f;
	app.rms_tx  = rms_tx  * 0.1f + app.rms_tx  * 0.9f;

	float angle_old = app.angle;
	app.angle = atan2(sum_sin, sum_cos);
	float angle_diff = app.angle - angle_old;
	if (angle_diff > 2.0f && app.n > 0) {
	    app.n--;
	}
	if (angle_diff < -2.0f && app.n < 4) {
		app.n++;
	}
	app.distance = ((app.angle  / 2.0f / 3.14159f) + app.n) * 80.0f;
	app.distance_filter = app.distance_filter * 0.9f + app.distance * 0.1f;

	app.amplitude = sqrtf(sum_sin*sum_sin + sum_cos*sum_cos);
}
