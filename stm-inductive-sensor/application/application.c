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
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;

#define DATASIZE 900
#define DATAWIDTH 2
static uint16_t dma_buf[DATASIZE*DATAWIDTH];
static uint32_t dma_finish = 0;
static uint32_t task_start = 0;


static void main_task();
static void process_data();
static void process_data_sin();
static void process_data_cos();
static void process_data_final();

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
	//timer 1 is used for re-starting the measurement task and to output the result
	//on PWM channel 1 (PA8). 1ms <=> 1cm
	//measurement runs every 50ms
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	//timer17 is used to generate the TX coil signal.
	//the PWM is always active, but set to 0 for no output
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	htim17.Instance->CCR1 = 0;
	htim17.Instance->ARR = 512;

	//calibrate the ADC
    HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_Delay(10);

}

void appliation_loop() {
	if (task_start) {
		task_start = 0;
		main_task();
	}
}

void main_task() {
	ADC_ChannelConfTypeDef sConfig = {0};


	//start the timer to generate PWM for the TX coil
	htim17.Instance->CCR1 = htim17.Instance->ARR / 2;
	//delay very slightly so a few cycles pass
	volatile uint16_t i;
	i = 2500; while (i--);


	//configure ADC channels to capture
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		while(1);
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		while(1);
	//start the DMA transfer
	dma_finish = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_buf, DATASIZE*DATAWIDTH);
	//wait for the DMA finish interrupt (flag DMA_FLAG_TC1)
	while (dma_finish == 0);
	//stop the PWM
	htim17.Instance->CCR1 = 0;
	//finish calculations
	process_data_sin();


	//start the timer to generate PWM for the TX coil
	htim17.Instance->CCR1 = htim17.Instance->ARR / 2;
	//delay very slightly so a few cycles pass
	i = 2500; while (i--);

	//configure ADC channels to capture
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		while(1);
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		while(1);
	//start the DMA transfer
	dma_finish = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_buf, DATASIZE*DATAWIDTH);
	//wait for the DMA finish interrupt (flag DMA_FLAG_TC1)
	while (dma_finish == 0);
	//stop the PWM
	htim17.Instance->CCR1 = 0;
	//finish calculations
	process_data_cos();




	//process all data
	process_data_final();

	//wait for next conversion
	//HAL_Delay(5);
	htim1.Instance->CCR1 = app.distance_filter * 10;

	static uint16_t distance = 0;
	distance = app.distance_filter;
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&distance, sizeof(distance));
}

void application_dma_complete_isr() {
	dma_finish = 1;
}

void application_timer_isr() {
	task_start = 1;
}


//#define get_sin(x) (dma_buf[(x)*DATAWIDTH+2])
//#define get_cos(x) (dma_buf[(x)*DATAWIDTH+0])
//#define get_tx(x)  (dma_buf[(x)*DATAWIDTH+1])

#define get_sin(x) (dma_buf[(x)*DATAWIDTH+1])
#define get_cos(x) (dma_buf[(x)*DATAWIDTH+1])
#define get_tx(x)  (dma_buf[(x)*DATAWIDTH+0])

//for ARR=888
//float bias_cal_sin = 6.0f;
//float bias_cal_cos = 5.5f;
//float bias_cal_tx = 87.0f;

//for ARR=512
float coupling_cal_sin = 0.0f;   //to cancel out signal bleed in from TX
float coupling_cal_cos = 0.009f; //to cancel out signal bleed in from TX

__attribute__((optimize("-O3")))
static void process_data_sin() {

	float avg_sin = 0.0f;
	float avg_tx = 0.0f;

	float sum_sin = 0.0f;

	const int n = DATASIZE;

	for (int i = 0; i < n; i++) {
		avg_sin += get_sin(i);
		avg_tx  += get_tx(i);
	}
	avg_sin = avg_sin / n;
	avg_tx  = avg_tx  / n;

	for (int i = 0; i < n; i++) {
		float tx_hp  = get_tx(i)  - avg_tx;
		float sin_hp = get_sin(i) - avg_sin + tx_hp * coupling_cal_sin;
		sum_sin = sum_sin + sin_hp * tx_hp;
	}
	app.sin = sum_sin / n / 4096;

}

__attribute__((optimize("-O3")))
static void process_data_cos() {

	float avg_cos = 0.0f;
	float avg_tx = 0.0f;

	float sum_cos = 0.0f;

	const int n = DATASIZE;

	for (int i = 0; i < n; i++) {
		avg_cos += get_cos(i);
		avg_tx  += get_tx(i);
	}
	avg_cos = avg_cos / n;
	avg_tx  = avg_tx  / n;

	for (int i = 0; i < n; i++) {
		float tx_hp  = get_tx(i)  - avg_tx;
		float cos_hp = get_cos(i) - avg_cos + tx_hp * coupling_cal_cos;
		sum_cos = sum_cos + cos_hp * tx_hp;
	}
	app.cos = sum_cos / n / 4096;

}


static void process_data_final() {
	float angle_old = app.angle;
	app.angle = atan2(app.sin, app.cos);
	float angle_diff = app.angle - angle_old;
	if (angle_diff > 2.0f && app.n > 0) {
		app.n--;
	}
	if (angle_diff < -2.0f && app.n < 4) {
		app.n++;
	}
	app.distance = ((app.angle  / 2.0f / 3.14159f) + app.n) * 80.0f;
	app.distance_filter = app.distance_filter * 0.25f + app.distance * 0.75f;

	app.amplitude = sqrtf(app.sin*app.sin + app.cos*app.cos);
}









static void process_data() {

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
	avg_sin = avg_sin / n;
	avg_cos = avg_cos / n;
	avg_tx  = avg_tx  / n;

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
