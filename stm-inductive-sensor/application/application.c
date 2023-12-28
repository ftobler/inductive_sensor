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
static void process_data_sin();
static void process_data_cos();
static void process_data_final();

typedef struct {
	float sin;
	float cos;
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
	//measurement runs every 50ms and is also controlled through this timer
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	//timer17 is used to generate the TX coil signal.
	//the PWM is always active, but set to 0 for no output
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	htim17.Instance->CCR1 = 0; //capture compare (PWM value) / off - will be set to 50% of ARR when ON.
	htim17.Instance->ARR = 512; //auto reload register / this sets the TX frequency

	//calibrate the ADC
	//the adc is configured with DMA and set to 2 channel scan.
	//the actual channels are set in the measuring task.
    HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_Delay(10);

}

void appliation_loop() {
	if (task_start) {
		task_start = 0;
		main_task();
	}
}


/**
 * Do one measurement. Taskes ~40ms depending on how large the DMA buffer is.
 * Main time is spent in calculations, this is originally made for a STM32G030 without FPU.
 *
 * To prevent asymmetry in measurements, the ADC DMA is started 2 times. First to capture the sine signal and the tx
 * and then to capture the cosine and the tx. Tx is captured for a simplified lock-in amplifier. The lock-in is missing a
 * discrete sine/cosine path because in this application the phase is fixed and only the amplitude (with sign) is needed.
 *
 * The ADC is set to a relatively high sampling rate. It is critical that the two measured signals do not have a big
 * delay between them. Due to the high sampling rate, the DMA buffer is filled fast.
 */
void main_task() {
	ADC_ChannelConfTypeDef sConfig = {0};


	//// SINE CAPTURE ////

	//start the timer to generate PWM for the TX coil
	htim17.Instance->CCR1 = htim17.Instance->ARR / 2;
	//delay so the DC goes away
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


	//// COSINE CAPTURE ////

	//start the timer to generate PWM for the TX coil
	htim17.Instance->CCR1 = htim17.Instance->ARR / 2;
	//delay so the DC goes away
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


	//// FINISH AND OUTPUT MEASUREMENT ////

	//process all data
	process_data_final();

	//wait for next conversion
	//HAL_Delay(5);
	htim1.Instance->CCR1 = app.distance_filter * 10;

	static uint16_t distance = 0;
	distance = app.distance_filter;
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&distance, sizeof(distance));
}

/**
 * ISR
 * interrupt on ADC DMA complete.
 */
void application_dma_complete_isr() {
	dma_finish = 1;
}

/**
 * ISR
 * interrupt on timer1 complete and ready to start new measurement
 */
void application_timer_isr() {
	task_start = 1;
}


//macros to extract the right values from the DMA buffer
#define get_sin(x) (dma_buf[(x)*DATAWIDTH+1])
#define get_cos(x) (dma_buf[(x)*DATAWIDTH+1])
#define get_tx(x)  (dma_buf[(x)*DATAWIDTH+0])

//calibration to cancel out interference on the PCB by digitally mixing the TX signal in the receive paths.
//it no metal is near the sensor strip, the app.sin and app.cos variables should hover around 0.
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


